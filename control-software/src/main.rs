use std::{future, time::Duration};
use futures_concurrency::future::Race;
use packbytes::{FromBytes, ToBytes};
use uartcat::master::Host;

pub mod registers;

#[tokio::main]
async fn main() -> Result<(), uartcat::master::Error> {
    let master = uartcat::master::Master::new("/dev/ttyUSB1", 1_500_000) .expect("openning uart port");
    
    let task = async {
        let slave = master.slave(Host::Topological(0));
        
        println!("reset errors");
        slave.write(registers::target::MODE, registers::Mode::Off).await
            .expect("no answer from slave")
            .one().expect("wrong number of slaves");
        while slave.read(registers::current::STATUS).await?.one()?.fault() {}
        
        let mut mapping = uartcat::master::Mapping::new();
        let buffer = mapping.buffer::<State>().unwrap()
            .register(slave.address(), registers::current::ERROR)
            .register(slave.address(), registers::current::POSITION)
            .register(slave.address(), registers::current::VELOCITY)
            .register(slave.address(), registers::current::FORCE)
            .register(slave.address(), registers::current::CURRENTS)
            .register(slave.address(), registers::current::VOLTAGES)
            .build();
        
        mapping.configure(&slave).await.unwrap();
        
        let control = async {
            println!("calibrate");
            slave.write(registers::target::MODE, registers::Mode::CalibrateFocContinuous).await?.one()?;
            loop {
                let status = slave.read(registers::current::STATUS).await?.one()?;
                if status.fault() {
                    println!("calibration error {:?}", slave.read(registers::current::ERROR).await?.one()?);
                    panic!("failed to calibrate");
                }
                if status.calibrated() {break;}
                tokio::time::sleep(Duration::from_millis(300)).await;
            }
            
            println!("apply constant force");
            let rated_force = slave.read(registers::typical::RATED_FORCE).await?.one()?;
            dbg!(rated_force);
            slave.write(registers::target::LIMIT_POSITION, registers::Range {start: -f32::INFINITY, stop: f32::INFINITY}).await?.one()?;
            slave.write(registers::target::LIMIT_VELOCITY, registers::Range {start: -f32::INFINITY, stop: f32::INFINITY}).await?.one()?;
            slave.write(registers::target::LIMIT_FORCE, registers::Range {start: -f32::INFINITY, stop: f32::INFINITY}).await?.one()?;
            slave.write(registers::target::FORCE, rated_force * 0.8).await?.one()?;
            slave.write(registers::target::MODE, registers::Mode::Control).await?.one()?;
            
            future::pending().await
        };
        let monitor = async {
            let stream = master.stream(buffer).await?;
            stream.send_read().await.unwrap();
            let rr = rerun::RecordingStreamBuilder::new("joint control").spawn().unwrap();

            loop {
                tokio::time::sleep(Duration::from_millis(10)).await;
                stream.send_read().await.unwrap();
                let data = stream.receive().await?.one()?;
                
                rr.log("position", &rerun::Scalars::single(data.position)).unwrap();
                rr.log("velocity", &rerun::Scalars::single(data.velocity)).unwrap();
                rr.log("force", &rerun::Scalars::single(data.force)).unwrap();
                rr.log("currents", &rerun::Scalars::new(data.currents.phases.into_iter().map(|x| f32::from(x) * registers::CURRENT_UNIT))).unwrap();
                rr.log("voltages", &rerun::Scalars::new(data.voltages.phases.into_iter().map(|x| f32::from(x) * registers::VOLTAGE_UNIT))).unwrap();
            }
        };
        (control, monitor).race().await
    };
    let com = async {
        Ok(master.run().await?)
    };
    (task, com).race().await
}

#[derive(FromBytes, ToBytes, Debug)]
struct State {
    error: registers::ControlError,
    position: f32,
    velocity: f32,
    force: f32,
    currents: registers::Phases,
    voltages: registers::Phases,
}
