use std::{future, time::{Duration, Instant}};
use futures_concurrency::future::Race;
use packbytes::{FromBytes, ToBytes};
use uartcat::master::{Error, Master, Slave, Host, Mapping};

pub mod registers;

#[tokio::main]
async fn main() -> Result<(), Error> {
    let master = Master::new("/dev/ttyUSB1", 1_000_000) .expect("openning uart port");
    
    let task = async {
        let slave = master.slave(Host::Topological(0));
        
        println!("reset errors");
        slave.write(registers::target::MODE, registers::Mode::Off).await
            .expect("no answer from slave")
            .one().expect("wrong number of slaves");
        while slave.read(registers::current::STATUS).await?.one()?.fault() {}
        
        let mut mapping = Mapping::new();
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
            calibrate(&slave).await?;
//             println!("apply constant force");
//             constant_force(&slave).await?;
            println!("pd control loop");
            infinite_sine(&slave).await?;
            
            future::pending().await
        };
        let monitor = async {
            let stream = master.stream(buffer).await?;
            stream.send_read().await.unwrap();
            let rr = rerun::RecordingStreamBuilder::new("joint control").spawn().unwrap();

            loop {
                tokio::time::sleep(Duration::from_millis(10)).await;
                stream.send_read().await.unwrap();
//                 let data = stream.receive().await.unwrap().one().unwrap();
                let data = stream.get().await;
                
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


async fn calibrate(slave: &Slave<'_>) -> Result<(), Error> {
    slave.write(registers::target::MODE, registers::Mode::CalibrateFocContinuous).await?.one()?;
    loop {
        let status = slave.read(registers::current::STATUS).await;
        let Ok(status) = status else {continue};
        let status = status.one()?;
        if status.fault() {
            println!("calibration error {:?}", slave.read(registers::current::ERROR).await?.one()?);
            panic!("failed to calibrate");
        }
        if status.calibrated() {
            println!("calibration done");
            break;
        }
        tokio::time::sleep(Duration::from_millis(300)).await;
    }
    slave.write(registers::target::MODE, registers::Mode::Off).await?.one()?;
    Ok(())
}

async fn infinite_sine(slave: &Slave<'_>) -> Result<(), Error> {
    let gain_position = 10.;
    let gain_velocity = 10.;
    let inertia = 0.01;
    let rated_force = slave.read(registers::typical::RATED_FORCE).await?.one()?;
    
    slave.write(registers::target::LIMIT_POSITION, registers::Range {start: -0.5, stop: 0.5}).await?.one()?;
    slave.write(registers::target::LIMIT_VELOCITY, registers::Range {start: -1., stop: 1.}).await?.one()?;
    slave.write(registers::target::LIMIT_FORCE, registers::Range {start: -1.5*rated_force, stop: 1.5*rated_force}).await?.one()?;
    
    slave.write(registers::target::FORCE_CONSTANT, 0.).await?.one()?;
    slave.write(registers::target::FORCE_POSITION, -gain_position * gain_velocity * inertia).await?.one()?;
    slave.write(registers::target::FORCE_VELOCITY, -gain_velocity * inertia).await?.one()?;
    
    slave.write(registers::target::FORCE, 0.).await?.one()?;
    slave.write(registers::target::MODE, registers::Mode::Control).await?.one()?;
    
    let frequency = 0.3;
    let amplitude = 0.3;
    let offset = slave.read(registers::current::POSITION).await?.one()?;
    let trajectory = |t: f32| (t * frequency * 2. * std::f32::consts::PI).cos() * amplitude + offset;
    let start = Instant::now();
    loop {
        tokio::time::sleep(Duration::from_millis(10)).await;
        
        let t = start.elapsed().as_micros() as f32 * 1e-6;
        let dt = 1e-3;
        let target_position = trajectory(t);
        let target_velocity = (trajectory(t + dt) - target_position) / dt;
        dbg!(target_position);
        slave.write(registers::target::FORCE, 
            target_position * gain_position * gain_velocity * inertia
            + target_velocity * gain_velocity * inertia).await?.one()?;
    }
}

async fn constant_force(slave: &Slave<'_>) -> Result<(), Error> {
    let rated_force = slave.read(registers::typical::RATED_FORCE).await?.one()?;
    dbg!(rated_force);
    
    slave.write(registers::target::LIMIT_POSITION, registers::Range {start: -0.3, stop: 0.3}).await?.one()?;
    slave.write(registers::target::LIMIT_VELOCITY, registers::Range {start: -0.5, stop: 0.5}).await?.one()?;
    slave.write(registers::target::LIMIT_FORCE, registers::Range {start: -f32::INFINITY, stop: f32::INFINITY}).await?.one()?;
    
    slave.write(registers::target::FORCE_CONSTANT, 0.).await?.one()?;
    slave.write(registers::target::FORCE_POSITION, 0.).await?.one()?;
    slave.write(registers::target::FORCE_VELOCITY, 0.).await?.one()?;
    
    slave.write(registers::target::FORCE, rated_force * 1.5).await?.one()?;
    slave.write(registers::target::MODE, registers::Mode::Control).await?.one()?;
    
//     let start = Instant::now();
//     while start.elapsed() < Duration::from_secs(10) {
//         tokio::time::sleep(Duration::from_millis(300)).await;
//     }
//     slave.write(registers::target::MODE, registers::Mode::Off).await?.one()?;
    Ok(())
}
