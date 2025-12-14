use std::time::Duration;
use futures_concurrency::future::Race;
use uartcat::master::Host;

pub mod registers;

#[tokio::main]
async fn main() -> Result<(), uartcat::master::Error> {
    let master = uartcat::master::Master::new("/dev/ttyUSB1", 1_500_000) .expect("openning uart port");
    
    let task = async {
        let slave = master.slave(Host::Topological(0));
        
        let mut control = registers::Control::default();
        
        println!("reset errors");
        while slave.read(registers::current::STATUS).await 
                .expect("no answer from slave")
                .one().unwrap().fault() {
            control.set_reset(true);
            slave.write(registers::target::CONTROL, control).await?.one()?;
        }
        control.set_reset(false);
        
        println!("calibrate");
        control.set_power(true);
        control.set_calibrate(true);
        slave.write(registers::target::CONTROL, control).await?.one()?;
        loop {
            let status = slave.read(registers::current::STATUS).await?.one()?;
            dbg!(status);
            if status.calibrated() {break;}
            if status.fault() {
                println!("calibration error {:?}", slave.read(registers::current::ERROR).await?.one()?);
                panic!("failed to calibrate");
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
        control.set_calibrate(false);
        slave.write(registers::target::CONTROL, control).await?.one()?;
        
        println!("apply constant force");
        let rated_force = slave.read(registers::typical::RATED_FORCE).await?.one()?;
        slave.write(registers::target::FORCE, rated_force * 0.5).await?.one()?;
        loop {
            dbg!(
                slave.read(registers::current::ERROR).await?.one()?,
                slave.read(registers::current::POSITION).await?.one()?,
                slave.read(registers::current::FORCE).await?.one()?,
                slave.read(registers::current::CURRENTS).await?.one()?,
                slave.read(registers::current::VOLTAGES).await?.one()?,
                );
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    };
    let com = async {
        Ok(master.run().await?)
    };
    (task, com).race().await
}
