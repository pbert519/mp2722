use mp2722::{self, Mp2722Interface, Mp2722Registers};
use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    let i2c = linux_embedded_hal::I2cdev::new("/dev/i2c-0").unwrap();

    let interface = Mp2722Interface::new(i2c);

    let mut device = Mp2722Registers::new(interface);

    println!("MP2722 Status: \n {:?} \n {:?} \n {:?} \n {:?} \n {:?} \n {:?} \n", 
        device.status_17().read().unwrap(),
        device.status_18().read().unwrap(),
        device.status_19().read().unwrap(),
        device.status_20().read().unwrap(),
        device.status_21().read().unwrap(),
        device.status_22().read().unwrap()
    );

    Ok(())
}
