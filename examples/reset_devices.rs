use anyhow::{ensure, Result};
use realsense_rust::Context;

fn main() -> Result<()> {
    println!("Looking for RealSense devices");
    let devices = Context::new()?.query_devices(None)?;

    ensure!(!devices.is_empty()?, "No devices found");

    for result in devices {
        let device = result?;
        let name = device.name()?.unwrap();
        let sn = device.serial_number()?.unwrap();
        println!("Found {} SN {}", name, sn);
        device.hardware_reset()?;
    }

    Ok(())
}
