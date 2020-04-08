use failure::ensure;
use failure::Fallible;

fn main() -> Fallible<()> {
    println!("Looking for RealSense devices");
    let ctx = realsense_rust::Context::new()?;
    let devices = ctx.query_devices(None)?;
    let mut devices_found: bool = false;
    for device in devices {
        let device = device.unwrap();
        let name = device.name().unwrap().unwrap();
        let sn = device.serial_number().unwrap().unwrap();
        println!("Found {} SN {}", name, sn);
        devices_found = true;
        device.hardware_reset()?;
    }
    ensure!(devices_found, "No devices found");
    Ok(())
}
