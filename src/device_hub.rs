//! Defines the type of device hubs.

use crate::{
    common::*,
    device::Device,
    error::{ErrorChecker, Result},
};

/// Represents a collection of devices.
#[derive(Debug)]
pub struct DeviceHub {
    pub(crate) ptr: NonNull<realsense_sys::rs2_device_hub>,
}

impl DeviceHub {
    /// Block and wait until a device is available.
    pub fn wait_for_device(&self) -> Result<Device> {
        let device = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_device_hub_wait_for_device(
                self.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Device::from_ptr(NonNull::new(ptr).unwrap())
        };
        Ok(device)
    }

    /// Check whether the given device is connected to the device hub.
    pub fn is_device_connected(&self, device: &Device) -> Result<bool> {
        let val = unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_device_hub_is_device_connected(
                self.ptr.as_ptr(),
                device.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            val
        };
        Ok(val != 0)
    }
}

impl Drop for DeviceHub {
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_device_hub(self.ptr.as_ptr());
        }
    }
}

unsafe impl Send for DeviceHub {}
