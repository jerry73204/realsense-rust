//! Defines the type of device hubs.

use crate::{
    common::*,
    device::Device,
    error::{ErrorChecker, Result},
};

/// Represents a collection of devices.
#[derive(Debug)]
pub struct DeviceHub {
    pub(crate) ptr: NonNull<sys::rs2_device_hub>,
}

impl DeviceHub {
    /// Block and wait until a device is available.
    pub fn wait_for_device(&self) -> Result<Device> {
        let device = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr =
                sys::rs2_device_hub_wait_for_device(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            Device::from_raw(ptr)
        };
        Ok(device)
    }

    /// Check whether the given device is connected to the device hub.
    pub fn is_device_connected(&self, device: &Device) -> Result<bool> {
        let val = unsafe {
            let mut checker = ErrorChecker::new();
            let val = sys::rs2_device_hub_is_device_connected(
                self.ptr.as_ptr(),
                device.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            val
        };
        Ok(val != 0)
    }

    pub fn into_raw(self) -> *mut sys::rs2_device_hub {
        let ptr = self.ptr;
        mem::forget(self);
        ptr.as_ptr()
    }

    pub unsafe fn from_raw(ptr: *mut sys::rs2_device_hub) -> Self {
        Self {
            ptr: NonNull::new(ptr).unwrap(),
        }
    }
}

impl Drop for DeviceHub {
    fn drop(&mut self) {
        unsafe {
            sys::rs2_delete_device_hub(self.ptr.as_ptr());
        }
    }
}

unsafe impl Send for DeviceHub {}
