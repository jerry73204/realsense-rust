use crate::{
    device::Device,
    error::{ErrorChecker, Result as RsResult},
};
#[cfg(any(windows))]
use std::os::windows::ffi::OsStrExt;
use std::ptr::NonNull;

#[derive(Debug)]
pub struct DeviceHub {
    pub(crate) ptr: NonNull<realsense_sys::rs2_device_hub>,
}

impl DeviceHub {
    pub fn wait_for_device(&self) -> RsResult<Device> {
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

    pub fn is_device_connected(&self, device: &Device) -> RsResult<bool> {
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
