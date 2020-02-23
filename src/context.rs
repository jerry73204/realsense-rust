use crate::{
    device_hub::DeviceHub,
    device_list::DeviceList,
    error::{ErrorChecker, Result as RsResult},
};
#[cfg(any(unix))]
use std::os::unix::ffi::OsStrExt;
#[cfg(any(windows))]
use std::os::windows::ffi::OsStrExt;
use std::{ffi::CString, os::raw::c_int, path::Path, ptr::NonNull};

#[derive(Debug)]
pub struct Context {
    pub(crate) ptr: NonNull<realsense_sys::rs2_context>,
}

impl Context {
    pub fn new() -> RsResult<Self> {
        let ptr = {
            let mut checker = ErrorChecker::new();
            let context = unsafe {
                realsense_sys::rs2_create_context(
                    realsense_sys::RS2_API_VERSION as i32,
                    checker.inner_mut_ptr(),
                )
            };
            checker.check()?;
            context
        };

        let context = Self {
            ptr: NonNull::new(ptr).unwrap(),
        };

        Ok(context)
    }

    pub fn create_device_hub(&self) -> RsResult<DeviceHub> {
        let ptr = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr =
                realsense_sys::rs2_create_device_hub(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            ptr
        };

        let hub = DeviceHub {
            ptr: NonNull::new(ptr).unwrap(),
        };
        Ok(hub)
    }

    pub fn query_devices(&self, product_mask: Option<c_int>) -> RsResult<DeviceList> {
        let list = match product_mask {
            Some(mask) => unsafe {
                let mut checker = ErrorChecker::new();
                let list = realsense_sys::rs2_query_devices_ex(
                    self.ptr.as_ptr(),
                    mask,
                    checker.inner_mut_ptr(),
                );
                checker.check()?;
                DeviceList::from_ptr(NonNull::new(list).unwrap())
            },
            None => unsafe {
                let mut checker = ErrorChecker::new();
                let list =
                    realsense_sys::rs2_query_devices(self.ptr.as_ptr(), checker.inner_mut_ptr());
                checker.check()?;
                DeviceList::from_ptr(NonNull::new(list).unwrap())
            },
        };

        Ok(list)
    }

    pub fn add_device<P>(&mut self, file: P) -> RsResult<()>
    where
        P: AsRef<Path>,
    {
        let cstring = CString::new(file.as_ref().as_os_str().as_bytes()).unwrap();
        unsafe {
            let mut checker = ErrorChecker::new();
            realsense_sys::rs2_context_add_device(
                self.ptr.as_ptr(),
                cstring.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
        }
        Ok(())
    }

    pub fn remove_device<P>(&mut self, file: P) -> RsResult<()>
    where
        P: AsRef<Path>,
    {
        todo!();
    }
}

impl Drop for Context {
    fn drop(&mut self) {
        unsafe { realsense_sys::rs2_delete_context(self.ptr.as_ptr()) }
    }
}

unsafe impl Send for Context {}
