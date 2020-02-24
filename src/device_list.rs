use crate::{
    device::Device,
    error::{ErrorChecker, Result as RsResult},
};
use std::{iter::FusedIterator, mem::MaybeUninit, os::raw::c_int, ptr::NonNull};

/// An iterable list of devices.
#[derive(Debug)]
pub struct DeviceList {
    ptr: NonNull<realsense_sys::rs2_device_list>,
}

impl DeviceList {
    /// Gets the device at given index.
    ///
    /// The method returns error if index is out of bound given by [DeviceList::len].
    pub fn get(&self, index: usize) -> RsResult<Device> {
        let device = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_device(
                self.ptr.as_ptr(),
                index as c_int,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Device::from_ptr(NonNull::new(ptr).unwrap())
        };
        Ok(device)
    }

    /// Gets the length of the list.
    pub fn len(&self) -> RsResult<usize> {
        let len = unsafe {
            let mut checker = ErrorChecker::new();
            let len =
                realsense_sys::rs2_get_device_count(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            len
        };
        Ok(len as usize)
    }

    /// Turns into [DeviceListIntoIter] instance that implements [IntoIterator] trait.
    pub fn try_into_iter(self) -> RsResult<DeviceListIntoIter> {
        let len = self.len()?;
        let ptr = unsafe { self.take() };
        let iter = DeviceListIntoIter {
            index: 0,
            len,
            ptr,
            fused: len == 0,
        };
        Ok(iter)
    }

    pub(crate) unsafe fn take(mut self) -> NonNull<realsense_sys::rs2_device_list> {
        let ptr = std::mem::replace(&mut self.ptr, { MaybeUninit::uninit().assume_init() });
        std::mem::forget(self);
        ptr
    }

    pub(crate) unsafe fn from_ptr(ptr: NonNull<realsense_sys::rs2_device_list>) -> Self {
        Self { ptr }
    }
}

impl IntoIterator for DeviceList {
    type Item = RsResult<Device>;
    type IntoIter = DeviceListIntoIter;

    fn into_iter(self) -> Self::IntoIter {
        self.try_into_iter().unwrap()
    }
}

impl Drop for DeviceList {
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_device_list(self.ptr.as_ptr());
        }
    }
}

#[derive(Debug)]
pub struct DeviceListIntoIter {
    index: usize,
    len: usize,
    ptr: NonNull<realsense_sys::rs2_device_list>,
    fused: bool,
}

impl Iterator for DeviceListIntoIter {
    type Item = RsResult<Device>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.fused {
            return None;
        }

        let ptr = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_device(
                self.ptr.as_ptr(),
                self.index as c_int,
                checker.inner_mut_ptr(),
            );
            match checker.check() {
                Ok(()) => ptr,
                Err(err) => {
                    self.fused = true;
                    return Some(Err(err));
                }
            }
        };

        self.index += 1;
        if self.index >= self.len {
            self.fused = true;
        }

        let device = Device {
            ptr: NonNull::new(ptr).unwrap(),
        };
        Some(Ok(device))
    }
}

impl FusedIterator for DeviceListIntoIter {}

unsafe impl Send for DeviceList {}

impl Drop for DeviceListIntoIter {
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_device_list(self.ptr.as_ptr());
        }
    }
}
