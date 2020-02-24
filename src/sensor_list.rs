use crate::{
    error::{ErrorChecker, Result as RsResult},
    sensor::{marker as sensor_marker, Sensor},
};
use std::{iter::FusedIterator, mem::MaybeUninit, os::raw::c_int, ptr::NonNull};

#[derive(Debug)]
pub struct SensorList {
    ptr: NonNull<realsense_sys::rs2_sensor_list>,
}

impl SensorList {
    pub fn get(&mut self, index: usize) -> RsResult<Self> {
        let sensor = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_sensor(
                self.ptr.as_ptr(),
                index as c_int,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Self::from_ptr(NonNull::new(ptr as *mut _).unwrap())
        };
        Ok(sensor)
    }

    pub fn len(&mut self) -> RsResult<usize> {
        let len = unsafe {
            let mut checker = ErrorChecker::new();
            let len =
                realsense_sys::rs2_get_sensors_count(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            len
        };
        Ok(len as usize)
    }

    pub fn try_into_iter(mut self) -> RsResult<SensorListIntoIter> {
        let len = self.len()?;
        let ptr = unsafe { self.take() };
        let iter = SensorListIntoIter {
            len,
            index: 0,
            ptr,
            fused: len == 0,
        };
        Ok(iter)
    }

    pub(crate) unsafe fn take(mut self) -> NonNull<realsense_sys::rs2_sensor_list> {
        let ptr = std::mem::replace(&mut self.ptr, { MaybeUninit::uninit().assume_init() });
        std::mem::forget(self);
        ptr
    }

    pub(crate) unsafe fn from_ptr(ptr: NonNull<realsense_sys::rs2_sensor_list>) -> Self {
        Self { ptr }
    }
}

impl IntoIterator for SensorList {
    type Item = RsResult<Sensor<sensor_marker::Any>>;
    type IntoIter = SensorListIntoIter;

    fn into_iter(self) -> Self::IntoIter {
        self.try_into_iter().unwrap()
    }
}

#[derive(Debug)]
pub struct SensorListIntoIter {
    len: usize,
    index: usize,
    ptr: NonNull<realsense_sys::rs2_sensor_list>,
    fused: bool,
}

impl Iterator for SensorListIntoIter {
    type Item = RsResult<Sensor<sensor_marker::Any>>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.fused {
            return None;
        }

        let ptr = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_sensor(
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

        let sensor = unsafe { Sensor::from_ptr(NonNull::new(ptr).unwrap()) };
        Some(Ok(sensor))
    }
}

impl FusedIterator for SensorListIntoIter {}

unsafe impl Send for SensorList {}

impl Drop for SensorList {
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_sensor_list(self.ptr.as_ptr());
        }
    }
}
