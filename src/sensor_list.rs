//! Defines the iterable list of sensors.

use crate::{
    common::*,
    error::{ErrorChecker, Result},
    sensor::{AnySensor, Sensor},
};

/// An iterable list of sensors.
#[derive(Debug)]
pub struct SensorList {
    ptr: NonNull<sys::rs2_sensor_list>,
}

impl SensorList {
    /// Gets the sensor instance at given index.
    ///
    /// It returns error if index is out of bound given by [SensorList::len].
    pub fn get(&mut self, index: usize) -> Result<AnySensor> {
        let sensor = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr =
                sys::rs2_create_sensor(self.ptr.as_ptr(), index as c_int, checker.inner_mut_ptr());
            checker.check()?;
            Sensor::from_raw(ptr as *mut _)
        };
        Ok(sensor)
    }

    /// Gets the number of sensors in list.
    pub fn len(&mut self) -> Result<usize> {
        let len = unsafe {
            let mut checker = ErrorChecker::new();
            let len = sys::rs2_get_sensors_count(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            len
        };
        Ok(len as usize)
    }

    /// Checks if the list is empty.
    pub fn is_empty(&mut self) -> Result<bool> {
        Ok(self.len()? == 0)
    }

    /// Turns into [SensorListIntoIter] iterable type.
    pub fn try_into_iter(mut self) -> Result<SensorListIntoIter> {
        let len = self.len()?;
        let ptr = self.into_raw();
        let iter = SensorListIntoIter {
            len,
            index: 0,
            ptr: NonNull::new(ptr).unwrap(),
            fused: len == 0,
        };
        Ok(iter)
    }

    pub fn into_raw(self) -> *mut sys::rs2_sensor_list {
        let ptr = self.ptr;
        mem::forget(self);
        ptr.as_ptr()
    }

    pub unsafe fn from_raw(ptr: *mut sys::rs2_sensor_list) -> Self {
        Self {
            ptr: NonNull::new(ptr).unwrap(),
        }
    }
}

impl IntoIterator for SensorList {
    type Item = Result<AnySensor>;
    type IntoIter = SensorListIntoIter;

    /// The method internally calls [SensorList::try_into_iter](SensorList::try_into_iter).
    ///
    /// # Panics
    /// It panics if [SensorList::try_into_iter](SensorList::try_into_iter) returns error.
    fn into_iter(self) -> Self::IntoIter {
        self.try_into_iter().unwrap()
    }
}

/// The iterator type returned by [SensorList::try_into_iter](SensorList::try_into_iter).
#[derive(Debug)]
pub struct SensorListIntoIter {
    len: usize,
    index: usize,
    ptr: NonNull<sys::rs2_sensor_list>,
    fused: bool,
}

impl Iterator for SensorListIntoIter {
    type Item = Result<AnySensor>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.fused {
            return None;
        }

        let ptr = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = sys::rs2_create_sensor(
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

        let sensor = unsafe { Sensor::from_raw(ptr) };
        Some(Ok(sensor))
    }
}

impl FusedIterator for SensorListIntoIter {}

unsafe impl Send for SensorList {}

impl Drop for SensorList {
    fn drop(&mut self) {
        unsafe {
            sys::rs2_delete_sensor_list(self.ptr.as_ptr());
        }
    }
}
