//! Configuration type for [Pipeline](crate::pipeline::Pipeline).

use crate::{
    base::TryIntoCowCStr,
    common::*,
    error::{ErrorChecker, Result},
    kind::{Format, StreamKind},
    pipeline::Pipeline,
    pipeline_kind::PipelineState,
    pipeline_profile::PipelineProfile,
};

/// The pipeline configuration that will be consumed by [Pipeline::start()](crate::pipeline::Pipeline::start).
#[derive(Debug)]
pub struct Config {
    pub(crate) ptr: NonNull<sys::rs2_config>,
}

impl Config {
    /// Create an instance.
    pub fn new() -> Result<Self> {
        let ptr = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = sys::rs2_create_config(checker.inner_mut_ptr());
            checker.check()?;
            ptr
        };
        let config = Self {
            ptr: NonNull::new(ptr).unwrap(),
        };
        Ok(config)
    }

    /// Enable data stream with attributes.
    pub fn enable_stream(
        self,
        stream: StreamKind,
        index: usize,
        width: usize,
        height: usize,
        format: Format,
        framerate: usize,
    ) -> Result<Self> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = sys::rs2_config_enable_stream(
                self.ptr.as_ptr(),
                stream as sys::rs2_stream,
                index as c_int,
                width as c_int,
                height as c_int,
                format as sys::rs2_format,
                framerate as c_int,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            ptr
        };
        Ok(self)
    }

    /// Enable all device streams explicitly.
    pub fn enable_all_streams(self) -> Result<Self> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = sys::rs2_config_enable_all_stream(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            ptr
        };
        Ok(self)
    }

    /// Enable device from a serial number.
    pub fn enable_device_from_serial<'a>(self, serial: impl TryIntoCowCStr<'a>) -> Result<Self> {
        let serial = serial.try_into_cow_cstr()?;
        unsafe {
            let mut checker = ErrorChecker::new();
            sys::rs2_config_enable_device(
                self.ptr.as_ptr(),
                serial.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
        }
        Ok(self)
    }

    /// Enable device from a file path.
    pub fn enable_device_from_file<'a>(self, file: impl TryIntoCowCStr<'a>) -> Result<Self> {
        let file = file.try_into_cow_cstr()?;
        unsafe {
            let mut checker = ErrorChecker::new();
            sys::rs2_config_enable_device_from_file(
                self.ptr.as_ptr(),
                file.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
        }
        Ok(self)
    }

    /// Enable all device streams explicitly.
    pub fn resolve<S>(&self, pipeline: &Pipeline<S>) -> Result<PipelineProfile>
    where
        S: PipelineState,
    {
        let profile = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = sys::rs2_config_resolve(
                self.ptr.as_ptr(),
                pipeline.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            PipelineProfile::from_raw(ptr)
        };
        Ok(profile)
    }

    pub fn into_raw(self) -> *mut sys::rs2_config {
        let ptr = self.ptr;
        mem::forget(self);
        ptr.as_ptr()
    }

    pub unsafe fn from_raw(ptr: *mut sys::rs2_config) -> Self {
        Self {
            ptr: NonNull::new(ptr).unwrap(),
        }
    }
}

impl Drop for Config {
    fn drop(&mut self) {
        unsafe {
            sys::rs2_delete_config(self.ptr.as_ptr());
        }
    }
}

unsafe impl Send for Config {}
