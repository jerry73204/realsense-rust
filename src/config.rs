//! Configuration type for [Pipeline](crate::pipeline::Pipeline).

use crate::{
    common::*,
    error::{ErrorChecker, Result as RsResult},
    kind::{Format, StreamKind},
};

/// The pipeline configuration that will be consumed by [Pipeline::start()](crate::pipeline::Pipeline::start).
#[derive(Debug)]
pub struct Config {
    pub(crate) ptr: NonNull<realsense_sys::rs2_config>,
}

impl Config {
    /// Create an instance.
    pub fn new() -> RsResult<Self> {
        let ptr = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_config(checker.inner_mut_ptr());
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
    ) -> RsResult<Self> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_config_enable_stream(
                self.ptr.as_ptr(),
                stream as realsense_sys::rs2_stream,
                index as c_int,
                width as c_int,
                height as c_int,
                format as realsense_sys::rs2_format,
                framerate as c_int,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            ptr
        };
        Ok(self)
    }

    pub(crate) unsafe fn unsafe_clone(&self) -> Self {
        Self { ptr: self.ptr }
    }
}

impl Drop for Config {
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_config(self.ptr.as_ptr());
        }
    }
}

unsafe impl Send for Config {}
