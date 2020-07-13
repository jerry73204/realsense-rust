//! Defines the profile type of pipeline.

use crate::{
    common::*,
    device::Device,
    error::{ErrorChecker, Result as RsResult},
    stream_profile_list::StreamProfileList,
};

#[derive(Debug)]
pub struct PipelineProfile {
    ptr: NonNull<realsense_sys::rs2_pipeline_profile>,
}

impl PipelineProfile {
    /// Gets corresponding device of pipeline.
    pub fn device(&self) -> RsResult<Device> {
        let ptr = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_pipeline_profile_get_device(
                self.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            ptr
        };

        let device = unsafe { Device::from_ptr(NonNull::new(ptr).unwrap()) };
        Ok(device)
    }

    /// Gets iterable list of streams of pipeline.
    pub fn streams(&self) -> RsResult<StreamProfileList> {
        let ptr = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_pipeline_profile_get_streams(
                self.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            ptr
        };

        let list = unsafe { StreamProfileList::from_ptr(NonNull::new(ptr).unwrap()) };
        Ok(list)
    }

    pub(crate) unsafe fn from_ptr(ptr: NonNull<realsense_sys::rs2_pipeline_profile>) -> Self {
        Self { ptr }
    }
}

impl Drop for PipelineProfile {
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_pipeline_profile(self.ptr.as_ptr());
        }
    }
}

unsafe impl Send for PipelineProfile {}
