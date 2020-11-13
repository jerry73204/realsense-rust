//! Defines the profile type of pipeline.

use crate::{
    common::*,
    device::Device,
    error::{ErrorChecker, Result},
    stream_profile_list::StreamProfileList,
};

#[derive(Debug)]
pub struct PipelineProfile {
    pub(crate) ptr: NonNull<sys::rs2_pipeline_profile>,
}

impl PipelineProfile {
    /// Gets corresponding device of pipeline.
    pub fn device(&self) -> Result<Device> {
        let ptr = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr =
                sys::rs2_pipeline_profile_get_device(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            ptr
        };

        let device = unsafe { Device::from_raw(ptr) };
        Ok(device)
    }

    /// Gets iterable list of streams of pipeline.
    pub fn streams(&self) -> Result<StreamProfileList> {
        let ptr = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr =
                sys::rs2_pipeline_profile_get_streams(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            ptr
        };

        let list = unsafe { StreamProfileList::from_raw(ptr) };
        Ok(list)
    }

    pub fn into_raw(self) -> *mut sys::rs2_pipeline_profile {
        let ptr = self.ptr;
        mem::forget(self);
        ptr.as_ptr()
    }

    pub unsafe fn from_raw(ptr: *mut sys::rs2_pipeline_profile) -> Self {
        Self {
            ptr: NonNull::new(ptr).unwrap(),
        }
    }

    pub(crate) unsafe fn unsafe_clone(&self) -> Self {
        Self { ptr: self.ptr }
    }
}

impl Drop for PipelineProfile {
    fn drop(&mut self) {
        unsafe {
            sys::rs2_delete_pipeline_profile(self.ptr.as_ptr());
        }
    }
}

unsafe impl Send for PipelineProfile {}
