//! Marker traits and types for [Pipeline](crate::pipeline::Pipeline).

use crate::{common::*, pipeline_profile::PipelineProfile};

/// Marker trait for pipeline marker types.
pub trait PipelineState {
    #[doc(hidden)]
    unsafe fn unsafe_clone(&self) -> Self;
}

/// A marker type indicating the [Pipeline](crate::pipeline::Pipeline) is started.
#[derive(Debug)]
pub struct Active {
    pub profile: PipelineProfile,
}

impl Active {
    pub fn into_raw_parts(self) -> *mut sys::rs2_pipeline_profile {
        let profile_ptr = unsafe { self.profile.unsafe_clone().into_raw() };
        mem::forget(self);
        profile_ptr
    }

    pub unsafe fn from_raw_parts(profile_ptr: *mut sys::rs2_pipeline_profile) -> Self {
        Self {
            profile: PipelineProfile::from_raw(profile_ptr),
        }
    }
}

impl PipelineState for Active {
    unsafe fn unsafe_clone(&self) -> Self {
        Self {
            profile: self.profile.unsafe_clone(),
        }
    }
}

/// A marker type indicating the [Pipeline](crate::pipeline::Pipeline) is stopped.
#[derive(Debug)]
pub struct Inactive;

impl PipelineState for Inactive {
    unsafe fn unsafe_clone(&self) -> Self {
        Self
    }
}
