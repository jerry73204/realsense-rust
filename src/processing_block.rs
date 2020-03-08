use crate::{
    error::{ErrorChecker, Result as RsResult},
    kind::Extension,
};
use std::{marker::PhantomData, ptr::NonNull};

pub mod marker {
    use super::*;

    pub trait ProcessingBlockKind {}
    pub trait NonAnyProcessingBlockKind
    where
        Self: ProcessingBlockKind,
    {
        const TYPE: Extension;
    }

    #[derive(Debug)]
    pub struct Any;
    impl ProcessingBlockKind for Any {}

    #[derive(Debug)]
    pub struct Decimation;
    impl ProcessingBlockKind for Decimation {}
    impl NonAnyProcessingBlockKind for Decimation {
        const TYPE: Extension = Extension::DecimationFilter;
    }

    #[derive(Debug)]
    pub struct Threshold;
    impl ProcessingBlockKind for Threshold {}
    impl NonAnyProcessingBlockKind for Threshold {
        const TYPE: Extension = Extension::ThresholdFilter;
    }

    #[derive(Debug)]
    pub struct Disparity;
    impl ProcessingBlockKind for Disparity {}
    impl NonAnyProcessingBlockKind for Disparity {
        const TYPE: Extension = Extension::DisparityFilter;
    }

    #[derive(Debug)]
    pub struct Spatial;
    impl ProcessingBlockKind for Spatial {}
    impl NonAnyProcessingBlockKind for Spatial {
        const TYPE: Extension = Extension::SpatialFilter;
    }

    #[derive(Debug)]
    pub struct Temporal;
    impl ProcessingBlockKind for Temporal {}
    impl NonAnyProcessingBlockKind for Temporal {
        const TYPE: Extension = Extension::TemporalFilter;
    }

    #[derive(Debug)]
    pub struct HoleFilling;
    impl ProcessingBlockKind for HoleFilling {}
    impl NonAnyProcessingBlockKind for HoleFilling {
        const TYPE: Extension = Extension::HoleFillingFilter;
    }

    #[derive(Debug)]
    pub struct ZeroOrder;
    impl ProcessingBlockKind for ZeroOrder {}
    impl NonAnyProcessingBlockKind for ZeroOrder {
        const TYPE: Extension = Extension::ZeroOrderFilter;
    }

}

#[derive(Debug)]
pub struct ProcessingBlock<Kind>
where
    Kind: marker::ProcessingBlockKind,
{
    ptr: NonNull<realsense_sys::rs2_processing_block>,
    _phantom: PhantomData<Kind>,
}

impl<Kind> ProcessingBlock<Kind>
where
    Kind: marker::ProcessingBlockKind,
{
    pub(crate) unsafe fn from_ptr(ptr: NonNull<realsense_sys::rs2_processing_block>) -> Self {
        Self {
            ptr,
            _phantom: PhantomData,
        }
    }
}

impl ProcessingBlock<marker::Spatial> {
    pub fn create() -> RsResult<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_spatial_filter_block(checker.inner_mut_ptr());
            checker.check()?;
            Self::from_ptr(NonNull::new(ptr).unwrap())
        };
        Ok(processing_block)
    }
}

impl ProcessingBlock<marker::Temporal> {
    pub fn create() -> RsResult<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_temporal_filter_block(checker.inner_mut_ptr());
            checker.check()?;
            Self::from_ptr(NonNull::new(ptr).unwrap())
        };
        Ok(processing_block)
    }
}

impl ProcessingBlock<marker::Decimation> {
    pub fn create() -> RsResult<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_decimation_filter_block(checker.inner_mut_ptr());
            checker.check()?;
            Self::from_ptr(NonNull::new(ptr).unwrap())
        };
        Ok(processing_block)
    }
}

impl ProcessingBlock<marker::HoleFilling> {
    pub fn create() -> RsResult<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_hole_filling_filter_block(checker.inner_mut_ptr());
            checker.check()?;
            Self::from_ptr(NonNull::new(ptr).unwrap())
        };
        Ok(processing_block)
    }
}

impl<Kind> Drop for ProcessingBlock<Kind>
where
    Kind: marker::ProcessingBlockKind,
{
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_processing_block(self.ptr.as_ptr());
        }
    }
}
