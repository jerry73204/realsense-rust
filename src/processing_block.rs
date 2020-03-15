//! Defines the processing block type.

use crate::{
    error::{ErrorChecker, Result as RsResult},
    frame::{marker as frame_marker, Frame},
    frame_queue::FrameQueue,
    kind::{Extension, StreamKind},
    options::ToOptions,
};
use std::{marker::PhantomData, mem::MaybeUninit, ptr::NonNull};

pub mod marker {
    use super::*;

    pub trait ProcessingBlockKind {}
    pub trait ExtendableProcessingBlockKind
    where
        Self: ProcessingBlockKind,
    {
        const EXTENSION: Extension;
    }

    #[derive(Debug)]
    pub struct Any;
    impl ProcessingBlockKind for Any {}

    #[derive(Debug)]
    pub struct DecimationFilter;
    impl ProcessingBlockKind for DecimationFilter {}
    impl ExtendableProcessingBlockKind for DecimationFilter {
        const EXTENSION: Extension = Extension::DecimationFilter;
    }

    #[derive(Debug)]
    pub struct ThresholdFilter;
    impl ProcessingBlockKind for ThresholdFilter {}
    impl ExtendableProcessingBlockKind for ThresholdFilter {
        const EXTENSION: Extension = Extension::ThresholdFilter;
    }

    #[derive(Debug)]
    pub struct DisparityFilter;
    impl ProcessingBlockKind for DisparityFilter {}
    impl ExtendableProcessingBlockKind for DisparityFilter {
        const EXTENSION: Extension = Extension::DisparityFilter;
    }

    #[derive(Debug)]
    pub struct SpatialFilter;
    impl ProcessingBlockKind for SpatialFilter {}
    impl ExtendableProcessingBlockKind for SpatialFilter {
        const EXTENSION: Extension = Extension::SpatialFilter;
    }

    #[derive(Debug)]
    pub struct TemporalFilter;
    impl ProcessingBlockKind for TemporalFilter {}
    impl ExtendableProcessingBlockKind for TemporalFilter {
        const EXTENSION: Extension = Extension::TemporalFilter;
    }

    #[derive(Debug)]
    pub struct HoleFillingFilter;
    impl ProcessingBlockKind for HoleFillingFilter {}
    impl ExtendableProcessingBlockKind for HoleFillingFilter {
        const EXTENSION: Extension = Extension::HoleFillingFilter;
    }

    #[derive(Debug)]
    pub struct ZeroOrderFilter;
    impl ProcessingBlockKind for ZeroOrderFilter {}
    impl ExtendableProcessingBlockKind for ZeroOrderFilter {
        const EXTENSION: Extension = Extension::ZeroOrderFilter;
    }

    #[derive(Debug)]
    pub struct PointCloud;
    impl ProcessingBlockKind for PointCloud {}

    #[derive(Debug)]
    pub struct YuyDecoder;
    impl ProcessingBlockKind for YuyDecoder {}

    #[derive(Debug)]
    pub struct UnitsTransform;
    impl ProcessingBlockKind for UnitsTransform {}

    #[derive(Debug)]
    pub struct Syncer;
    impl ProcessingBlockKind for Syncer {}

    #[derive(Debug)]
    pub struct Align;
    impl ProcessingBlockKind for Align {}

    #[derive(Debug)]
    pub struct Colorizer;
    impl ProcessingBlockKind for Colorizer {}

    #[derive(Debug)]
    pub struct HuffmanDepthDecompress;
    impl ProcessingBlockKind for HuffmanDepthDecompress {}

    #[derive(Debug)]
    pub struct RatesPrinter;
    impl ProcessingBlockKind for RatesPrinter {}
}

/// The type returned by [ProcessingBlock::<Any>::try_extend](ProcessingBlock::try_extend).
///
/// It enumerates all possible frame extensions. If the frame failed to
/// extend any one of the kind, it falls back to [ExtendedProcessingBlock::Other](ExtendedProcessingBlock::Other) variant.
#[derive(Debug)]
pub enum ExtendedProcessingBlock {
    DecimationFilter(ProcessingBlock<marker::DecimationFilter>),
    ThresholdFilter(ProcessingBlock<marker::ThresholdFilter>),
    DisparityFilter(ProcessingBlock<marker::DisparityFilter>),
    SpatialFilter(ProcessingBlock<marker::SpatialFilter>),
    TemporalFilter(ProcessingBlock<marker::TemporalFilter>),
    HoleFillingFilter(ProcessingBlock<marker::HoleFillingFilter>),
    ZeroOrderFilter(ProcessingBlock<marker::ZeroOrderFilter>),
    Other(ProcessingBlock<marker::Any>),
}

/// The type of data processing unit.
#[derive(Debug)]
pub struct ProcessingBlock<Kind>
where
    Kind: marker::ProcessingBlockKind,
{
    pub(crate) ptr: NonNull<realsense_sys::rs2_processing_block>,
    queue: FrameQueue,
    _phantom: PhantomData<Kind>,
}

impl<Kind> ProcessingBlock<Kind>
where
    Kind: marker::ProcessingBlockKind,
{
    pub fn process<K>(&mut self, input: Frame<K>) -> RsResult<Frame<frame_marker::Any>>
    where
        K: frame_marker::FrameKind,
    {
        unsafe {
            let mut checker = ErrorChecker::new();
            realsense_sys::rs2_process_frame(
                self.ptr.as_ptr(),
                input.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
        }
        let output = self.queue.wait(None)?;
        Ok(output)
    }

    pub async fn process_async<K>(&mut self, input: Frame<K>) -> RsResult<Frame<frame_marker::Any>>
    where
        K: frame_marker::FrameKind,
    {
        unsafe {
            let mut checker = ErrorChecker::new();
            realsense_sys::rs2_process_frame(
                self.ptr.as_ptr(),
                input.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
        }
        let output = self.queue.wait_async(None).await?;
        Ok(output)
    }

    unsafe fn take(mut self) -> (NonNull<realsense_sys::rs2_processing_block>, FrameQueue) {
        let ptr = self.ptr.clone();
        let queue = std::mem::replace(&mut self.queue, { MaybeUninit::uninit().assume_init() });
        std::mem::forget(self);
        (ptr, queue)
    }

    pub(crate) unsafe fn from_parts(
        ptr: NonNull<realsense_sys::rs2_processing_block>,
        queue: FrameQueue,
    ) -> Self {
        Self {
            ptr,
            queue,
            _phantom: PhantomData,
        }
    }

    pub(crate) unsafe fn new_from_ptr(
        ptr: NonNull<realsense_sys::rs2_processing_block>,
    ) -> RsResult<Self> {
        let queue = FrameQueue::with_capacity(1)?;

        // start processing
        {
            let mut checker = ErrorChecker::new();
            realsense_sys::rs2_start_processing_queue(
                ptr.as_ptr(),
                queue.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
        }

        let block = Self {
            ptr,
            queue,
            _phantom: PhantomData,
        };
        Ok(block)
    }
}

impl ProcessingBlock<marker::Any> {
    pub fn is_extendable_to<Kind>(&self) -> RsResult<bool>
    where
        Kind: marker::ExtendableProcessingBlockKind,
    {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_is_processing_block_extendable_to(
                self.ptr.as_ptr(),
                Kind::EXTENSION as realsense_sys::rs2_extension,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val != 0)
        }
    }

    pub fn try_extend_to<Kind>(self) -> RsResult<Result<ProcessingBlock<Kind>, Self>>
    where
        Kind: marker::ExtendableProcessingBlockKind,
    {
        unsafe {
            let is_extendable = self.is_extendable_to::<Kind>()?;
            if is_extendable {
                let (ptr, queue) = self.take();
                let block = ProcessingBlock::from_parts(ptr, queue);
                Ok(Ok(block))
            } else {
                Ok(Err(self))
            }
        }
    }

    pub fn try_extend(self) -> RsResult<ExtendedProcessingBlock> {
        let frame_any = self;

        let frame_any = match frame_any.try_extend_to::<marker::DecimationFilter>()? {
            Ok(frame) => return Ok(ExtendedProcessingBlock::DecimationFilter(frame)),
            Err(frame) => frame,
        };

        let frame_any = match frame_any.try_extend_to::<marker::ThresholdFilter>()? {
            Ok(frame) => return Ok(ExtendedProcessingBlock::ThresholdFilter(frame)),
            Err(frame) => frame,
        };

        let frame_any = match frame_any.try_extend_to::<marker::DisparityFilter>()? {
            Ok(frame) => return Ok(ExtendedProcessingBlock::DisparityFilter(frame)),
            Err(frame) => frame,
        };

        let frame_any = match frame_any.try_extend_to::<marker::SpatialFilter>()? {
            Ok(frame) => return Ok(ExtendedProcessingBlock::SpatialFilter(frame)),
            Err(frame) => frame,
        };

        let frame_any = match frame_any.try_extend_to::<marker::TemporalFilter>()? {
            Ok(frame) => return Ok(ExtendedProcessingBlock::TemporalFilter(frame)),
            Err(frame) => frame,
        };

        let frame_any = match frame_any.try_extend_to::<marker::HoleFillingFilter>()? {
            Ok(frame) => return Ok(ExtendedProcessingBlock::HoleFillingFilter(frame)),
            Err(frame) => frame,
        };

        let frame_any = match frame_any.try_extend_to::<marker::ZeroOrderFilter>()? {
            Ok(frame) => return Ok(ExtendedProcessingBlock::ZeroOrderFilter(frame)),
            Err(frame) => frame,
        };

        Ok(ExtendedProcessingBlock::Other(frame_any))
    }
}

impl ProcessingBlock<marker::SpatialFilter> {
    pub fn create() -> RsResult<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_spatial_filter_block(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }
}

impl ProcessingBlock<marker::TemporalFilter> {
    pub fn create() -> RsResult<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_temporal_filter_block(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }
}

impl ProcessingBlock<marker::DecimationFilter> {
    pub fn create() -> RsResult<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_decimation_filter_block(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }
}

impl ProcessingBlock<marker::HoleFillingFilter> {
    pub fn create() -> RsResult<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_hole_filling_filter_block(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }
}

impl ProcessingBlock<marker::YuyDecoder> {
    pub fn create() -> RsResult<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_yuy_decoder(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }
}

impl ProcessingBlock<marker::UnitsTransform> {
    pub fn create() -> RsResult<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_units_transform(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }
}

impl ProcessingBlock<marker::Syncer> {
    pub fn create() -> RsResult<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_sync_processing_block(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }
}

impl ProcessingBlock<marker::Align> {
    pub fn create(align_to: StreamKind) -> RsResult<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_align(
                align_to as realsense_sys::rs2_stream,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }
}

impl ProcessingBlock<marker::Colorizer> {
    pub fn create() -> RsResult<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_colorizer(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }
}

impl ProcessingBlock<marker::HuffmanDepthDecompress> {
    pub fn create() -> RsResult<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr =
                realsense_sys::rs2_create_huffman_depth_decompress_block(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }
}

impl ProcessingBlock<marker::RatesPrinter> {
    pub fn create() -> RsResult<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_rates_printer_block(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }
}

// impl<Kind> ToOptions for ProcessingBlock<Kind>
// where
//     Kind: marker::ProcessingBlockKind
// {
//     fn get_options_ptr(&self) -> NonNull<realsense_sys::rs2_options> {
//         self.ptr.cast::<realsense_sys::rs2_options>()
//     }
// }

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
