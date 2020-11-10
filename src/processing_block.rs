//! Defines the processing block type.

use crate::{
    base::StreamProfileData,
    common::*,
    error::{ErrorChecker, Result},
    frame::{AnyFrame, DepthFrame, ExtendedFrame, Frame, GenericFrameEx, PointsFrame, VideoFrame},
    frame_kind::FrameKind,
    frame_queue::FrameQueue,
    kind::{ColorScheme, HoleFillingMode, PersistenceControl, Rs2Option, StreamKind},
    options::ToOptions,
    processing_block_kind,
};

/// The type returned by [ProcessingBlock::<Any>::try_extend](ProcessingBlock::try_extend).
///
/// It enumerates all possible frame extensions. If the frame failed to
/// extend any one of the kind, it falls back to [ExtendedProcessingBlock::Other](ExtendedProcessingBlock::Other) variant.
#[derive(Debug)]
pub enum ExtendedProcessingBlock {
    DecimationFilter(DecimationFilter),
    ThresholdFilter(ThresholdFilter),
    DisparityFilter(DisparityFilter),
    SpatialFilter(SpatialFilter),
    TemporalFilter(TemporalFilter),
    HoleFillingFilter(HoleFillingFilter),
    ZeroOrderFilter(ZeroOrderFilter),
    Other(AnyProcessingBlock),
}

/// The type of data processing unit.
#[derive(Debug)]
pub struct ProcessingBlock<Kind>
where
    Kind: processing_block_kind::ProcessingBlockKind,
{
    pub(crate) ptr: NonNull<realsense_sys::rs2_processing_block>,
    queue: FrameQueue,
    _phantom: PhantomData<Kind>,
}

// type aliases

pub type DecimationFilter = ProcessingBlock<processing_block_kind::DecimationFilterKind>;
pub type ThresholdFilter = ProcessingBlock<processing_block_kind::ThresholdFilterKind>;
pub type DisparityFilter = ProcessingBlock<processing_block_kind::DisparityFilterKind>;
pub type SpatialFilter = ProcessingBlock<processing_block_kind::SpatialFilterKind>;
pub type TemporalFilter = ProcessingBlock<processing_block_kind::TemporalFilterKind>;
pub type HoleFillingFilter = ProcessingBlock<processing_block_kind::HoleFillingFilterKind>;
pub type ZeroOrderFilter = ProcessingBlock<processing_block_kind::ZeroOrderFilterKind>;
pub type PointCloud = ProcessingBlock<processing_block_kind::PointCloudKind>;
pub type YuyDecoder = ProcessingBlock<processing_block_kind::YuyDecoderKind>;
pub type UnitsTransform = ProcessingBlock<processing_block_kind::UnitsTransformKind>;
pub type Syncer = ProcessingBlock<processing_block_kind::SyncerKind>;
pub type Align = ProcessingBlock<processing_block_kind::AlignKind>;
pub type Colorizer = ProcessingBlock<processing_block_kind::ColorizerKind>;
pub type HuffmanDepthDecompress =
    ProcessingBlock<processing_block_kind::HuffmanDepthDecompressKind>;
pub type RatesPrinter = ProcessingBlock<processing_block_kind::RatesPrinterKind>;
pub type AnyProcessingBlock = ProcessingBlock<processing_block_kind::Any>;

impl<Kind> ProcessingBlock<Kind>
where
    Kind: processing_block_kind::ProcessingBlockKind,
{
    pub fn process<K>(&mut self, input: Frame<K>) -> Result<AnyFrame>
    where
        K: FrameKind,
    {
        unsafe {
            let frame_ptr = input.take();
            let mut checker = ErrorChecker::new();
            realsense_sys::rs2_process_frame(
                self.ptr.as_ptr(),
                frame_ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
        }
        let output = self.queue.wait(None)?;
        Ok(output)
    }

    pub async fn process_async<K>(&mut self, input: Frame<K>) -> Result<AnyFrame>
    where
        K: FrameKind,
    {
        unsafe {
            let frame_ptr = input.take();
            let mut checker = ErrorChecker::new();
            realsense_sys::rs2_process_frame(
                self.ptr.as_ptr(),
                frame_ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
        }
        let output = self.queue.wait_async(None).await?;
        Ok(output)
    }

    unsafe fn take(self) -> (NonNull<realsense_sys::rs2_processing_block>, FrameQueue) {
        let ptr = self.ptr;
        let queue = self.queue.unsafe_clone();
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

    pub(crate) unsafe fn new_from_ptr_and_capacity(
        ptr: NonNull<realsense_sys::rs2_processing_block>,
        capacity: usize,
    ) -> Result<Self> {
        let queue = FrameQueue::with_capacity(capacity)?;

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

    pub(crate) unsafe fn new_from_ptr(
        ptr: NonNull<realsense_sys::rs2_processing_block>,
    ) -> Result<Self> {
        Self::new_from_ptr_and_capacity(ptr, 1)
    }
}

impl AnyProcessingBlock {
    pub fn is_extendable_to<Kind>(&self) -> Result<bool>
    where
        Kind: processing_block_kind::ExtendableProcessingBlockKind,
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

    pub fn try_extend_to<Kind>(self) -> Result<std::result::Result<ProcessingBlock<Kind>, Self>>
    where
        Kind: processing_block_kind::ExtendableProcessingBlockKind,
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

    pub fn try_extend(self) -> Result<ExtendedProcessingBlock> {
        let frame_any = self;

        let frame_any =
            match frame_any.try_extend_to::<processing_block_kind::DecimationFilterKind>()? {
                Ok(frame) => return Ok(ExtendedProcessingBlock::DecimationFilter(frame)),
                Err(frame) => frame,
            };

        let frame_any =
            match frame_any.try_extend_to::<processing_block_kind::ThresholdFilterKind>()? {
                Ok(frame) => return Ok(ExtendedProcessingBlock::ThresholdFilter(frame)),
                Err(frame) => frame,
            };

        let frame_any =
            match frame_any.try_extend_to::<processing_block_kind::DisparityFilterKind>()? {
                Ok(frame) => return Ok(ExtendedProcessingBlock::DisparityFilter(frame)),
                Err(frame) => frame,
            };

        let frame_any =
            match frame_any.try_extend_to::<processing_block_kind::SpatialFilterKind>()? {
                Ok(frame) => return Ok(ExtendedProcessingBlock::SpatialFilter(frame)),
                Err(frame) => frame,
            };

        let frame_any =
            match frame_any.try_extend_to::<processing_block_kind::TemporalFilterKind>()? {
                Ok(frame) => return Ok(ExtendedProcessingBlock::TemporalFilter(frame)),
                Err(frame) => frame,
            };

        let frame_any =
            match frame_any.try_extend_to::<processing_block_kind::HoleFillingFilterKind>()? {
                Ok(frame) => return Ok(ExtendedProcessingBlock::HoleFillingFilter(frame)),
                Err(frame) => frame,
            };

        let frame_any =
            match frame_any.try_extend_to::<processing_block_kind::ZeroOrderFilterKind>()? {
                Ok(frame) => return Ok(ExtendedProcessingBlock::ZeroOrderFilter(frame)),
                Err(frame) => frame,
            };

        Ok(ExtendedProcessingBlock::Other(frame_any))
    }
}

impl ThresholdFilter {
    pub fn create() -> Result<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_threshold(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }

    pub fn with_options(min_dist: Option<f32>, max_dist: Option<f32>) -> Result<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_threshold(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };

        let options = processing_block.to_options()?;
        if let Some(dist) = min_dist {
            options[&Rs2Option::MinDistance].set_value(dist)?;
        }
        if let Some(dist) = max_dist {
            options[&Rs2Option::MaxDistance].set_value(dist)?;
        }

        Ok(processing_block)
    }
}

impl SpatialFilter {
    pub fn create() -> Result<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_spatial_filter_block(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }

    pub fn with_options(
        smooth_alpha: f32,
        smooth_delta: f32,
        magnitude: f32,
        hole_fill: f32,
    ) -> Result<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_temporal_filter_block(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };

        let options = processing_block.to_options()?;
        options[&Rs2Option::FilterSmoothAlpha].set_value(smooth_alpha)?;
        options[&Rs2Option::FilterSmoothDelta].set_value(smooth_delta)?;
        options[&Rs2Option::FilterMagnitude].set_value(magnitude)?;
        options[&Rs2Option::HolesFill].set_value(hole_fill)?;

        Ok(processing_block)
    }
}

impl TemporalFilter {
    pub fn create() -> Result<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_temporal_filter_block(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }

    pub fn with_options(
        smooth_alpha: f32,
        smooth_delta: f32,
        persistence_control: PersistenceControl,
    ) -> Result<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_temporal_filter_block(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };

        let options = processing_block.to_options()?;
        options[&Rs2Option::HolesFill].set_value(persistence_control as usize as f32)?;
        options[&Rs2Option::FilterSmoothAlpha].set_value(smooth_alpha)?;
        options[&Rs2Option::FilterSmoothDelta].set_value(smooth_delta)?;

        Ok(processing_block)
    }
}

impl DecimationFilter {
    pub fn create() -> Result<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_decimation_filter_block(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }

    pub fn with_options(magnitude: f32) -> Result<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_decimation_filter_block(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };

        let options = processing_block.to_options()?;
        options[&Rs2Option::FilterMagnitude].set_value(magnitude)?;

        Ok(processing_block)
    }
}

impl HoleFillingFilter {
    pub fn create() -> Result<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_hole_filling_filter_block(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }

    pub fn with_options(mode: HoleFillingMode) -> Result<Self> {
        let processing_block = Self::create()?;

        let options = processing_block.to_options()?;
        options[&Rs2Option::HolesFill].set_value(mode as usize as f32)?;

        Ok(processing_block)
    }
}

impl DisparityFilter {
    pub fn create() -> Result<Self> {
        Self::with_options(true)
    }

    pub fn with_options(transform_to_disparity: bool) -> Result<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_disparity_transform_block(
                transform_to_disparity as c_uchar,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }
}

impl PointCloud {
    pub fn create() -> Result<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_pointcloud(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }

    pub fn calculate(&mut self, depth_frame: DepthFrame) -> Result<PointsFrame> {
        let frame_any = self.process(depth_frame)?;
        match frame_any.try_extend()? {
            ExtendedFrame::Points(points_frame) => Ok(points_frame),
            ExtendedFrame::Composite(composite_frame) => {
                for result in composite_frame.try_into_iter()? {
                    let frame = result?;
                    if let Ok(points_frame) = frame.try_extend_to()? {
                        return Ok(points_frame);
                    }
                }
                unreachable!();
            }
            _ => unreachable!(),
        }
    }

    pub fn map_to(&mut self, color_frame: VideoFrame) -> Result<()> {
        let StreamProfileData {
            stream,
            format,
            index,
            ..
        } = color_frame.stream_profile()?.get_data()?;
        let options = self.to_options()?;
        options[&Rs2Option::StreamFilter].set_value(stream as realsense_sys::rs2_stream as f32)?;
        options[&Rs2Option::StreamFormatFilter]
            .set_value(format as realsense_sys::rs2_format as f32)?;
        options[&Rs2Option::StreamIndexFilter].set_value(index as f32)?;

        self.process(color_frame)?;
        Ok(())
    }

    pub async fn calculate_async(&mut self, depth_frame: DepthFrame) -> Result<PointsFrame> {
        let frame_any = self.process_async(depth_frame).await?;
        match frame_any.try_extend()? {
            ExtendedFrame::Points(points_frame) => Ok(points_frame),
            ExtendedFrame::Composite(composite_frame) => {
                for result in composite_frame.try_into_iter()? {
                    let frame = result?;
                    if let Ok(points_frame) = frame.try_extend_to()? {
                        return Ok(points_frame);
                    }
                }
                unreachable!();
            }
            _ => unreachable!(),
        }
    }

    pub async fn map_to_async(&mut self, color_frame: VideoFrame) -> Result<()> {
        let StreamProfileData {
            stream,
            format,
            index,
            ..
        } = color_frame.stream_profile()?.get_data()?;
        let options = self.to_options()?;
        options[&Rs2Option::StreamFilter].set_value(stream as realsense_sys::rs2_stream as f32)?;
        options[&Rs2Option::StreamFormatFilter]
            .set_value(format as realsense_sys::rs2_format as f32)?;
        options[&Rs2Option::StreamIndexFilter].set_value(index as f32)?;

        self.process_async(color_frame).await?;
        Ok(())
    }
}

impl YuyDecoder {
    pub fn create() -> Result<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_yuy_decoder(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }
}

impl UnitsTransform {
    pub fn create() -> Result<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_units_transform(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }
}

impl Syncer {
    pub fn create() -> Result<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_sync_processing_block(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }
}

impl Align {
    pub fn create(align_to: StreamKind) -> Result<Self> {
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

impl Colorizer {
    pub fn create() -> Result<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_colorizer(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }

    pub fn with_options(color_scheme: ColorScheme) -> Result<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_colorizer(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };

        let options = processing_block.to_options()?;
        options[&Rs2Option::ColorScheme].set_value(color_scheme as usize as f32)?;

        Ok(processing_block)
    }

    pub fn colorize(&mut self, depth_frame: DepthFrame) -> Result<VideoFrame> {
        let frame_any = self.process(depth_frame)?;
        let color_frame: VideoFrame = frame_any.try_extend_to()?.unwrap();
        Ok(color_frame)
    }
}

impl HuffmanDepthDecompress {
    pub fn create() -> Result<Self> {
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

impl RatesPrinter {
    pub fn create() -> Result<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_rates_printer_block(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }
}

impl<Kind> ToOptions for ProcessingBlock<Kind>
where
    Kind: processing_block_kind::ProcessingBlockKind,
{
    fn get_options_ptr(&self) -> NonNull<realsense_sys::rs2_options> {
        self.ptr.cast::<realsense_sys::rs2_options>()
    }
}

impl<Kind> Drop for ProcessingBlock<Kind>
where
    Kind: processing_block_kind::ProcessingBlockKind,
{
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_processing_block(self.ptr.as_ptr());
        }
    }
}

unsafe impl<Kind> Send for ProcessingBlock<Kind> where
    Kind: processing_block_kind::ProcessingBlockKind
{
}
