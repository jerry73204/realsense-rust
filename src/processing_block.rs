//! Defines the processing block type.

use crate::{
    base::StreamProfileData,
    common::*,
    error::{ErrorChecker, Result as RsResult},
    frame::{
        marker as frame_marker, AnyFrame, DepthFrame, ExtendedFrame, Frame, GenericFrameEx,
        PointsFrame, VideoFrame,
    },
    frame_queue::FrameQueue,
    kind::{ColorScheme, Extension, HoleFillingMode, PersistenceControl, Rs2Option, StreamKind},
    options::ToOptions,
};

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
    pub struct DecimationFilterKind;
    impl ProcessingBlockKind for DecimationFilterKind {}
    impl ExtendableProcessingBlockKind for DecimationFilterKind {
        const EXTENSION: Extension = Extension::DecimationFilter;
    }

    #[derive(Debug)]
    pub struct ThresholdFilterKind;
    impl ProcessingBlockKind for ThresholdFilterKind {}
    impl ExtendableProcessingBlockKind for ThresholdFilterKind {
        const EXTENSION: Extension = Extension::ThresholdFilter;
    }

    #[derive(Debug)]
    pub struct DisparityFilterKind;
    impl ProcessingBlockKind for DisparityFilterKind {}
    impl ExtendableProcessingBlockKind for DisparityFilterKind {
        const EXTENSION: Extension = Extension::DisparityFilter;
    }

    #[derive(Debug)]
    pub struct SpatialFilterKind;
    impl ProcessingBlockKind for SpatialFilterKind {}
    impl ExtendableProcessingBlockKind for SpatialFilterKind {
        const EXTENSION: Extension = Extension::SpatialFilter;
    }

    #[derive(Debug)]
    pub struct TemporalFilterKind;
    impl ProcessingBlockKind for TemporalFilterKind {}
    impl ExtendableProcessingBlockKind for TemporalFilterKind {
        const EXTENSION: Extension = Extension::TemporalFilter;
    }

    #[derive(Debug)]
    pub struct HoleFillingFilterKind;
    impl ProcessingBlockKind for HoleFillingFilterKind {}
    impl ExtendableProcessingBlockKind for HoleFillingFilterKind {
        const EXTENSION: Extension = Extension::HoleFillingFilter;
    }

    #[derive(Debug)]
    pub struct ZeroOrderFilterKind;
    impl ProcessingBlockKind for ZeroOrderFilterKind {}
    impl ExtendableProcessingBlockKind for ZeroOrderFilterKind {
        const EXTENSION: Extension = Extension::ZeroOrderFilter;
    }

    #[derive(Debug)]
    pub struct PointCloudKind;
    impl ProcessingBlockKind for PointCloudKind {}

    #[derive(Debug)]
    pub struct YuyDecoderKind;
    impl ProcessingBlockKind for YuyDecoderKind {}

    #[derive(Debug)]
    pub struct UnitsTransformKind;
    impl ProcessingBlockKind for UnitsTransformKind {}

    #[derive(Debug)]
    pub struct SyncerKind;
    impl ProcessingBlockKind for SyncerKind {}

    #[derive(Debug)]
    pub struct AlignKind;
    impl ProcessingBlockKind for AlignKind {}

    #[derive(Debug)]
    pub struct ColorizerKind;
    impl ProcessingBlockKind for ColorizerKind {}

    #[derive(Debug)]
    pub struct HuffmanDepthDecompressKind;
    impl ProcessingBlockKind for HuffmanDepthDecompressKind {}

    #[derive(Debug)]
    pub struct RatesPrinterKind;
    impl ProcessingBlockKind for RatesPrinterKind {}
}

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
    Kind: marker::ProcessingBlockKind,
{
    pub(crate) ptr: NonNull<realsense_sys::rs2_processing_block>,
    queue: FrameQueue,
    _phantom: PhantomData<Kind>,
}

// type aliases

pub type DecimationFilter = ProcessingBlock<marker::DecimationFilterKind>;
pub type ThresholdFilter = ProcessingBlock<marker::ThresholdFilterKind>;
pub type DisparityFilter = ProcessingBlock<marker::DisparityFilterKind>;
pub type SpatialFilter = ProcessingBlock<marker::SpatialFilterKind>;
pub type TemporalFilter = ProcessingBlock<marker::TemporalFilterKind>;
pub type HoleFillingFilter = ProcessingBlock<marker::HoleFillingFilterKind>;
pub type ZeroOrderFilter = ProcessingBlock<marker::ZeroOrderFilterKind>;
pub type PointCloud = ProcessingBlock<marker::PointCloudKind>;
pub type YuyDecoder = ProcessingBlock<marker::YuyDecoderKind>;
pub type UnitsTransform = ProcessingBlock<marker::UnitsTransformKind>;
pub type Syncer = ProcessingBlock<marker::SyncerKind>;
pub type Align = ProcessingBlock<marker::AlignKind>;
pub type Colorizer = ProcessingBlock<marker::ColorizerKind>;
pub type HuffmanDepthDecompress = ProcessingBlock<marker::HuffmanDepthDecompressKind>;
pub type RatesPrinter = ProcessingBlock<marker::RatesPrinterKind>;
pub type AnyProcessingBlock = ProcessingBlock<marker::Any>;

impl<Kind> ProcessingBlock<Kind>
where
    Kind: marker::ProcessingBlockKind,
{
    pub fn process<K>(&mut self, input: Frame<K>) -> RsResult<AnyFrame>
    where
        K: frame_marker::FrameKind,
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

    pub async fn process_async<K>(&mut self, input: Frame<K>) -> RsResult<AnyFrame>
    where
        K: frame_marker::FrameKind,
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
    ) -> RsResult<Self> {
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
    ) -> RsResult<Self> {
        Self::new_from_ptr_and_capacity(ptr, 1)
    }
}

impl AnyProcessingBlock {
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

        let frame_any = match frame_any.try_extend_to::<marker::DecimationFilterKind>()? {
            Ok(frame) => return Ok(ExtendedProcessingBlock::DecimationFilter(frame)),
            Err(frame) => frame,
        };

        let frame_any = match frame_any.try_extend_to::<marker::ThresholdFilterKind>()? {
            Ok(frame) => return Ok(ExtendedProcessingBlock::ThresholdFilter(frame)),
            Err(frame) => frame,
        };

        let frame_any = match frame_any.try_extend_to::<marker::DisparityFilterKind>()? {
            Ok(frame) => return Ok(ExtendedProcessingBlock::DisparityFilter(frame)),
            Err(frame) => frame,
        };

        let frame_any = match frame_any.try_extend_to::<marker::SpatialFilterKind>()? {
            Ok(frame) => return Ok(ExtendedProcessingBlock::SpatialFilter(frame)),
            Err(frame) => frame,
        };

        let frame_any = match frame_any.try_extend_to::<marker::TemporalFilterKind>()? {
            Ok(frame) => return Ok(ExtendedProcessingBlock::TemporalFilter(frame)),
            Err(frame) => frame,
        };

        let frame_any = match frame_any.try_extend_to::<marker::HoleFillingFilterKind>()? {
            Ok(frame) => return Ok(ExtendedProcessingBlock::HoleFillingFilter(frame)),
            Err(frame) => frame,
        };

        let frame_any = match frame_any.try_extend_to::<marker::ZeroOrderFilterKind>()? {
            Ok(frame) => return Ok(ExtendedProcessingBlock::ZeroOrderFilter(frame)),
            Err(frame) => frame,
        };

        Ok(ExtendedProcessingBlock::Other(frame_any))
    }
}

impl ThresholdFilter {
    pub fn create() -> RsResult<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_threshold(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }

    pub fn with_options(min_dist: Option<f32>, max_dist: Option<f32>) -> RsResult<Self> {
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
    pub fn create() -> RsResult<Self> {
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
    ) -> RsResult<Self> {
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
    pub fn create() -> RsResult<Self> {
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
    ) -> RsResult<Self> {
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
    pub fn create() -> RsResult<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_decimation_filter_block(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }

    pub fn with_options(magnitude: f32) -> RsResult<Self> {
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
    pub fn create() -> RsResult<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_hole_filling_filter_block(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }

    pub fn with_options(mode: HoleFillingMode) -> RsResult<Self> {
        let processing_block = Self::create()?;

        let options = processing_block.to_options()?;
        options[&Rs2Option::HolesFill].set_value(mode as usize as f32)?;

        Ok(processing_block)
    }
}

impl DisparityFilter {
    pub fn create() -> RsResult<Self> {
        Self::with_options(true)
    }

    pub fn with_options(transform_to_disparity: bool) -> RsResult<Self> {
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
    pub fn create() -> RsResult<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_pointcloud(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }

    pub fn calculate(&mut self, depth_frame: DepthFrame) -> RsResult<PointsFrame> {
        let frame_any = self.process(depth_frame)?;
        match frame_any.try_extend()? {
            ExtendedFrame::Points(points_frame) => Ok(points_frame),
            ExtendedFrame::Composite(composite_frame) => {
                for result in composite_frame.try_into_iter()? {
                    let frame = result?;
                    if let Ok(points_frame) = frame.try_extend_to::<frame_marker::Points>()? {
                        return Ok(points_frame);
                    }
                }
                unreachable!();
            }
            _ => unreachable!(),
        }
    }

    pub fn map_to(&mut self, color_frame: VideoFrame) -> RsResult<()> {
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

    pub async fn calculate_async(&mut self, depth_frame: DepthFrame) -> RsResult<PointsFrame> {
        let frame_any = self.process_async(depth_frame).await?;
        match frame_any.try_extend()? {
            ExtendedFrame::Points(points_frame) => Ok(points_frame),
            ExtendedFrame::Composite(composite_frame) => {
                for result in composite_frame.try_into_iter()? {
                    let frame = result?;
                    if let Ok(points_frame) = frame.try_extend_to::<frame_marker::Points>()? {
                        return Ok(points_frame);
                    }
                }
                unreachable!();
            }
            _ => unreachable!(),
        }
    }

    pub async fn map_to_async(&mut self, color_frame: VideoFrame) -> RsResult<()> {
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

impl UnitsTransform {
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

impl Syncer {
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

impl Align {
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

impl Colorizer {
    pub fn create() -> RsResult<Self> {
        let processing_block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_colorizer(checker.inner_mut_ptr());
            checker.check()?;
            Self::new_from_ptr(NonNull::new(ptr).unwrap())?
        };
        Ok(processing_block)
    }

    pub fn with_options(color_scheme: ColorScheme) -> RsResult<Self> {
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

    pub fn colorize(&mut self, depth_frame: DepthFrame) -> RsResult<VideoFrame> {
        let frame_any = self.process(depth_frame)?;
        let color_frame = frame_any.try_extend_to::<frame_marker::Video>()?.unwrap();
        Ok(color_frame)
    }
}

impl HuffmanDepthDecompress {
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

impl RatesPrinter {
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

impl<Kind> ToOptions for ProcessingBlock<Kind>
where
    Kind: marker::ProcessingBlockKind,
{
    fn get_options_ptr(&self) -> NonNull<realsense_sys::rs2_options> {
        self.ptr.cast::<realsense_sys::rs2_options>()
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

unsafe impl<Kind> Send for ProcessingBlock<Kind> where Kind: marker::ProcessingBlockKind {}
