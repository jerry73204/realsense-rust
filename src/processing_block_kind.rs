use crate::{common::*, kind::Extension};

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
