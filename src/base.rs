//! Common types and functions.

use image::{Bgr, Bgra, ConvertBuffer, DynamicImage, ImageBuffer, Rgb, Rgba};

/// Contains width and height of a frame.
#[derive(Debug, Clone, Hash, PartialEq, Eq)]
pub struct Resolution {
    pub width: usize,
    pub height: usize,
}

/// Image type returned by [Frame::color_image](Frame::color_image).
#[derive(Debug, Clone)]
pub enum ColorImage<'a> {
    Bgr8(ImageBuffer<Bgr<u8>, &'a [u8]>),
    Bgra8(ImageBuffer<Bgra<u8>, &'a [u8]>),
    Rgb8(ImageBuffer<Rgb<u8>, &'a [u8]>),
    Rgba8(ImageBuffer<Rgba<u8>, &'a [u8]>),
}

impl<'a> From<ColorImage<'a>> for DynamicImage {
    fn from(from: ColorImage<'a>) -> DynamicImage {
        match from {
            ColorImage::Bgr8(image) => DynamicImage::ImageBgr8(image.convert()),
            ColorImage::Bgra8(image) => DynamicImage::ImageBgra8(image.convert()),
            ColorImage::Rgb8(image) => DynamicImage::ImageRgb8(image.convert()),
            ColorImage::Rgba8(image) => DynamicImage::ImageRgba8(image.convert()),
        }
    }
}
