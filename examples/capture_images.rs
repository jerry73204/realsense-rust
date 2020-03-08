use failure::Fallible;
use image::{DynamicImage, ImageFormat};
use realsense_rust::{
    prelude::*, Config, Error as RsError, ExtendedFrame, Format, Pipeline, Resolution, StreamKind,
};
use std::time::Duration;

fn main() -> Fallible<()> {
    // init pipeline
    let pipeline = Pipeline::new()?;
    let config = Config::new()?
        .enable_stream(StreamKind::Depth, 0, 640, 0, Format::Z16, 30)?
        .enable_stream(StreamKind::Color, 0, 640, 0, Format::Rgb8, 30)?;
    let mut pipeline = pipeline.start(Some(config))?;
    let profile = pipeline.profile();

    // show stream info
    for (idx, stream_result) in profile.streams()?.try_into_iter()?.enumerate() {
        let stream = stream_result?;
        println!("stream data {}: {:#?}", idx, stream.get_data()?);
    }

    // process frames
    for _ in 0..16 {
        let timeout = Duration::from_millis(1000);
        let frames_result = pipeline.wait(Some(timeout));
        let frames = match frames_result {
            Err(RsError::Timeout(..)) => {
                println!("timeout error");
                continue;
            }
            result @ _ => result?,
        };

        println!("frame number = {}", frames.number()?);

        for frame_result in frames.try_into_iter()? {
            let frame_any = frame_result?;

            match frame_any.try_extend()? {
                ExtendedFrame::Video(frame) => {
                    let image: DynamicImage = frame.image()?.into();

                    image.save_with_format(
                        format!("sync-video-example-{}.png", frame.number()?),
                        ImageFormat::Png,
                    )?;
                }
                ExtendedFrame::Depth(frame) => {
                    let Resolution { width, height } = frame.resolution()?;
                    let distance = frame.distance(width / 2, height / 2)?;
                    println!("distance = {}", distance);

                    let image: DynamicImage = frame.image()?.into();
                    image.save_with_format(
                        format!("sync-depth-example-{}.png", frame.number()?),
                        ImageFormat::Png,
                    )?;
                }
                _ => unreachable!(),
            }
        }
    }

    Ok(())
}
