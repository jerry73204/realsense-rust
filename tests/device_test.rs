#![cfg(feature = "with-image")]
#![cfg(feature = "device-test")]

use failure::Fallible;
use image::{DynamicImage, ImageFormat};
use lazy_static::lazy_static;
use realsense_rust::{
    prelude::*, Config, Error as RsError, ExtendedFrame, Format, Pipeline, Resolution, StreamKind,
};
use std::{sync::Mutex, time::Duration};
use tokio::runtime::Runtime;

lazy_static! {
    /// this lock prevenst multiple tests control RealSense device concurrently
    static ref GLOBAL_MUTEX: Mutex<usize> = Mutex::new(0);
}

#[test]
fn async_test() -> Fallible<()> {
    // lock global mutex
    let mut counter = GLOBAL_MUTEX.lock().unwrap();

    // init async runtime
    let mut runtime = Runtime::new()?;

    runtime.block_on(async {
        // init pipeline
        let pipeline = Pipeline::new()?;
        let config = Config::new()?
            .enable_stream(StreamKind::Depth, 0, 640, 0, Format::Z16, 30)?
            .enable_stream(StreamKind::Color, 0, 640, 0, Format::Rgb8, 30)?;
        let mut pipeline = pipeline.start_async(Some(config)).await?;

        // show stream info
        let profile = pipeline.profile();
        for (idx, stream_result) in profile.streams()?.try_into_iter()?.enumerate() {
            let stream = stream_result?;
            println!("stream data {}: {:#?}", idx, stream.get_data()?);
        }

        // process frames
        for _ in 0..16 {
            let timeout = Duration::from_millis(1000);
            let frames_result = pipeline.wait_async(Some(timeout)).await;
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
                            format!("async-video-example-{}.png", frame.number()?),
                            ImageFormat::Png,
                        )?;
                    }
                    ExtendedFrame::Depth(frame) => {
                        let Resolution { width, height } = frame.resolution()?;
                        let distance = frame.distance(width / 2, height / 2)?;
                        println!("distance = {} meter", distance);

                        let image: DynamicImage = frame.image()?.into();
                        image.save_with_format(
                            format!("async-depth-example-{}.png", frame.number()?),
                            ImageFormat::Png,
                        )?;
                    }
                    _ => unreachable!(),
                }
            }
        }

        Fallible::Ok(())
    })?;

    *counter += 1;
    Ok(())
}

#[test]
fn sync_test() -> Fallible<()> {
    // lock global mutex
    let mut counter = GLOBAL_MUTEX.lock().unwrap();

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

    *counter += 1;
    Ok(())
}
