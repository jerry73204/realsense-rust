use failure::Fallible;
use image::{DynamicImage, ImageFormat};
use lazy_static::lazy_static;
use realsense_rust::{
    base::Resolution,
    config::Config,
    context::Context,
    error::Result as RsResult,
    frame::marker::{Depth, Video},
    kind::{Format, StreamKind},
    pipeline::Pipeline,
    sensor::marker as sensor_marker,
};
use std::sync::Mutex;
use tokio::runtime::Runtime;

lazy_static! {
    // this lock prevenst multiple tests control RealSense device concurrently
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
        let profile = pipeline.profile();

        // show stream info
        for (idx, stream_result) in profile.streams()?.try_into_iter()?.enumerate() {
            let stream = stream_result?;
            println!("stream data {}: {:#?}", idx, stream.get_data()?);
        }

        // process frames
        for _ in 0..16 {
            let (pipeline_returned, frames) = pipeline.wait_async().await?;
            pipeline = pipeline_returned;

            println!("frame number = {}", frames.number()?);
            for frame_result in frames.try_into_iter()? {
                let frame_any = frame_result?;

                let frame_any = match frame_any.try_extend_to::<Depth>()? {
                    Ok(frame) => {
                        let Resolution { width, height } = frame.resolution()?;
                        let distance = frame.distance(width / 2, height / 2)?;
                        println!("distance = {}", distance);

                        let image = frame.depth_image()?;
                        image.save_with_format(
                            format!("depth-example-{}.png", frame.number()?),
                            ImageFormat::Png,
                        )?;
                        continue;
                    }
                    Err(frame) => frame,
                };

                let _frame_any = match frame_any.try_extend_to::<Video>()? {
                    Ok(frame) => {
                        let image: DynamicImage = frame.color_image()?.into();

                        image.save_with_format(
                            format!("video-example-{}.png", frame.number()?),
                            ImageFormat::Png,
                        )?;
                        continue;
                    }
                    Err(frame) => frame,
                };
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

    // find the depth unit
    let context = Context::new()?;
    let depth_unit_opt = context
        .query_devices(None)?
        .try_into_iter()?
        .map(|device_result| {
            let device = device_result?;
            let depth_units = device
                .query_sensors()?
                .try_into_iter()?
                .map(|sensor_result| {
                    let sensor = sensor_result?;
                    let depth_unit_opt = sensor
                        .try_extend_to::<sensor_marker::Depth>()?
                        .ok()
                        .map(|depth_sensor| depth_sensor.depth_units())
                        .transpose()?;
                    Ok(depth_unit_opt)
                })
                .flat_map(|result| result.transpose())
                .collect::<RsResult<Vec<_>>>()?;

            RsResult::Ok(depth_units)
        })
        .collect::<RsResult<Vec<_>>>()?
        .into_iter()
        .flat_map(|vec| vec)
        .next();

    let depth_unit = match depth_unit_opt {
        Some(unit) => unit,
        None => return Ok(()),
    };
    println!("depth unit = {}", depth_unit);

    // init pipeline
    let pipeline = Pipeline::from_context(context)?;
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
        let frames = pipeline.wait(None)?;

        println!("frame number = {}", frames.number()?);
        for frame_result in frames.try_into_iter()? {
            let frame_any = frame_result?;

            let frame_any = match frame_any.try_extend_to::<Depth>()? {
                Ok(frame) => {
                    let Resolution { width, height } = frame.resolution()?;
                    let distance = frame.distance(width / 2, height / 2)?;
                    println!("distance = {}", distance);

                    let image = frame.depth_image()?;
                    image.save_with_format(
                        format!("depth-example-{}.png", frame.number()?),
                        ImageFormat::Png,
                    )?;
                    continue;
                }
                Err(frame) => frame,
            };

            let _frame_any = match frame_any.try_extend_to::<Video>()? {
                Ok(frame) => {
                    let image: DynamicImage = frame.color_image()?.into();

                    image.save_with_format(
                        format!("video-example-{}.png", frame.number()?),
                        ImageFormat::Png,
                    )?;
                    continue;
                }
                Err(frame) => frame,
            };
        }
    }

    *counter += 1;
    Ok(())
}
