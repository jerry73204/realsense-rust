use lazy_static::lazy_static;
use realsense_rust::{
    base::Resolution,
    config::Config,
    context::Context,
    error::Result as RsResult,
    frame::marker::Depth,
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
fn async_test() -> RsResult<()> {
    // lock global mutex
    let mut counter = GLOBAL_MUTEX.lock().unwrap();

    // init async runtime
    let mut runtime = Runtime::new().unwrap();

    runtime.block_on(async {
        // init pipeline
        let pipeline = Pipeline::new()?;
        let config = Config::new()?.enable_stream(StreamKind::Depth, 0, 640, 0, Format::Z16, 30)?;
        let mut pipeline = pipeline.start_async(Some(config)).await?;
        let profile = pipeline.profile();

        // get stream info
        let stream = match profile.streams()?.try_into_iter()?.next().transpose()? {
            Some(stream) => stream,
            None => return Ok(()),
        };
        let stream_data = stream.get_data()?;
        let resolution = stream.resolution()?;
        println!("stream data = {:#?}", stream_data);
        println!("stream resolution = {:#?}", resolution);

        // process frames
        for _ in 0..100 {
            let (pipeline_returned, frames) = pipeline.wait_async().await?;
            pipeline = pipeline_returned;

            println!("frame number = {}", frames.number()?);
            for frame_result in frames.try_into_iter()? {
                let composite_frame = frame_result?;

                if let Ok(frame) = composite_frame.try_extend_to::<Depth>()? {
                    let Resolution { width, height } = resolution;
                    let distance = frame.get_distance(width / 2, height / 2)?;
                    println!("distance = {}", distance);
                }
            }
        }

        RsResult::Ok(())
    })?;

    *counter += 1;
    Ok(())
}

#[test]
fn depth_image_test() -> RsResult<()> {
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
    let config = Config::new()?.enable_stream(StreamKind::Depth, 0, 640, 0, Format::Z16, 30)?;
    let mut pipeline = pipeline.start(Some(config))?;
    let profile = pipeline.profile();

    // get stream info
    let stream = match profile.streams()?.try_into_iter()?.next().transpose()? {
        Some(stream) => stream,
        None => return Ok(()),
    };
    let stream_data = stream.get_data()?;
    let resolution = stream.resolution()?;
    println!("stream data = {:#?}", stream_data);
    println!("stream resolution = {:#?}", resolution);

    // process frames
    for _ in 0..100 {
        let frames = pipeline.wait(None)?;
        println!("frame number = {}", frames.number()?);
        for frame_result in frames.try_into_iter()? {
            let composite_frame = frame_result?;

            if let Ok(frame) = composite_frame.try_extend_to::<Depth>()? {
                let Resolution { width, height } = resolution;
                let distance = frame.get_distance(width / 2, height / 2)?;
                println!("distance = {}", distance);
            }
        }
    }

    *counter += 1;
    Ok(())
}
