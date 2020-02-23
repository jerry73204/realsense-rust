use lazy_static::lazy_static;
use realsense_rust::{
    base::Resolution,
    config::Config,
    context::Context,
    error::Result as RsResult,
    frame::marker::Depth,
    kind::{Extension, Format, Rs2Option, StreamKind},
    pipeline::Pipeline,
};
use std::sync::Mutex;

lazy_static! {
    static ref GLOBAL_MUTEX: Mutex<usize> = Mutex::new(0);
}

#[test]
fn depth_image_test() -> RsResult<()> {
    let mut counter = GLOBAL_MUTEX.lock().unwrap();
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
                    let depth_unit_opt = if sensor.is_extendable_to(Extension::DepthSensor)? {
                        sensor.get_option(Rs2Option::DepthUnits)?
                    } else {
                        None
                    };
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

    let pipeline = Pipeline::from_context(context)?;
    let config = Config::new()?.enable_stream(StreamKind::Depth, 0, 640, 0, Format::Z16, 30)?;
    let pipeline = pipeline.start(Some(config))?;
    let profile = pipeline.profile();

    let stream = match profile.streams()?.try_into_iter()?.next().transpose()? {
        Some(stream) => stream,
        None => return Ok(()),
    };
    let stream_data = stream.get_data()?;
    let resolution = stream.resolution()?;
    println!("stream data = {:#?}", stream_data);
    println!("stream resolution = {:#?}", resolution);

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
