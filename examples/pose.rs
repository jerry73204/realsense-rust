use anyhow::{bail, Result};
use realsense_rust::{Config, Context, Format as RsFormat, Pipeline, StreamKind};
use std::time::Duration;

#[tokio::main]
async fn main() -> Result<()> {
    println!("Looking for RealSense devices");
    let ctx = Context::new()?;
    let devices = ctx.query_devices(None)?;
    let dcount = devices.len()?;
    if dcount == 0 {
        bail!("No RS devices found");
    }

    let pipeline = Pipeline::new()?;
    let config = Config::new()?.enable_stream(StreamKind::Pose, 0, 0, 0, RsFormat::_6Dof, 200)?;
    let mut pipeline = pipeline.start(config)?;

    let profile = pipeline.profile();
    for (idx, stream_result) in profile.streams()?.try_into_iter()?.enumerate() {
        let stream = stream_result?;
        println!("stream data {}: {:#?}", idx, stream.get_data()?);
    }

    loop {
        let timeout = Duration::from_millis(1000);
        let frames = match pipeline.wait_async(timeout).await? {
            Some(frame) => frame,
            None => continue,
        };
        let pose_frame = frames.pose_frame()?.unwrap();
        let pose = pose_frame.pose()?;
        println!("pose data: {:?}", pose);
    }
}
