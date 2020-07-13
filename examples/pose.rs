use anyhow::{bail, Result};
use realsense_rust::Error as RsError;
use realsense_rust::{Format as RsFormat, StreamKind};
use std::time::Duration;

#[tokio::main]
async fn main() -> Result<()> {
    println!("Looking for RealSense devices");
    let ctx = realsense_rust::Context::new()?;
    let devices = ctx.query_devices(None)?;
    let dcount = devices.len()?;
    if dcount == 0 {
        bail!("No RS devices found");
    }

    let pipeline = realsense_rust::Pipeline::new()?;
    let config = realsense_rust::Config::new()?;
    let config = config.enable_stream(StreamKind::Pose, 0, 0, 0, RsFormat::_6Dof, 200)?;
    let mut pipeline = pipeline.start(Some(config))?;

    let profile = pipeline.profile();
    for (idx, stream_result) in profile.streams()?.try_into_iter()?.enumerate() {
        let stream = stream_result?;
        println!("stream data {}: {:#?}", idx, stream.get_data()?);
    }

    loop {
        let timeout = Duration::from_millis(1000);
        let frames_result = pipeline.wait_async(Some(timeout)).await;
        let frames = match frames_result {
            Err(RsError::Timeout(..)) => {
                println!("timeout error");
                continue;
            }
            result => result?,
        };
        let poseframe = frames.pose_frame()?.unwrap();
        let pose = poseframe.pose()?;
        println!("posedata: {:?}", pose);
    }
}
