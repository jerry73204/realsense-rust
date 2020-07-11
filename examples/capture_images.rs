#![cfg(feature = "with-image")]
#![cfg(feature = "with-nalgebra")]

use crossbeam::channel;
use anyhow::Result;
use image::{DynamicImage, ImageFormat};
use kiss3d::{
    light::Light,
    window::{State, Window},
};
use nalgebra::Point3;
use realsense_rust::{
    prelude::*, processing_block::marker as processing_block_marker, Config, Error as RsError,
    ExtendedFrame, Format, Pipeline, ProcessingBlock, Resolution, StreamKind,
};
use std::time::Duration;

#[derive(Debug)]
struct PcdVizState {
    rx: channel::Receiver<Vec<Point3<f32>>>,
    points: Option<Vec<Point3<f32>>>,
}

impl PcdVizState {
    pub fn new(rx: channel::Receiver<Vec<Point3<f32>>>) -> Self {
        let state = Self { rx, points: None };
        state
    }
}

impl State for PcdVizState {
    fn step(&mut self, window: &mut Window) {
        // try to receive recent points
        if let Ok(points) = self.rx.try_recv() {
            self.points = Some(points);
        };

        // draw axis
        window.draw_line(
            &Point3::origin(),
            &Point3::new(1.0, 0.0, 0.0),
            &Point3::new(1.0, 0.0, 0.0),
        );
        window.draw_line(
            &Point3::origin(),
            &Point3::new(0.0, 1.0, 0.0),
            &Point3::new(0.0, 1.0, 0.0),
        );
        window.draw_line(
            &Point3::origin(),
            &Point3::new(0.0, 0.0, 1.0),
            &Point3::new(0.0, 0.0, 1.0),
        );

        // draw points
        if let Some(points) = &self.points {
            for point in points.iter() {
                window.draw_point(point, &Point3::new(1.0, 1.0, 1.0));
            }
        }
    }
}

fn main() -> Result<()> {
    let (tx, rx) = channel::unbounded();

    std::thread::spawn(move || {
        let state = PcdVizState::new(rx);
        let mut window = Window::new("point cloud");
        window.set_light(Light::StickToCamera);
        window.render_loop(state);
    });

    // init pipeline
    let pipeline = Pipeline::new()?;
    let config = Config::new()?
        .enable_stream(StreamKind::Depth, 0, 640, 0, Format::Z16, 30)?
        .enable_stream(StreamKind::Color, 0, 640, 0, Format::Rgb8, 30)?;
    let mut pipeline = pipeline.start(Some(config))?;
    let profile = pipeline.profile();

    // pointcloud filter
    let mut pointcloud = ProcessingBlock::<processing_block_marker::PointCloud>::create()?;

    // show stream info
    for (idx, stream_result) in profile.streams()?.try_into_iter()?.enumerate() {
        let stream = stream_result?;
        println!("stream data {}: {:#?}", idx, stream.get_data()?);
    }

    // process frames
    for _ in 0..1000 {
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

        let color_frame = frames.color_frame()?.unwrap();
        let depth_frame = frames.depth_frame()?.unwrap();

        // save video frame
        {
            let image: DynamicImage = color_frame.image()?.into();
            image.save_with_format(
                format!("sync-video-example-{}.png", color_frame.number()?),
                ImageFormat::Png,
            )?;
        }

        // save depth frame
        {
            let Resolution { width, height } = depth_frame.resolution()?;
            let distance = depth_frame.distance(width / 2, height / 2)?;
            println!("distance = {}", distance);

            let image: DynamicImage = depth_frame.image()?.into();
            image.save_with_format(
                format!("sync-depth-example-{}.png", depth_frame.number()?),
                ImageFormat::Png,
            )?;
        }

        // compute point cloud
        pointcloud.map_to(color_frame.clone())?;
        let points_frame = pointcloud.calculate(depth_frame.clone())?;
        let points = points_frame
            .vertices()?
            .iter()
            .map(|vertex| {
                let [x, y, z] = vertex.xyz;
                Point3::new(x, y, z)
            })
            .collect::<Vec<_>>();

        if let Err(_) = tx.send(points) {
            break;
        }
    }

    Ok(())
}
