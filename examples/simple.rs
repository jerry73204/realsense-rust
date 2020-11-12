use anyhow::Result;

#[cfg(all(feature = "with-image", feature = "with-nalgebra"))]
mod example {
    use anyhow::Result;
    use image::ImageFormat;
    use realsense_rust::{prelude::*, Config, Format, Pipeline, Resolution, StreamKind};
    use std::time::Duration;

    pub fn main() -> Result<()> {
        // create pipeline
        let pipeline = Pipeline::new()?;
        let config = Config::new()?
            .enable_stream(StreamKind::Depth, 0, 640, 0, Format::Z16, 30)?
            .enable_stream(StreamKind::Color, 0, 640, 0, Format::Rgb8, 30)?;
        let mut pipeline = pipeline.start(config)?;

        // process frames
        for _ in 0..1000 {
            let timeout = Duration::from_millis(1000);
            let frames = match pipeline.wait(timeout)? {
                Some(frames) => frames,
                None => {
                    println!("timeout error");
                    continue;
                }
            };

            println!("frame number = {}", frames.number()?);

            let color_frame = frames.color_frame()?.unwrap();
            let depth_frame = frames.depth_frame()?.unwrap();

            // save video frame
            {
                let image = color_frame.owned_image()?;
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

                let image = depth_frame.owned_image()?;
                image.save_with_format(
                    format!("sync-depth-example-{}.png", depth_frame.number()?),
                    ImageFormat::Png,
                )?;
            }
        }

        Ok(())
    }
}

#[cfg(all(feature = "with-image", feature = "with-nalgebra"))]
fn main() -> Result<()> {
    example::main()
}

#[cfg(not(all(feature = "with-image", feature = "with-nalgebra")))]
fn main() {
    panic!("please enable with-image and with-nalgebra features to run this example");
}
