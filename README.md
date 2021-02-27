# (2021/02/27) RealSense-Rust is moved Tangram Vision's [Gitlab](https://gitlab.com/tangram-vision-oss/realsense-rust)

RealSense-Rust is moved to Gitlab and now is maintained by Tangram Vision.
The Tangram Vision has a full-time team to continue the development
and take care of future issues. The project always keeps open source
and allows community contributions. I believe the migration keeps
the project alive and prosper.

The project's new home:

https://gitlab.com/tangram-vision-oss/realsense-rust

# RealSense bindings for Rust

The project provides high-level bindings to librealsense2 library as well as low-level FFI interface.
It supports asynchronous API and integration with [image](https://github.com/image-rs/image) and [nalgebra](https://github.com/rustsim/nalgebra) types.

## Use this crate in your project

Make sure **librealsense 2.39.0** is installed on your system. You may visit [RealSense official repository](https://github.com/IntelRealSense/librealsense).

Add this crate to your `Cargo.toml`.

```toml
[dependencies]
realsense-rust = "0.5"
```

If you're using older librealsense for reasons. You may enable `buildtime-bindgen` to re-generate bindings and good luck.

```toml
[dependencies]
realsense-rust = { version = "0.5", features = ["buildtime-bindgen"] }
```

## Cargo Features

The crate enables **with-nalgebra** and **with-image** features by default.

- **with-nalgebra** (default): Enable [nalgebra](https://github.com/rustsim/nalgebra) support.
- **with-image** (default): Enable [image](https://github.com/image-rs/image) support.
- **buildtime-bindgen**: Generate Rust bindings during build time.
- **device-test**: Enable tests that requires connections to RealSense devices.

## Get Started

You can start by `Pipeline`. This is the minimal example to capture color and depth images.

```rust
use anyhow::Result;
use realsense_rust::{Config, Format, Pipeline, StreamKind};

fn main() -> anyhow::Result<()> {
    let pipeline = Pipeline::new()?;
    let config = Config::new()?
        .enable_stream(StreamKind::Depth, 0, 640, 0, Format::Z16, 30)?
        .enable_stream(StreamKind::Color, 0, 640, 0, Format::Rgb8, 30)?;
    let mut pipeline = pipeline.start(&config)?;

    let frames = pipeline.wait(None)?.unwrap();
    let color_frame = frames.color_frame()?.unwrap();
    let depth_frame = frames.depth_frame()?.unwrap();

    Ok(())
}
```

## Examples

To capture image with your RealSense device,


```rust
cargo run --release --example capture_images
```

More examples can be found in [examples](examples) directory.

## Develop this project

### Work with realsense-sys low-level API

The realsense-sys crate provides C bindings generated from librealsense headers. The reference can be found on RealSense official [documentation](https://github.com/IntelRealSense/librealsense/tree/master/doc).

Import realsense-sys to your `Cargo.toml`.

```toml
[dependencies]
realsense-sys = "0.3"
```

and you can call low level C functions.

```rust
let pipeline = Pipeline::new()?;
let (pipeline_ptr, context_ptr) = pipeline.into_raw_parts();

unsafe {
    let mut error: *mut realsense_sys::rs2_error = std::ptr::null_mut();
    realsense_sys::rs2_pipeline_start(pipeline_ptr, &mut error as *mut _);
    if !error.is_null() {
        panic!("fail");
    }
}
```

### Generate documents from source code

The API changes may not be found on docs.rs. To generate document from the most recent commit,

```sh
cargo doc --open
```

## License

Apache 2.0. See [LICENSE](LICENSE) file.
