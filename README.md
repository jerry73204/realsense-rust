# RealSense bindings for Rust

The project provides high-level bindings to librealsense2 library as well as low-level FFI interface.
It supports asynchronous API and integration with [image](https://github.com/image-rs/image) and [nalgebra](https://github.com/rustsim/nalgebra) types.

## Usage

Visit RealSense official repository ([link](https://github.com/IntelRealSense/librealsense)) and install **librealsense 2.36.0** on your system.

To add this crate to your project,

```toml
[dependencies]
realsense-rust = "0.4"
```


If you have troubles compiling this project, perhaps your system is using older librealsense. Try to update the library. Otherwise you can enable `buildtime-bindgen` cargo feature to generate Rust bindings during build time.

```toml
[dependencies]
realsense-rust = { version = "0.4", features = ["buildtime-bindgen"] }
```

## Cargo Features

- **with-nalgebra** (default): Enable [nalgebra](https://github.com/rustsim/nalgebra) support.
- **with-image** (default): Enable [image](https://github.com/image-rs/image) support.
- **buildtime-bindgen**: Generate Rust bindings during build time.
- **device-test**: Enable tests that requires connections to RealSense devices.

## Examples

Please visit the [examples](examples) directory to see the example usages.

## Contribute to the project

### realsense-sys: Low-level API

The project uses bindgen to generate bindings from pure C API.

We suggest the official librealsense2 [documentation](https://github.com/IntelRealSense/librealsense/tree/master/doc) to check out the actual usage of low-level API.

Also, you can choose to browse the generated document for `realsense-sys`.

```sh
cd $repo/realsense-sys
cargo doc --open
```

### realsense-rust: High-level API

The `realsense-rust` crate is basically a wrapper upon the `realsense-sys` crate. We suggest to take C++ API as reference and rewrite them in rusty style.

## License

Apache 2.0. See [LICENSE](LICENSE) file.
