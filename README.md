# RealSense bindings for Rust

The project provides high-level bindings to librealsense2 library as well as low-level FFI interface.
It supports asynchronous API and integration with [image](https://github.com/image-rs/image) and [nalgebra](https://github.com/rustsim/nalgebra) types.

## Usage

Add the dependency in `Cargo.toml` to import this crate in your project.

```toml
[dependencies]
realsense-rust = "^0.3.0"
```

If you would like to generate Rust bindings during build time, you can enable the `buildtime-bindgen` feature.

```toml
[dependencies]
realsense-rust = { version = "^0.3.0", features = ["buildtime-bindgen"] }
```

## Examples

Please visit the [examples](examples) directory to see the example usages.

## How to make contributions

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
