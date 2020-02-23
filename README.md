# RealSense bindings for Rust

The project provides high-level bindings to librealsense2 library.

It includes two crates. The `realsense-rust` serves as high-level library, and `realsense-sys` serves FFI interface to C API.

The project is still under development and production use should be avoided.

## Usage

The crate is not published to crates.io yet. Add the following to your `Cargo.toml` if you really want this fancy new, bleeding edge and unstable library.

```toml
[dependencies]
realsense-rust = { git = "https://github.com/jerry73204/realsense-rust.git" }
```

## How to make contributions

The project uses bindgen to generate bindings from pure C API, no C++ parts included. All low-level C types and function are gathered in `realsense-sys` crate.

We suggest to follow these steps check out C functions. Their names should be self-explanatory. Also, we suggest C/C++ examples on official [librealsense](https://github.com/IntelRealSense/librealsense) repository.

```sh
cd $repo
cargo doc

firefox target/doc/realsense_sys/index.html  # open doc in browser
```

The high level API in `realsense-rust` crate is basically wrappers to low-level FFI interface. It written in Rust from scratch rather than by automated means. It's encouraged to use marker traits, trait ops and else to build rusty style interface.

## License

Apache 2.0. See [LICENSE](LICENSE) file.
