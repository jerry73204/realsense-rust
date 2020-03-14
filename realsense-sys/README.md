# realsense-sys

Use RealSense C library as Rust crate

## Usage

This crate finds and links RealSense SDK version 2 at minimum. It detects `realsense2.pc` using pkg-config and finds the headers and shared libraries accordingly.

To use this crate, add this line in your `Cargo.toml`.

```toml
realsense-sys = "^0.2.1"
```
