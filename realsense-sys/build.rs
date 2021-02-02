use std::path::PathBuf;

fn main() {
    if cfg!(feature = "doc-only") {
        return;
    }

    let cargo_manifest_dir: PathBuf = PathBuf::from(std::env::var_os("CARGO_MANIFEST_DIR").unwrap());

    // Probe libary
    let library = pkg_config::probe_library("realsense2")
        .expect("pkg-config failed to find realsense2 package");
    let major_version = library.version.find('.')
        .map(|i| &library.version[..i])
        .expect("failed to determine librealsense major version");

    if major_version != "2" {
        panic!("librealsense2 version {} is not supported, expected major version 2", library.version)
    }

    // generate bindings
    #[cfg(feature = "buildtime-bindgen")]
    {
        use std::path::Path;

        let include_dir = library
            .include_paths
            .iter()
            .filter_map(|path| {
                let dir = Path::new(path).join("librealsense2");
                if dir.is_dir() {
                    Some(dir)
                } else {
                    None
                }
            })
            .next()
            .expect("fail find librealsense2 include directory");

        let bindings = bindgen::Builder::default()
            .clang_arg("-fno-inline-functions")
            .header(include_dir.join("rs.h").to_str().unwrap())
            .header(
                include_dir
                    .join("h")
                    .join("rs_pipeline.h")
                    .to_str()
                    .unwrap(),
            )
            .header(
                include_dir
                    .join("h")
                    .join("rs_advanced_mode_command.h")
                    .to_str()
                    .unwrap(),
            )
            .header(include_dir.join("h").join("rs_config.h").to_str().unwrap())
            .header(
                cargo_manifest_dir
                    .join("c")
                    .join("rsutil_delegate.h")
                    .to_str()
                    .unwrap(),
            )
            .whitelist_var("RS2_.*")
            .whitelist_type("rs2_.*")
            .whitelist_function("rs2_.*")
            .whitelist_function("_rs2_.*")
            .generate()
            .expect("Unable to generate bindings");

        // Write the bindings to file
        let bindings_dir = cargo_manifest_dir.join("bindings");
        let bindings_file = bindings_dir.join("bindings.rs");

        if let Err(e) = std::fs::create_dir_all(&bindings_dir) {
            panic!("failed to create directory {}: {}", bindings_dir.display(), e);
        }
        bindings
            .write_to_file(bindings_file)
            .expect("Couldn't write bindings!");
    }

    // compile and link rsutil_delegate.h statically
    cc::Build::new()
        .includes(&library.include_paths)
        .file(cargo_manifest_dir.join("c/rsutil_delegate.c"))
        .compile("rsutil_delegate");

    // link the libraries specified by pkg-config.
    for dir in &library.link_paths {
        println!("cargo:rustc-link-search=native={}", dir.to_str().unwrap());
    }
    for lib in &library.libs {
        println!("cargo:rustc-link-lib={}", lib);
    }
}
