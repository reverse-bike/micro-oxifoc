fn main() {
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").expect("manifest directory");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rustc-link-search={manifest_dir}");
}
