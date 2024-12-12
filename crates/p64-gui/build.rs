use std::env;
use std::process::Command;

use winres::WindowsResource;

const PARALLEL_RDP_PATH: &str = "parallel-rdp";
const STANDALONE_PATH: &str = "parallel-rdp/parallel-rdp-standalone";

fn add_source_files(build: &mut cc::Build) {
    // RDP core files
    let rdp_files = [
        "command_ring.cpp", "rdp_device.cpp", "rdp_dump_write.cpp",
        "rdp_renderer.cpp", "video_interface.cpp"
    ];
    for file in rdp_files {
        build.file(format!("{}/parallel-rdp/{}", STANDALONE_PATH, file));
    }

    // Vulkan implementation files
    let vulkan_files = [
        "buffer.cpp", "buffer_pool.cpp", "command_buffer.cpp", "command_pool.cpp",
        "context.cpp", "cookie.cpp", "descriptor_set.cpp", "device.cpp",
        "event_manager.cpp", "fence.cpp", "fence_manager.cpp", "image.cpp",
        "indirect_layout.cpp", "memory_allocator.cpp", "pipeline_event.cpp",
        "query_pool.cpp", "render_pass.cpp", "sampler.cpp", "semaphore.cpp",
        "semaphore_manager.cpp", "shader.cpp", "wsi.cpp"
    ];
    for file in vulkan_files {
        build.file(format!("{}/vulkan/{}", STANDALONE_PATH, file));
    }

    // Utility files
    let util_files = [
        "arena_allocator.cpp", "logging.cpp", "thread_id.cpp", "aligned_alloc.cpp",
        "timer.cpp", "timeline_trace_file.cpp", "environment.cpp", "thread_name.cpp"
    ];
    for file in util_files {
        build.file(format!("{}/util/{}", STANDALONE_PATH, file));
    }

    // Additional files
    build
        .file(format!("{}/vulkan/texture/texture_format.cpp", STANDALONE_PATH))
        .file(format!("{}/volk/volk.c", STANDALONE_PATH))
        .file(format!("{}/bridge.cpp", PARALLEL_RDP_PATH));
}

fn set_includes(build: &mut cc::Build) {
    let include_paths = [
        "parallel-rdp", "volk", "vulkan",
        "vulkan-headers/include", "util"
    ];
    for path in include_paths {
        build.include(format!("{}/{}", STANDALONE_PATH, path));
    }
}

#[cfg(all(target_os = "linux", not(target_arch = "wasm32")))]
fn configure_linux_sdl2(build: &mut cc::Build) {
    if let Ok(output) = Command::new("pkg-config").args(["--cflags", "sdl2"]).output() {
        let include_path = String::from_utf8_lossy(&output.stdout);
        for path in include_path.split_whitespace() {
            if path.starts_with("-I") {
                build.flag(path);
            }
        }
    }
    if let Ok(include_path) = env::var("DEP_SDL2_INCLUDE") {
        build.include(include_path);
    }
}

fn configure_platform_specific(build: &mut cc::Build) {
    #[cfg(target_arch = "wasm32")]
    {
        build
            .flag("-Wno-missing-field-initializers")
            .flag("-Wno-unused-parameter");
    }

    #[cfg(target_os = "windows")]
    {
        if cfg!(target_arch = "x86_64") {
            build.flag("/arch:AVX2");
        }
        build.flag("-DVK_USE_PLATFORM_WIN32_KHR");
    }

    #[cfg(any(target_os = "linux", target_os = "macos"))]
    {
        if cfg!(target_arch = "x86_64") {
            build.flag("-march=x86-64-v3");
        }
        build
            .flag("-Wno-missing-field-initializers")
            .flag("-Wno-unused-parameter");
    }
}

fn main() {
    let mut build = cc::Build::new();
    
    build.cpp(true).std("c++17");

    let mut res = WindowsResource::new();
    if cfg!(target_os = "windows") {
        res.set_icon("../p64-gui/N64.ico");
        res.set_icon("N64.ico");
        res.compile().unwrap();
    }
    
    add_source_files(&mut build);
    set_includes(&mut build);
    
    #[cfg(all(target_os = "linux", not(target_arch = "wasm32")))]
    configure_linux_sdl2(&mut build);
    
    configure_platform_specific(&mut build);
    
    build.compile("parallel-rdp");
}