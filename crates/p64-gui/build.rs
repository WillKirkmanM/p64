use std::env;
use std::process::Command;
use winres::WindowsResource;

const PARALLEL_RDP_PATH: &str = "parallel-rdp";
const STANDALONE_PATH: &str = "parallel-rdp/parallel-rdp-standalone";

fn add_source_files(build: &mut cc::Build) {
    let rdp_files = [
        "command_ring.cpp", "rdp_device.cpp", "rdp_dump_write.cpp",
        "rdp_renderer.cpp", "video_interface.cpp"
    ];
    for file in rdp_files {
        build.file(format!("{}/parallel-rdp/{}", STANDALONE_PATH, file));
    }

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

    let util_files = [
        "arena_allocator.cpp", "logging.cpp", "thread_id.cpp", "aligned_alloc.cpp",
        "timer.cpp", "timeline_trace_file.cpp", "environment.cpp", "thread_name.cpp"
    ];
    for file in util_files {
        build.file(format!("{}/util/{}", STANDALONE_PATH, file));
    }

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

fn configure_sdl2(build: &mut cc::Build) {
    if let Ok(sdl2_dir) = env::var("SDL2_DIR") {
        println!("cargo:warning=SDL2_DIR: {}", sdl2_dir);
        
        let mingw_include = format!("{}/x86_64-w64-mingw32/include/SDL2", sdl2_dir);
        let mingw_lib = format!("{}/x86_64-w64-mingw32/lib", sdl2_dir);
        let msvc_include = format!("{}/include", sdl2_dir);
        let msvc_lib = format!("{}/lib/x64", sdl2_dir);
        
        if std::path::Path::new(&mingw_include).exists() {
            println!("cargo:warning=Using MinGW SDL2 include: {}", mingw_include);
            build.include(&mingw_include);
            println!("cargo:rustc-link-search=native={}", mingw_lib);
        } else if std::path::Path::new(&msvc_include).exists() {
            println!("cargo:warning=Using MSVC SDL2 include: {}", msvc_include);
            build.include(&msvc_include);
            println!("cargo:rustc-link-search=native={}", msvc_lib);
        } else {
            panic!("Could not find SDL2 include directory");
        }
        
        println!("cargo:rustc-link-lib=SDL2");
        println!("cargo:rustc-link-lib=SDL2main");
    } else {
        panic!("SDL2_DIR environment variable not set");
    }
}

fn configure_msvc(build: &mut cc::Build) {
    build
        .flag("/EHsc")           // Enable C++ exception handling
        .flag("/bigobj")         // Increase object file section limit
        .flag("/permissive-")    // Enforce strict standard compliance
        .flag("/Zc:preprocessor") // Enable standards-conforming preprocessor
        .flag("/wd4244")         // Disable conversion warnings
        .flag("/wd4267")         // Disable size_t conversion warnings
        .flag("/wd4389")         // Disable signed/unsigned mismatch warnings
        .flag("/W4")             // Warning level 4
        .flag("/O2")             // Optimization level
        .define("WIN32", None)
        .define("_WINDOWS", None)
        .define("NOMINMAX", None)
        .define("_CRT_SECURE_NO_WARNINGS", None);
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
        build.define("VK_USE_PLATFORM_WIN32_KHR", None);
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
    println!("cargo:rerun-if-changed=parallel-rdp/bridge.cpp");
    println!("cargo:rerun-if-changed=build.rs");

    let mut build = cc::Build::new();
    
    build.cpp(true)
         .std("c++17")
         .warnings(true)
         .debug(true);

    #[cfg(target_os = "windows")] 
    {
        let mut res = WindowsResource::new();
        res.set_icon("N64.ico");
        res.compile().unwrap();
        
        configure_sdl2(&mut build);
        configure_msvc(&mut build);
        
        if let Ok(vulkan_sdk) = env::var("VULKAN_SDK") {
            println!("cargo:warning=Found Vulkan SDK: {}", vulkan_sdk);
            build.include(format!("{}/Include", vulkan_sdk));
        }
    }
    
    add_source_files(&mut build);
    set_includes(&mut build);
    configure_platform_specific(&mut build);
    
    build.compile("parallel-rdp");
}