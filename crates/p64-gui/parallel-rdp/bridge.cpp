/*
 * This file serves as a bridge between the SDL windowing system and the Vulkan-based Parallel RDP (Reality Display Processor) emulator.
 * It initializes the Vulkan context, handles SDL events, and processes RDP commands.
 * 
 * The file handles:
 * - Definitions of constants and enums for DPC (Display Processor Command) and VI (Video Interface) registers.
 * - A class SDL_WSIPlatform that implements the Vulkan::WSIPlatform interface for creating and managing Vulkan surfaces with SDL.
 * - Functions for initializing and closing the Vulkan context, setting SDL window, calculating viewport dimensions, rendering frames, and processing RDP commands.
 * - The main loop for handling SDL events and updating the screen.
*/

#include <chrono>
#include <iostream>
#include <SDL2/SDL.h>
#include <SDL2/SDL_vulkan.h>

#include "wsi.hpp"
#include "rdp_device.hpp"

using namespace Vulkan;

#define DP_STATUS_XBUS_DMA 0x01
#define DP_STATUS_FREEZE 0x02
#define DP_STATUS_FLUSH 0x04
#define DP_STATUS_START_GCLK 0x008
#define DP_STATUS_TMEM_BUSY 0x010
#define DP_STATUS_PIPE_BUSY 0x020
#define DP_STATUS_CMD_BUSY 0x040
#define DP_STATUS_CBUF_READY 0x080
#define DP_STATUS_DMA_BUSY 0x100
#define DP_STATUS_END_VALID 0x200
#define DP_STATUS_START_VALID 0x400

enum dpc_registers
{
	DPC_START_REG,
	DPC_END_REG,
	DPC_CURRENT_REG,
	DPC_STATUS_REG,
	DPC_CLOCK_REG,
	DPC_BUFBUSY_REG,
	DPC_PIPEBUSY_REG,
	DPC_TMEM_REG,
	DPC_REGS_COUNT
};

enum vi_registers
{
	VI_STATUS_REG,
	VI_ORIGIN_REG,
	VI_WIDTH_REG,
	VI_V_INTR_REG,
	VI_CURRENT_REG,
	VI_BURST_REG,
	VI_V_SYNC_REG,
	VI_H_SYNC_REG,
	VI_LEAP_REG,
	VI_H_START_REG,
	VI_V_START_REG,
	VI_V_BURST_REG,
	VI_X_SCALE_REG,
	VI_Y_SCALE_REG,
	VI_REGS_COUNT
};

static bool fullscreen;
static void *rdram;
static SDL_Window *window;
static RDP::CommandProcessor *processor;
static WSI *wsi;
static uint32_t cmd_data[0x00040000 >> 2];
static int cmd_cur;
static int cmd_ptr;
static uint8_t emu_running;
static uint64_t rdp_sync_signal;
static uint32_t vi_registers[14];

static uint64_t last_frame_counter;
static uint64_t frame_counter;

RDP::CommandProcessorFlags flags = 0;

static const unsigned cmd_len_lut[64] = {
	1, 1, 1, 1, 1, 1, 1, 1,		  // 0-7
	4, 6, 12, 14, 12, 14, 20, 22, // 8-15
	1, 1, 1, 1, 1, 1, 1, 1,		  // 16-23
	1, 1, 1, 1, 1, 1, 1, 1,		  // 24-31
	1, 1, 1, 1, 2, 2, 1, 1,		  // 32-39
	1, 1, 1, 1, 1, 1, 1, 1,		  // 40-47
	1, 1, 1, 1, 1, 1, 1, 1,		  // 48-55
	1, 1, 1, 1, 1, 1, 1, 1		  // 56-63
};

static const uint32_t vertex_spirv[] =
	{0x07230203, 0x00010000, 0x000d000a, 0x00000034,
	 0x00000000, 0x00020011, 0x00000001, 0x0006000b,
	 0x00000001, 0x4c534c47, 0x6474732e, 0x3035342e,
	 0x00000000, 0x0003000e, 0x00000000, 0x00000001,
	 0x0008000f, 0x00000000, 0x00000004, 0x6e69616d,
	 0x00000000, 0x00000008, 0x00000016, 0x0000002b,
	 0x00040047, 0x00000008, 0x0000000b, 0x0000002a,
	 0x00050048, 0x00000014, 0x00000000, 0x0000000b,
	 0x00000000, 0x00050048, 0x00000014, 0x00000001,
	 0x0000000b, 0x00000001, 0x00050048, 0x00000014,
	 0x00000002, 0x0000000b, 0x00000003, 0x00050048,
	 0x00000014, 0x00000003, 0x0000000b, 0x00000004,
	 0x00030047, 0x00000014, 0x00000002, 0x00040047,
	 0x0000002b, 0x0000001e, 0x00000000, 0x00020013,
	 0x00000002, 0x00030021, 0x00000003, 0x00000002,
	 0x00040015, 0x00000006, 0x00000020, 0x00000001,
	 0x00040020, 0x00000007, 0x00000001, 0x00000006,
	 0x0004003b, 0x00000007, 0x00000008, 0x00000001,
	 0x0004002b, 0x00000006, 0x0000000a, 0x00000000,
	 0x00020014, 0x0000000b, 0x00030016, 0x0000000f,
	 0x00000020, 0x00040017, 0x00000010, 0x0000000f,
	 0x00000004, 0x00040015, 0x00000011, 0x00000020,
	 0x00000000, 0x0004002b, 0x00000011, 0x00000012,
	 0x00000001, 0x0004001c, 0x00000013, 0x0000000f,
	 0x00000012, 0x0006001e, 0x00000014, 0x00000010,
	 0x0000000f, 0x00000013, 0x00000013, 0x00040020,
	 0x00000015, 0x00000003, 0x00000014, 0x0004003b,
	 0x00000015, 0x00000016, 0x00000003, 0x0004002b,
	 0x0000000f, 0x00000017, 0xbf800000, 0x0004002b,
	 0x0000000f, 0x00000018, 0x00000000, 0x0004002b,
	 0x0000000f, 0x00000019, 0x3f800000, 0x0007002c,
	 0x00000010, 0x0000001a, 0x00000017, 0x00000017,
	 0x00000018, 0x00000019, 0x00040020, 0x0000001b,
	 0x00000003, 0x00000010, 0x0004002b, 0x00000006,
	 0x0000001f, 0x00000001, 0x0004002b, 0x0000000f,
	 0x00000023, 0x40400000, 0x0007002c, 0x00000010,
	 0x00000024, 0x00000017, 0x00000023, 0x00000018,
	 0x00000019, 0x0007002c, 0x00000010, 0x00000027,
	 0x00000023, 0x00000017, 0x00000018, 0x00000019,
	 0x00040017, 0x00000029, 0x0000000f, 0x00000002,
	 0x00040020, 0x0000002a, 0x00000003, 0x00000029,
	 0x0004003b, 0x0000002a, 0x0000002b, 0x00000003,
	 0x0004002b, 0x0000000f, 0x0000002f, 0x3f000000,
	 0x0005002c, 0x00000029, 0x00000033, 0x0000002f,
	 0x0000002f, 0x00050036, 0x00000002, 0x00000004,
	 0x00000000, 0x00000003, 0x000200f8, 0x00000005,
	 0x0004003d, 0x00000006, 0x00000009, 0x00000008,
	 0x000500aa, 0x0000000b, 0x0000000c, 0x00000009,
	 0x0000000a, 0x000300f7, 0x0000000e, 0x00000000,
	 0x000400fa, 0x0000000c, 0x0000000d, 0x0000001d,
	 0x000200f8, 0x0000000d, 0x00050041, 0x0000001b,
	 0x0000001c, 0x00000016, 0x0000000a, 0x0003003e,
	 0x0000001c, 0x0000001a, 0x000200f9, 0x0000000e,
	 0x000200f8, 0x0000001d, 0x000500aa, 0x0000000b,
	 0x00000020, 0x00000009, 0x0000001f, 0x000300f7,
	 0x00000022, 0x00000000, 0x000400fa, 0x00000020,
	 0x00000021, 0x00000026, 0x000200f8, 0x00000021,
	 0x00050041, 0x0000001b, 0x00000025, 0x00000016,
	 0x0000000a, 0x0003003e, 0x00000025, 0x00000024,
	 0x000200f9, 0x00000022, 0x000200f8, 0x00000026,
	 0x00050041, 0x0000001b, 0x00000028, 0x00000016,
	 0x0000000a, 0x0003003e, 0x00000028, 0x00000027,
	 0x000200f9, 0x00000022, 0x000200f8, 0x00000022,
	 0x000200f9, 0x0000000e, 0x000200f8, 0x0000000e,
	 0x00050041, 0x0000001b, 0x0000002c, 0x00000016,
	 0x0000000a, 0x0004003d, 0x00000010, 0x0000002d,
	 0x0000002c, 0x0007004f, 0x00000029, 0x0000002e,
	 0x0000002d, 0x0000002d, 0x00000000, 0x00000001,
	 0x0005008e, 0x00000029, 0x00000030, 0x0000002e,
	 0x0000002f, 0x00050081, 0x00000029, 0x00000032,
	 0x00000030, 0x00000033, 0x0003003e, 0x0000002b,
	 0x00000032, 0x000100fd, 0x00010038};

static const uint32_t fragment_spirv[] =
	{0x07230203, 0x00010000, 0x000d000a, 0x00000015,
	 0x00000000, 0x00020011, 0x00000001, 0x0006000b,
	 0x00000001, 0x4c534c47, 0x6474732e, 0x3035342e,
	 0x00000000, 0x0003000e, 0x00000000, 0x00000001,
	 0x0007000f, 0x00000004, 0x00000004, 0x6e69616d,
	 0x00000000, 0x00000009, 0x00000011, 0x00030010,
	 0x00000004, 0x00000007, 0x00040047, 0x00000009,
	 0x0000001e, 0x00000000, 0x00040047, 0x0000000d,
	 0x00000022, 0x00000000, 0x00040047, 0x0000000d,
	 0x00000021, 0x00000000, 0x00040047, 0x00000011,
	 0x0000001e, 0x00000000, 0x00020013, 0x00000002,
	 0x00030021, 0x00000003, 0x00000002, 0x00030016,
	 0x00000006, 0x00000020, 0x00040017, 0x00000007,
	 0x00000006, 0x00000004, 0x00040020, 0x00000008,
	 0x00000003, 0x00000007, 0x0004003b, 0x00000008,
	 0x00000009, 0x00000003, 0x00090019, 0x0000000a,
	 0x00000006, 0x00000001, 0x00000000, 0x00000000,
	 0x00000000, 0x00000001, 0x00000000, 0x0003001b,
	 0x0000000b, 0x0000000a, 0x00040020, 0x0000000c,
	 0x00000000, 0x0000000b, 0x0004003b, 0x0000000c,
	 0x0000000d, 0x00000000, 0x00040017, 0x0000000f,
	 0x00000006, 0x00000002, 0x00040020, 0x00000010,
	 0x00000001, 0x0000000f, 0x0004003b, 0x00000010,
	 0x00000011, 0x00000001, 0x0004002b, 0x00000006,
	 0x00000013, 0x00000000, 0x00050036, 0x00000002,
	 0x00000004, 0x00000000, 0x00000003, 0x000200f8,
	 0x00000005, 0x0004003d, 0x0000000b, 0x0000000e,
	 0x0000000d, 0x0004003d, 0x0000000f, 0x00000012,
	 0x00000011, 0x00070058, 0x00000007, 0x00000014,
	 0x0000000e, 0x00000012, 0x00000002, 0x00000013,
	 0x0003003e, 0x00000009, 0x00000014, 0x000100fd,
	 0x00010038};
enum
{
	MB_RDRAM_DRAM_ALIGNMENT_REQUIREMENT = 64 * 1024
};


class SDL_WSIPlatform : public Vulkan::WSIPlatform
{
public:
	VkSurfaceKHR create_surface(VkInstance instance, VkPhysicalDevice gpu) override;
	void destroy_surface(VkInstance instance, VkSurfaceKHR surface) override;
	std::vector<const char *> get_instance_extensions() override;
	uint32_t get_surface_width() override;
	uint32_t get_surface_height() override;
	bool alive(Vulkan::WSI &wsi) override;
	void poll_input() override;
	void poll_input_async(Granite::InputTrackerHandler *handler) override;
	void set_window(SDL_Window *_window);
	void do_resize();

private:
	SDL_Window *window;
};

VkSurfaceKHR SDL_WSIPlatform::create_surface(VkInstance instance, VkPhysicalDevice gpu)
{
	VkSurfaceKHR surface = nullptr;
	SDL_bool result = SDL_Vulkan_CreateSurface(window, instance, &surface);
	if (result != SDL_TRUE)
	{
		printf("Error creating Vulkan surface\n");
	}
	return surface;
}

void SDL_WSIPlatform::destroy_surface(VkInstance instance, VkSurfaceKHR surface) {}

std::vector<const char *> SDL_WSIPlatform::get_instance_extensions()
{

	unsigned int extensionCount = 0;
	SDL_Vulkan_GetInstanceExtensions(window, &extensionCount, nullptr);
	std::vector<const char *> extensionNames(extensionCount);
	SDL_bool result = SDL_Vulkan_GetInstanceExtensions(window, &extensionCount, extensionNames.data());
	if (result != SDL_TRUE)
	{
		printf("Error creating SDL Vulkan surface\n");
	}
	return extensionNames;
}

uint32_t SDL_WSIPlatform::get_surface_width()
{
	int w, h;
	SDL_GetWindowSize(window, &w, &h);
	return w;
}
uint32_t SDL_WSIPlatform::get_surface_height()
{
	int w, h;
	SDL_GetWindowSize(window, &w, &h);
	return h;
}
bool SDL_WSIPlatform::alive(Vulkan::WSI &wsi) { return true; }
void SDL_WSIPlatform::poll_input() { SDL_PumpEvents(); }
void SDL_WSIPlatform::poll_input_async(Granite::InputTrackerHandler *handler) { SDL_PumpEvents(); }
void SDL_WSIPlatform::set_window(SDL_Window *_window) { window = _window; }
void SDL_WSIPlatform::do_resize() { resize = true; }

extern "C" {
	void set_upscaling_1x() { flags = 0; }
    void set_upscaling_2x() { flags = RDP::COMMAND_PROCESSOR_FLAG_UPSCALING_2X_BIT; }
    void set_upscaling_4x() { flags = RDP::COMMAND_PROCESSOR_FLAG_UPSCALING_4X_BIT; }
    void set_upscaling_8x() { flags = RDP::COMMAND_PROCESSOR_FLAG_UPSCALING_8X_BIT; }
}

extern "C" void vk_close() {
	delete processor;
	delete wsi; 
}

extern "C" void vk_init(void *mem_base, uint32_t rdram_size, uint8_t _fullscreen)
{
	auto start = std::chrono::high_resolution_clock::now();

	fullscreen = _fullscreen != 0;
	rdram = mem_base;
	// bool window_vsync = 0;
	bool window_vsync = 1;
	wsi = new WSI;
	SDL_WSIPlatform *wsi_platform = new SDL_WSIPlatform;
	wsi_platform->set_window(window);
	wsi->set_platform(wsi_platform);
	wsi->set_present_mode(window_vsync ? PresentMode::SyncToVBlank : PresentMode::UnlockedMaybeTear);
	wsi->set_backbuffer_srgb(false);

	auto end = std::chrono::high_resolution_clock::now();
	std::cout << "Initialization time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " microseconds" << std::endl;

	start = std::chrono::high_resolution_clock::now();
	Context::SystemHandles handles = {};
	if (!::Vulkan::Context::init_loader(nullptr))
	{
		vk_close();
	}
	end = std::chrono::high_resolution_clock::now();
	std::cout << "Vulkan Context Loader time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " microseconds" << std::endl;

	start = std::chrono::high_resolution_clock::now();
	if (!wsi->init_simple(1, handles))
	{
		vk_close();
	}
	end = std::chrono::high_resolution_clock::now();
	std::cout << "WSI Init Simple time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " microseconds" << std::endl;

	start = std::chrono::high_resolution_clock::now();
	processor = new RDP::CommandProcessor(wsi->get_device(), rdram, 0, rdram_size, rdram_size / 2, flags);
	end = std::chrono::high_resolution_clock::now();
	std::cout << "Command Processor creation time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " microseconds" << std::endl;

	start = std::chrono::high_resolution_clock::now();
	if (!processor->device_is_supported())
	{
		delete processor;
		delete wsi;
		processor = nullptr;
		vk_close();
	}
	end = std::chrono::high_resolution_clock::now();
	std::cout << "Device support check time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " microseconds" << std::endl;

	start = std::chrono::high_resolution_clock::now();
	wsi->begin_frame();
	end = std::chrono::high_resolution_clock::now();
	std::cout << "Begin frame time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " microseconds" << std::endl;

	emu_running = 1;
	last_frame_counter = 0;
	frame_counter = 0;
}



extern "C" int sdl_event_filter(void *userdata, SDL_Event *event)
{
	if (event->type == SDL_WINDOWEVENT)
	{
		SDL_WSIPlatform *platform = (SDL_WSIPlatform *)&wsi->get_platform();
		switch (event->window.event)
		{
		case SDL_WINDOWEVENT_CLOSE:
			emu_running = 0;
			break;
		case SDL_WINDOWEVENT_RESIZED:
			platform->do_resize();
			break;
		default:
			break;
		}
	}
	else if (fullscreen && event->type == SDL_KEYDOWN)
	{
		switch (event->key.keysym.scancode)
		{
		case SDL_SCANCODE_ESCAPE:
			emu_running = 0;
			break;
		default:
			break;
		}
	}

	return 0;
}

extern "C" void set_sdl_window(void *_window)
{
	window = (SDL_Window *)_window;
	SDL_SetEventFilter(sdl_event_filter, nullptr);
}

extern "C" void calculate_viewport(float *x, float *y, float *width, float *height, int32_t display_width, int32_t display_height)
{
    int w, h;
    SDL_GetWindowSize(window, &w, &h);

    *width = w;
    *height = h;
    *x = 0;
    *y = 0;
    int32_t hw = display_height * *width;
    int32_t wh = display_width * *height;

    if (hw > wh)
    {
        int32_t w_max = wh / display_height;
        *x += (*width - w_max) / 2;
        *width = w_max;
    }
    else if (hw < wh)
    {
        int32_t h_max = hw / display_width;
        *y += (*height - h_max) / 2;
        *height = h_max;
    }
}

struct ViewportData {
    float x;
    float y; 
    float width;
    float height;
};

extern "C" ViewportData calculate_viewport_values(int32_t width = 1280, int32_t height = 720) {
    ViewportData vp = {};
    calculate_viewport(&vp.x, &vp.y, &vp.width, &vp.height, width, height);
    return vp;
}

extern "C" void render_frame(Vulkan::Device &device)
{
	RDP::ScanoutOptions options = {};
	Vulkan::ImageHandle image = processor->scanout(options);

	// Normally reflection is automated.
	Vulkan::ResourceLayout vertex_layout = {};
	Vulkan::ResourceLayout fragment_layout = {};
	fragment_layout.output_mask = 1 << 0;
	fragment_layout.sets[0].sampled_image_mask = 1 << 0;

	// This request is cached.
	auto *program = device.request_program(vertex_spirv, sizeof(vertex_spirv),
										   fragment_spirv, sizeof(fragment_spirv),
										   &vertex_layout,
										   &fragment_layout);

	auto cmd = device.request_command_buffer(); {
		auto rp = device.get_swapchain_render_pass(Vulkan::SwapchainRenderPass::ColorOnly);
		cmd->begin_render_pass(rp);

		VkViewport vp = cmd->get_viewport();
		ViewportData viewport_data = calculate_viewport_values();
		
		// Apply calculated values to VkViewport
		vp.x = viewport_data.x;
		vp.y = viewport_data.y;
		vp.width = viewport_data.width;
		vp.height = viewport_data.height;
		
		cmd->set_viewport(vp);
		cmd->set_program(program);

		cmd->set_opaque_state();
		cmd->set_depth_test(false, false);
		cmd->set_cull_mode(VK_CULL_MODE_NONE);

		if (image) {
			cmd->set_texture(0, 0, image->get_view(), Vulkan::StockSampler::LinearClamp);
			cmd->set_viewport(vp);
			cmd->draw(3);
		}

		cmd->end_render_pass();
	}
	device.submit(cmd);
}

extern "C" void rdp_set_vi_register(uint32_t reg, uint32_t value)
{
	processor->set_vi_register(RDP::VIRegister(reg), value);
	vi_registers[reg] = value;
}

extern "C" uint8_t rdp_update_screen()
{
	auto &device = wsi->get_device();
	render_frame(device);
	wsi->end_frame();
	wsi->begin_frame();
	frame_counter++;
	return emu_running;
}

extern "C" uint32_t viCalculateHorizonalWidth(uint32_t hstart, uint32_t xscale, uint32_t width)
{
	if (xscale == 0)
		return 320;

	uint32_t start = ((hstart & 0x03FF0000) >> 16) & 0x3FF;
	uint32_t end = (hstart & 0x3FF);
	uint32_t delta;
	if (end > start)
		delta = end - start;
	else
		delta = start - end;
	uint32_t scale = (xscale & 0xFFF);

	if (delta == 0)
	{
		delta = width;
	}

	return (delta * scale) / 0x400;
}

extern "C" uint32_t viCalculateVerticalHeight(uint32_t vstart, uint32_t yscale)
{
	if (yscale == 0)
		return 240;

	uint32_t start = ((vstart & 0x03FF0000) >> 16) & 0x3FF;
	uint32_t end = (vstart & 0x3FF);
	uint32_t delta;
	if (end > start)
		delta = end - start;
	else
		delta = start - end;
	uint32_t scale = (yscale & 0xFFF);

	return (delta * scale) / 0x800;
}

extern "C" uint64_t rdp_process_commands(uint32_t *dpc_regs, uint8_t *SP_DMEM)
{
	uint64_t interrupt_timer = 0;
	const uint32_t DP_CURRENT = dpc_regs[DPC_CURRENT_REG] & 0x00FFFFF8;
	const uint32_t DP_END = dpc_regs[DPC_END_REG] & 0x00FFFFF8;

	int length = DP_END - DP_CURRENT;
	if (length <= 0)
		return interrupt_timer;

	length = unsigned(length) >> 3;
	if ((cmd_ptr + length) & ~(0x0003FFFF >> 3))
		return interrupt_timer;

	dpc_regs[DPC_STATUS_REG] |= DP_STATUS_PIPE_BUSY | DP_STATUS_START_GCLK;

	uint32_t offset = DP_CURRENT;
	if (dpc_regs[DPC_STATUS_REG] & DP_STATUS_XBUS_DMA)
	{
		do
		{
			offset &= 0xFF8;
			cmd_data[2 * cmd_ptr + 0] = SDL_SwapBE32(*reinterpret_cast<const uint32_t *>(SP_DMEM + offset));
			cmd_data[2 * cmd_ptr + 1] = SDL_SwapBE32(*reinterpret_cast<const uint32_t *>(SP_DMEM + offset + 4));
			offset += sizeof(uint64_t);
			cmd_ptr++;
		} while (--length > 0);
	}
	else
	{
		if (DP_END > 0x7ffffff || DP_CURRENT > 0x7ffffff)
		{
			return interrupt_timer;
		}
		else
		{
			do
			{
				offset &= 0xFFFFF8;
				cmd_data[2 * cmd_ptr + 0] = *reinterpret_cast<const uint32_t *>((uint8_t *)rdram + offset);
				cmd_data[2 * cmd_ptr + 1] = *reinterpret_cast<const uint32_t *>((uint8_t *)rdram + offset + 4);
				offset += sizeof(uint64_t);
				cmd_ptr++;
			} while (--length > 0);
		}
	}

	while (cmd_cur - cmd_ptr < 0)
	{
		uint32_t w1 = cmd_data[2 * cmd_cur];
		uint32_t command = (w1 >> 24) & 63;
		int cmd_length = cmd_len_lut[command];

		if (cmd_ptr - cmd_cur - cmd_length < 0)
		{
			dpc_regs[DPC_START_REG] = dpc_regs[DPC_CURRENT_REG] = dpc_regs[DPC_END_REG];
			return interrupt_timer;
		}

		if (command >= 8)
			processor->enqueue_command(cmd_length * 2, &cmd_data[2 * cmd_cur]);

		if (RDP::Op(command) == RDP::Op::SyncFull)
		{
			// Sync once per frame
			if (frame_counter != last_frame_counter) 
			{
				rdp_sync_signal = processor->signal_timeline();
				last_frame_counter = frame_counter;
			}
			else
			{
				rdp_sync_signal = 0;
			}

			uint32_t width = viCalculateHorizonalWidth(vi_registers[VI_H_START_REG], vi_registers[VI_X_SCALE_REG], vi_registers[VI_WIDTH_REG]);
			if (width == 0)
			{
				width = 320;
			}
			uint32_t height = viCalculateVerticalHeight(vi_registers[VI_V_START_REG], vi_registers[VI_Y_SCALE_REG]);
			if (height == 0)
			{
				height = 240;
			}
			interrupt_timer = width * height * 4;

			dpc_regs[DPC_STATUS_REG] &= ~(DP_STATUS_PIPE_BUSY | DP_STATUS_START_GCLK);
		}

		cmd_cur += cmd_length;
	}

	cmd_ptr = 0;
	cmd_cur = 0;
	dpc_regs[DPC_CURRENT_REG] = dpc_regs[DPC_END_REG];
	dpc_regs[DPC_STATUS_REG] |= DP_STATUS_CBUF_READY;

	return interrupt_timer;
}

extern "C" void full_sync() {
	if (rdp_sync_signal) { processor->wait_for_timeline(rdp_sync_signal); }
}
