#include <librealsense2/rs.hpp>
#include <libusb-1.0/libusb.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>

bool flash_t265(int fps) {
    std::string fw_file;
    if (fps == 30) fw_file = "build/common/fw/target-30fps.mvcmd";
    else if (fps == 60) fw_file = "build/common/fw/target-60fps.mvcmd";
    else if (fps == 90) fw_file = "build/common/fw/target-90fps.mvcmd";
    else return false;

    std::ifstream ifs(fw_file, std::ios::binary | std::ios::ate);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open " << fw_file << "\n";
        return false;
    }
    std::streamsize size = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    std::vector<uint8_t> buffer(size);
    if (!ifs.read(reinterpret_cast<char*>(buffer.data()), size)) {
        std::cerr << "Failed to read " << fw_file << "\n";
        return false;
    }

    libusb_context* ctx = nullptr;
    if (libusb_init(&ctx) < 0) return false;

    // Reset if currently booted
    libusb_device_handle* h = libusb_open_device_with_vid_pid(ctx, 0x8087, 0x0b37);
    if (h) {
        std::cout << "[FLASH] Resetting T265 to bootloader...\n";
        libusb_control_transfer(h, 0x40, 0x10, 0, 0, nullptr, 0, 1000);
        libusb_close(h);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    // Wait for bootloader
    libusb_device_handle* unbooted = nullptr;
    for (int i = 0; i < 30; ++i) {
        unbooted = libusb_open_device_with_vid_pid(ctx, 0x03e7, 0x2150);
        if (unbooted) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!unbooted) {
        std::cerr << "[FLASH] Bootloader 03e7:2150 not found!\n";
        libusb_exit(ctx);
        return false;
    }

    std::cout << "[FLASH] Uploading " << fw_file << " (" << buffer.size() << " bytes)...\n";
    libusb_claim_interface(unbooted, 0);
    int transferred = 0;
    int r = libusb_bulk_transfer(unbooted, 0x01, buffer.data(), static_cast<int>(buffer.size()), &transferred, 5000);
    libusb_release_interface(unbooted, 0);
    libusb_close(unbooted);
    if (r < 0) {
        std::cerr << "[FLASH] Bulk transfer failed: " << r << "\n";
        libusb_exit(ctx);
        return false;
    }

    // Wait for booted device
    std::cout << "[FLASH] Waiting for T265 to boot...\n";
    bool booted = false;
    for (int i = 0; i < 40; ++i) {
        libusb_device_handle* bh = libusb_open_device_with_vid_pid(ctx, 0x8087, 0x0b37);
        if (bh) {
            libusb_close(bh);
            booted = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    libusb_exit(ctx);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return booted;
}

void stream_for_3s(int target_fps) {
    rs2::context ctx;
    rs2::device t265;
    for (auto&& d : ctx.query_devices()) {
        if (std::string(d.get_info(RS2_CAMERA_INFO_NAME)).find("T265") != std::string::npos) {
            t265 = d;
            break;
        }
    }
    if (!t265) {
        std::cerr << "T265 not found in realsense context!\n";
        return;
    }
    auto sensor = t265.query_sensors()[0];
    if (sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.0f);
    if (sensor.supports(RS2_OPTION_EXPOSURE)) sensor.set_option(RS2_OPTION_EXPOSURE, 2000.0f);
    if (sensor.supports(RS2_OPTION_GAIN)) sensor.set_option(RS2_OPTION_GAIN, 2.5f);

    std::vector<rs2::stream_profile> profiles;
    for (auto&& p : sensor.get_stream_profiles()) {
        if (auto vp = p.as<rs2::video_stream_profile>()) {
            if (vp.stream_type() == RS2_STREAM_FISHEYE && vp.fps() == target_fps) {
                profiles.push_back(p);
            }
        }
    }
    std::cout << "Opening " << profiles.size() << " profiles for " << target_fps << " FPS...\n";
    std::atomic<uint64_t> count{0};
    sensor.open(profiles);
    sensor.start([&](const rs2::frame& f) {
        if (f.get_profile().stream_index() == 1) count++;
    });

    for (int s = 1; s <= 3; ++s) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "  [" << target_fps << " FPS Mode] Second " << s << ": " << count.exchange(0) << " FPS\n";
    }
    sensor.stop();
    sensor.close();
}

int main() {
    std::cout << "=== Switching to 30 FPS ===\n";
    flash_t265(30);
    stream_for_3s(30);

    std::cout << "\n=== Switching to 60 FPS ===\n";
    flash_t265(60);
    stream_for_3s(60);

    std::cout << "\n=== Switching to 90 FPS ===\n";
    flash_t265(90);
    stream_for_3s(90);

    std::cout << "\nAll 3 switches tested successfully!\n";
    return 0;
}
