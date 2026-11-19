#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <libusb-1.0/libusb.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <filesystem>
#include <atomic>
#include <chrono>
#include <thread>
#include <mutex>
#include <vector>
#include <csignal>
#include <cmath>

// Window configuration
const std::string STREAM_WINDOW = "RealSense Dual Stream Visualizer (D435i + T265)";
const std::string CONTROLLER_WINDOW = "RealSense Stream Controller";

// Fixed Canvas Dimensions
const int CANVAS_WIDTH = 2560;       // 1280 * 2 (Left & Right camera viewports)
const int CANVAS_HEIGHT = 1520;      // 720 (D435i top) + 800 (T265 bottom)
const int D435_SLOT_WIDTH = 1280;
const int D435_SLOT_HEIGHT = 720;
const int T265_SLOT_WIDTH = 1280;
const int T265_SLOT_HEIGHT = 800;

// Running flag for clean signal shutdown
std::atomic<bool> g_running{true};
void signal_handler(int) {
    g_running = false;
}

// Global librealsense context pointer
std::unique_ptr<rs2::context> g_rs_ctx;

// -------------------------------------------------------------
// D435i State & Profile Table
// -------------------------------------------------------------
std::unique_ptr<rs2::pipeline> g_d435_pipe;
std::string g_d435_serial = "";
std::mutex g_d435_mutex;

std::atomic<int> g_d435_width{848};
std::atomic<int> g_d435_height{100};
std::atomic<int> g_target_d435_fps{300};
std::atomic<bool> g_switching_d435{false};

cv::Mat g_d435_left_mat(100, 848, CV_8UC1, cv::Scalar(0));   // Left IR
cv::Mat g_d435_right_mat(100, 848, CV_8UC1, cv::Scalar(0));  // Right IR

std::atomic<uint64_t> g_d435_left_cnt{0};
std::atomic<uint64_t> g_d435_right_cnt{0};
std::atomic<uint64_t> g_last_ir1_fn{0};
std::atomic<uint64_t> g_last_ir2_fn{0};
std::atomic<bool> g_d435_enabled{true};
std::atomic<bool> g_emitter_enabled{false};
rs2::sensor g_d435_stereo_sensor;
bool g_has_d435 = false;

// D435i Specification Table
struct D435ProfileRow {
    std::string label;
    int width;
    int height;
    std::vector<int> fps_slots; // Slots for [6, 15, 30, 60, 90, 300] (-1 if unavailable)
};

const std::vector<int> D435_FPS_HEADERS = {6, 15, 30, 60, 90, 300};

const std::vector<D435ProfileRow> D435_TABLE = {
    {"848 x 100",  848,  100, {-1, -1, -1, -1, -1, 300}},
    {"1280 x 720", 1280, 720, { 6, 15, 30, -1, -1,  -1}},
    {"848 x 480",  848,  480, { 6, 15, 30, 60, 90,  -1}},
    {"640 x 480",  640,  480, { 6, 15, 30, 60, 90,  -1}},
    {"640 x 360",  640,  360, { 6, 15, 30, 60, 90,  -1}},
    {"480 x 270",  480,  270, { 6, 15, 30, 60, 90,  -1}},
    {"424 x 240",  424,  240, { 6, 15, 30, 60, 90,  -1}}
};

// -------------------------------------------------------------
// T265 State & Profile Table
// -------------------------------------------------------------
std::mutex g_t265_mutex;
cv::Mat g_t265_left_mat(800, 848, CV_8UC1, cv::Scalar(0));   // Left Fisheye
cv::Mat g_t265_right_mat(800, 848, CV_8UC1, cv::Scalar(0));  // Right Fisheye

std::atomic<uint64_t> g_t265_left_cnt{0};
std::atomic<uint64_t> g_t265_right_cnt{0};
std::atomic<uint64_t> g_last_t265_f1_fn{0};
std::atomic<uint64_t> g_last_t265_f2_fn{0};
std::atomic<bool> g_t265_enabled{true};
rs2::sensor g_t265_sensor;
bool g_has_t265 = false;
std::atomic<int> g_target_t265_fps{90};
std::atomic<bool> g_switching_t265{false};

// Float exposure and gain values
std::atomic<float> g_exposure_f{2000.0f}; // 200.0 to 16000.0 us (clamped to 10000.0 in 90 FPS)
std::atomic<float> g_gain_f{2.5f};        // 1.0 to 10.0 x

// T265 Specification Table
struct T265ProfileRow {
    std::string label;
    int width;
    int height;
    std::vector<int> fps_values;
};

const std::vector<T265ProfileRow> T265_TABLE = {
    {"848 x 800", 848, 800, {30, 60, 90}}
};

// -------------------------------------------------------------
// 2-Second Benchmark Recording State
// -------------------------------------------------------------
enum RecordingState { REC_IDLE, REC_RECORDING, REC_SAVING };
std::atomic<RecordingState> g_rec_state{REC_IDLE};
std::chrono::steady_clock::time_point g_rec_start_time;

std::mutex g_rec_mutex;
std::vector<cv::Mat> g_rec_d435_left;
std::vector<cv::Mat> g_rec_d435_right;
std::vector<cv::Mat> g_rec_t265_left;
std::vector<cv::Mat> g_rec_t265_right;
std::atomic<uint64_t> g_last_rec_d435_fn{0};
std::atomic<uint64_t> g_last_rec_t265_l_fn{0};
std::atomic<uint64_t> g_last_rec_t265_r_fn{0};

std::mutex g_banner_mutex;
std::string g_banner_text = "";
std::chrono::steady_clock::time_point g_banner_time;

void set_banner(const std::string& text) {
    std::lock_guard<std::mutex> lock(g_banner_mutex);
    g_banner_text = text;
    g_banner_time = std::chrono::steady_clock::now();
}

// -------------------------------------------------------------
// Controller Window Interactive Coordinates (520 x 700)
// -------------------------------------------------------------
const cv::Rect RECT_CTRL_RECORD(20, 50, 480, 44);

// D435i Row
const cv::Rect RECT_CTRL_D435_STREAM(20, 146, 170, 30);
const cv::Rect RECT_CTRL_D435_EMITTER(200, 146, 170, 30);

// T265 Row
const cv::Rect RECT_CTRL_T265_STREAM(20, 442, 170, 30);

// Sliders inside Controller Window
const cv::Rect RECT_CTRL_EXP_TRACK(100, 558, 380, 16);
const cv::Rect RECT_CTRL_GAIN_TRACK(100, 602, 380, 16);

bool g_dragging_exp = false;
bool g_dragging_gain = false;

// -------------------------------------------------------------
// Hardware Option Setters
// -------------------------------------------------------------
void set_hardware_exposure(float val) {
    if (val < 200.0f) val = 200.0f;
    float max_exp = (g_target_t265_fps.load() == 90) ? 10000.0f : 16000.0f;
    if (val > max_exp) val = max_exp;
    g_exposure_f = val;

    if (g_has_t265 && g_t265_sensor) {
        try {
            g_t265_sensor.set_option(RS2_OPTION_EXPOSURE, val);
        } catch (const std::exception& e) {
            std::cerr << "[T265] Error setting exposure: " << e.what() << "\n";
        }
    }
}

void set_hardware_gain(float val) {
    if (val < 1.0f) val = 1.0f;
    if (val > 10.0f) val = 10.0f;
    g_gain_f = val;

    if (g_has_t265 && g_t265_sensor) {
        try {
            g_t265_sensor.set_option(RS2_OPTION_GAIN, val);
        } catch (const std::exception& e) {
            std::cerr << "[T265] Error setting gain: " << e.what() << "\n";
        }
    }
}

// -------------------------------------------------------------
// Frame Callbacks
// -------------------------------------------------------------
void d435_frame_callback(const rs2::frame& f) {
    if (!g_d435_enabled && g_rec_state != REC_RECORDING) return;
    if (auto fs = f.as<rs2::frameset>()) {
        auto ir1 = fs.get_infrared_frame(1);
        auto ir2 = fs.get_infrared_frame(2);
        if (ir1) {
            uint64_t fn = ir1.get_frame_number();
            if (fn != g_last_ir1_fn.load()) {
                g_last_ir1_fn = fn;
                g_d435_left_cnt++;
                int w = ir1.get_width();
                int h = ir1.get_height();
                std::lock_guard<std::mutex> lock(g_d435_mutex);
                if (g_d435_left_mat.rows != h || g_d435_left_mat.cols != w) {
                    g_d435_left_mat.create(h, w, CV_8UC1);
                }
                memcpy(g_d435_left_mat.data, ir1.get_data(), w * h);
            }
        }
        if (ir2) {
            uint64_t fn = ir2.get_frame_number();
            if (fn != g_last_ir2_fn.load()) {
                g_last_ir2_fn = fn;
                g_d435_right_cnt++;
                int w = ir2.get_width();
                int h = ir2.get_height();
                std::lock_guard<std::mutex> lock(g_d435_mutex);
                if (g_d435_right_mat.rows != h || g_d435_right_mat.cols != w) {
                    g_d435_right_mat.create(h, w, CV_8UC1);
                }
                memcpy(g_d435_right_mat.data, ir2.get_data(), w * h);
            }
        }

        // Benchmark buffer recording (in-memory)
        if (g_rec_state == REC_RECORDING) {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - g_rec_start_time).count();
            if (elapsed < 2.0) {
                if (ir1 && ir2) {
                    uint64_t fn = ir1.get_frame_number();
                    if (fn != g_last_rec_d435_fn.load()) {
                        g_last_rec_d435_fn = fn;
                        int w = ir1.get_width();
                        int h = ir1.get_height();
                        cv::Mat m1(h, w, CV_8UC1);
                        memcpy(m1.data, ir1.get_data(), w * h);
                        cv::Mat m2(h, w, CV_8UC1);
                        memcpy(m2.data, ir2.get_data(), w * h);
                        std::lock_guard<std::mutex> lock(g_rec_mutex);
                        g_rec_d435_left.push_back(std::move(m1));
                        g_rec_d435_right.push_back(std::move(m2));
                    }
                }
            }
        }
    }
}

// T265 Frame Callback shared between initial startup and dynamic switches
void t265_frame_callback(const rs2::frame& f) {
    if (!g_t265_enabled && g_rec_state != REC_RECORDING) return;
    int idx = f.get_profile().stream_index();
    uint64_t fn = f.get_frame_number();
    if (idx == 1) {
        if (fn != g_last_t265_f1_fn.load()) {
            g_last_t265_f1_fn = fn;
            g_t265_left_cnt++;
            std::lock_guard<std::mutex> lock(g_t265_mutex);
            memcpy(g_t265_left_mat.data, f.get_data(), 848 * 800);
        }
    } else if (idx == 2) {
        if (fn != g_last_t265_f2_fn.load()) {
            g_last_t265_f2_fn = fn;
            g_t265_right_cnt++;
            std::lock_guard<std::mutex> lock(g_t265_mutex);
            memcpy(g_t265_right_mat.data, f.get_data(), 848 * 800);
        }
    }

    // Benchmark buffer recording (in-memory at 30, 60, or 90 FPS)
    if (g_rec_state == REC_RECORDING) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - g_rec_start_time).count();
        if (elapsed < 2.0) {
            if (idx == 1 && fn != g_last_rec_t265_l_fn.load()) {
                g_last_rec_t265_l_fn = fn;
                cv::Mat m(800, 848, CV_8UC1);
                memcpy(m.data, f.get_data(), 848 * 800);
                std::lock_guard<std::mutex> lock(g_rec_mutex);
                g_rec_t265_left.push_back(std::move(m));
            } else if (idx == 2 && fn != g_last_rec_t265_r_fn.load()) {
                g_last_rec_t265_r_fn = fn;
                cv::Mat m(800, 848, CV_8UC1);
                memcpy(m.data, f.get_data(), 848 * 800);
                std::lock_guard<std::mutex> lock(g_rec_mutex);
                g_rec_t265_right.push_back(std::move(m));
            }
        }
    }
}

// -------------------------------------------------------------
// D435i Dynamic Profile Switching
// -------------------------------------------------------------
void switch_d435_profile(int width, int height, int fps) {
    if (g_switching_d435.load()) {
        std::cout << "[D435i] Mode switch already in progress.\n";
        return;
    }
    if (width == g_d435_width.load() && height == g_d435_height.load() && fps == g_target_d435_fps.load()) {
        char buf[64];
        snprintf(buf, sizeof(buf), "D435i already operating at %dx%d @ %d FPS", width, height, fps);
        set_banner(buf);
        return;
    }

    g_switching_d435 = true;
    int prev_w = g_d435_width.load();
    int prev_h = g_d435_height.load();
    int prev_fps = g_target_d435_fps.load();

    g_d435_width = width;
    g_d435_height = height;
    g_target_d435_fps = fps;

    char b_text[128];
    snprintf(b_text, sizeof(b_text), "SWITCHING D435i: %dx%d @ %d FPS...", width, height, fps);
    set_banner(b_text);

    std::thread([width, height, fps, prev_w, prev_h, prev_fps]() {
        try {
            if (g_d435_pipe) {
                g_d435_pipe->stop();
            }

            {
                std::lock_guard<std::mutex> lock(g_d435_mutex);
                g_d435_left_mat = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
                g_d435_right_mat = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
            }

            rs2::config cfg;
            cfg.enable_device(g_d435_serial);
            cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
            cfg.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);

            g_d435_left_cnt = 0;
            g_d435_right_cnt = 0;
            g_last_ir1_fn = 0;
            g_last_ir2_fn = 0;

            g_d435_pipe->start(cfg, d435_frame_callback);

            if (g_d435_stereo_sensor && g_d435_stereo_sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
                g_d435_stereo_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, g_emitter_enabled ? 1.0f : 0.0f);
            }

            g_switching_d435 = false;
            char msg[128];
            snprintf(msg, sizeof(msg), "D435i successfully switched to %dx%d @ %d FPS!", width, height, fps);
            set_banner(msg);
            std::cout << "[D435i] Stream running at " << width << "x" << height << " @ " << fps << " FPS.\n";

        } catch (const std::exception& e) {
            std::cerr << "[D435i] Switch error: " << e.what() << "\n";
            set_banner(std::string("D435i ERROR: ") + e.what());
            g_d435_width = prev_w;
            g_d435_height = prev_h;
            g_target_d435_fps = prev_fps;
            g_switching_d435 = false;
        }
    }).detach();
}

// -------------------------------------------------------------
// T265 Reflash & Dynamic FPS Switching Engine
// -------------------------------------------------------------
bool flash_t265(int fps) {
    std::string fw_file;
    std::vector<std::string> search_paths = {
        "visualizer/firmware/target-" + std::to_string(fps) + "fps.mvcmd",
        "firmware/target-" + std::to_string(fps) + "fps.mvcmd",
        "build/common/fw/target-" + std::to_string(fps) + "fps.mvcmd"
    };
    for (const auto& path : search_paths) {
        if (std::filesystem::exists(path)) {
            fw_file = path;
            break;
        }
    }
    if (fw_file.empty()) {
        std::cerr << "[FLASH] Error: Firmware file for " << fps << " FPS not found!\n";
        return false;
    }

    std::ifstream ifs(fw_file, std::ios::binary | std::ios::ate);
    if (!ifs.is_open()) {
        std::cerr << "[FLASH] Error opening " << fw_file << "\n";
        return false;
    }
    std::streamsize size = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    std::vector<uint8_t> buffer(size);
    if (!ifs.read(reinterpret_cast<char*>(buffer.data()), size)) {
        std::cerr << "[FLASH] Error reading " << fw_file << "\n";
        return false;
    }

    libusb_context* usb_ctx = nullptr;
    if (libusb_init(&usb_ctx) < 0) {
        std::cerr << "[FLASH] libusb_init failed!\n";
        return false;
    }

    // 1. Reset T265 to bootloader if currently booted (8087:0b37)
    libusb_device_handle* h = libusb_open_device_with_vid_pid(usb_ctx, 0x8087, 0x0b37);
    if (h) {
        std::cout << "[FLASH] Resetting T265 (8087:0b37) to bootloader...\n";
        libusb_control_transfer(h, 0x40, 0x10, 0, 0, nullptr, 0, 1000);
        libusb_close(h);
    }

    // 2. Poll for bootloader 03e7:2150
    libusb_device_handle* unbooted = nullptr;
    for (int i = 0; i < 40; ++i) {
        unbooted = libusb_open_device_with_vid_pid(usb_ctx, 0x03e7, 0x2150);
        if (unbooted) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!unbooted) {
        std::cerr << "[FLASH] Error: Bootloader 03e7:2150 not detected!\n";
        libusb_exit(usb_ctx);
        return false;
    }

    // 3. Upload firmware via bulk transfer endpoint 0x01
    std::cout << "[FLASH] Uploading " << fw_file << " (" << buffer.size() << " bytes)...\n";
    libusb_claim_interface(unbooted, 0);
    int transferred = 0;
    int bt = libusb_bulk_transfer(unbooted, 0x01, buffer.data(), static_cast<int>(buffer.size()), &transferred, 5000);
    libusb_release_interface(unbooted, 0);
    libusb_close(unbooted);
    if (bt < 0) {
        std::cerr << "[FLASH] Bulk transfer failed: " << bt << "\n";
        libusb_exit(usb_ctx);
        return false;
    }

    // 4. Wait for T265 to re-enumerate as 8087:0b37
    std::cout << "[FLASH] Waiting for T265 to re-enumerate as 8087:0b37...\n";
    bool booted = false;
    for (int i = 0; i < 50; ++i) {
        libusb_device_handle* bh = libusb_open_device_with_vid_pid(usb_ctx, 0x8087, 0x0b37);
        if (bh) {
            libusb_close(bh);
            booted = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    libusb_exit(usb_ctx);

    // Give firmware an extra 600ms to initialize its internal USB descriptors
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    return booted;
}

void switch_t265_fps(int new_fps) {
    if (g_switching_t265.load()) {
        std::cout << "[T265] Switch already in progress. Ignoring request.\n";
        return;
    }
    if (new_fps == g_target_t265_fps.load()) {
        char buf[64];
        snprintf(buf, sizeof(buf), "T265 is already operating at %d FPS", new_fps);
        set_banner(buf);
        return;
    }

    g_switching_t265 = true;
    int prev_fps = g_target_t265_fps.load();
    g_target_t265_fps = new_fps; // update target for UI immediate feedback
    char buf[128];
    snprintf(buf, sizeof(buf), "SWITCHING T265: Re-flashing firmware from %d to %d FPS...", prev_fps, new_fps);
    set_banner(buf);

    std::thread([new_fps, prev_fps]() {
        try {
            // 1. Safely stop and close current sensor
            if (g_has_t265 && g_t265_sensor) {
                try { g_t265_sensor.stop(); } catch (...) {}
                try { g_t265_sensor.close(); } catch (...) {}
            }

            // 2. Reflash firmware
            bool ok = flash_t265(new_fps);
            if (!ok) {
                char err_buf[128];
                snprintf(err_buf, sizeof(err_buf), "ERROR: Failed to upload %d FPS firmware!", new_fps);
                set_banner(err_buf);
                g_target_t265_fps = prev_fps;
                g_switching_t265 = false;
                return;
            }

            // 3. Find newly booted T265 device in librealsense context
            rs2::device t265_dev;
            for (int retry = 0; retry < 40; ++retry) {
                if (g_rs_ctx) {
                    for (auto&& d : g_rs_ctx->query_devices()) {
                        if (std::string(d.get_info(RS2_CAMERA_INFO_NAME)).find("T265") != std::string::npos) {
                            t265_dev = d;
                            break;
                        }
                    }
                }
                if (t265_dev) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(150));
            }

            if (!t265_dev) {
                set_banner("ERROR: T265 device not found after reflash!");
                g_target_t265_fps = prev_fps;
                g_switching_t265 = false;
                return;
            }

            g_t265_sensor = t265_dev.query_sensors()[0];

            // 4. Disable auto-exposure and set exposure / gain
            if (g_t265_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
                g_t265_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.0f);
            }

            // Clamp exposure if 90 FPS mode
            float cur_exp = g_exposure_f.load();
            if (new_fps == 90 && cur_exp > 10000.0f) {
                cur_exp = 10000.0f;
                g_exposure_f = cur_exp;
            }
            set_hardware_exposure(cur_exp);
            set_hardware_gain(g_gain_f.load());

            // 5. Query matching stream profiles for target FPS
            std::vector<rs2::stream_profile> profiles;
            for (auto&& p : g_t265_sensor.get_stream_profiles()) {
                if (auto vp = p.as<rs2::video_stream_profile>()) {
                    if (vp.stream_type() == RS2_STREAM_FISHEYE && vp.fps() == new_fps) {
                        profiles.push_back(p);
                    }
                }
            }

            if (profiles.empty()) {
                char err_buf[128];
                snprintf(err_buf, sizeof(err_buf), "ERROR: No stream profiles found for %d FPS!", new_fps);
                set_banner(err_buf);
                g_target_t265_fps = prev_fps;
                g_switching_t265 = false;
                return;
            }

            // 6. Reset frame counters and start sensor
            g_t265_left_cnt = 0;
            g_t265_right_cnt = 0;
            g_last_t265_f1_fn = 0;
            g_last_t265_f2_fn = 0;

            g_t265_sensor.open(profiles);
            g_t265_sensor.start(t265_frame_callback);

            g_switching_t265 = false;
            char succ_buf[128];
            snprintf(succ_buf, sizeof(succ_buf), "T265 successfully switched to %d FPS!", new_fps);
            set_banner(succ_buf);
            std::cout << "[T265] Stream running at " << new_fps << " FPS.\n";

        } catch (const std::exception& e) {
            std::cerr << "[T265] Exception during switch: " << e.what() << "\n";
            set_banner(std::string("ERROR: ") + e.what());
            g_target_t265_fps = prev_fps;
            g_switching_t265 = false;
        }
    }).detach();
}

// -------------------------------------------------------------
// 2-Second Benchmark Recording Engine
// -------------------------------------------------------------
void save_benchmark_footage(double actual_duration) {
    std::cout << "\n[RECORD] 2-second capture window complete (" << std::fixed << std::setprecision(3) 
              << actual_duration << "s). Saving buffered images to disk...\n";

    std::vector<cv::Mat> d435_l, d435_r, t265_l, t265_r;
    {
        std::lock_guard<std::mutex> lock(g_rec_mutex);
        d435_l.swap(g_rec_d435_left);
        d435_r.swap(g_rec_d435_right);
        t265_l.swap(g_rec_t265_left);
        t265_r.swap(g_rec_t265_right);
    }

    const std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, 95};

    // Save D435i frames
    size_t d435_count = std::max(d435_l.size(), d435_r.size());
    int d435_w = g_d435_width.load();
    int d435_h = g_d435_height.load();
    for (size_t i = 0; i < d435_count; ++i) {
        size_t x = i + 1;
        if (i < d435_l.size()) {
            std::string path = "captures_d435i/d435i_left_" + std::to_string(x) + ".jpg";
            cv::imwrite(path, d435_l[i], jpeg_params);
        }
        if (i < d435_r.size()) {
            std::string path = "captures_d435i/d435i_right_" + std::to_string(x) + ".jpg";
            cv::imwrite(path, d435_r[i], jpeg_params);
        }
    }

    // Save T265 frames
    size_t t265_count = std::max(t265_l.size(), t265_r.size());
    for (size_t i = 0; i < t265_count; ++i) {
        size_t y = i + 1;
        if (i < t265_l.size()) {
            std::string path = "captures_t265/t265_left_" + std::to_string(y) + ".jpg";
            cv::imwrite(path, t265_l[i], jpeg_params);
        }
        if (i < t265_r.size()) {
            std::string path = "captures_t265/t265_right_" + std::to_string(y) + ".jpg";
            cv::imwrite(path, t265_r[i], jpeg_params);
        }
    }

    double d435_fps = (actual_duration > 0) ? (d435_l.size() / actual_duration) : 0.0;
    double t265_fps = (actual_duration > 0) ? (t265_l.size() / actual_duration) : 0.0;

    // Write metadata summaries
    std::ofstream ofs_d435("captures_d435i/benchmark_summary.txt");
    if (ofs_d435.is_open()) {
        ofs_d435 << "=== Intel RealSense D435i 2-Second Benchmark ===\n";
        ofs_d435 << "Duration: " << actual_duration << " seconds\n";
        ofs_d435 << "Mode: " << g_target_d435_fps.load() << " FPS\n";
        ofs_d435 << "Resolution: " << d435_w << "x" << d435_h << "\n";
        ofs_d435 << "Left Frames: " << d435_l.size() << " (FPS: " << d435_fps << ")\n";
        ofs_d435 << "Right Frames: " << d435_r.size() << " (FPS: " << d435_fps << ")\n";
    }

    std::ofstream ofs_t265("captures_t265/benchmark_summary.txt");
    if (ofs_t265.is_open()) {
        ofs_t265 << "=== Intel RealSense T265 2-Second Benchmark ===\n";
        ofs_t265 << "Duration: " << actual_duration << " seconds\n";
        ofs_t265 << "Mode: " << g_target_t265_fps.load() << " FPS\n";
        ofs_t265 << "Resolution: 848x800\n";
        ofs_t265 << "Left Frames: " << t265_l.size() << " (FPS: " << t265_fps << ")\n";
        ofs_t265 << "Right Frames: " << t265_r.size() << " (FPS: " << t265_fps << ")\n";
    }

    std::cout << "\n=======================================================\n";
    std::cout << "[BENCHMARK COMPLETE] 2-Second Capture Summary:\n";
    std::cout << "  Duration:     " << std::fixed << std::setprecision(3) << actual_duration << " seconds\n";
    std::cout << "  D435i Saved:  " << d435_l.size() << " left + " << d435_r.size() 
              << " right frames (" << d435_w << "x" << d435_h << ") in captures_d435i/ (" 
              << std::fixed << std::setprecision(1) << d435_fps << " FPS)\n";
    std::cout << "  T265 Saved:   " << t265_l.size() << " left + " << t265_r.size() 
              << " right frames in captures_t265/ (" << std::fixed << std::setprecision(1) << t265_fps << " FPS)\n";
    std::cout << "=======================================================\n" << std::endl;

    char banner[256];
    snprintf(banner, sizeof(banner), "CAPTURED 2s: D435i (%dx%d): %zu frames (%.1f FPS) | T265: %zu frames (%.1f FPS)",
             d435_w, d435_h, d435_l.size(), d435_fps, t265_l.size(), t265_fps);
    set_banner(banner);

    g_rec_state = REC_IDLE;
}

void trigger_record_2s() {
    if (g_rec_state != REC_IDLE) {
        std::cout << "[RECORD] Already recording or saving. Please wait...\n";
        return;
    }

    std::cout << "[RECORD] Clearing previous capture folders...\n";
    try {
        std::filesystem::remove_all("captures_d435i");
        std::filesystem::create_directories("captures_d435i");
        std::filesystem::remove_all("captures_t265");
        std::filesystem::create_directories("captures_t265");
    } catch (const std::exception& e) {
        std::cerr << "[RECORD] Error initializing folders: " << e.what() << "\n";
    }

    {
        std::lock_guard<std::mutex> lock(g_rec_mutex);
        g_rec_d435_left.clear();
        g_rec_d435_right.clear();
        g_rec_t265_left.clear();
        g_rec_t265_right.clear();
        g_rec_d435_left.reserve(700);
        g_rec_d435_right.reserve(700);
        g_rec_t265_left.reserve(250);
        g_rec_t265_right.reserve(250);
        g_last_rec_d435_fn = 0;
        g_last_rec_t265_l_fn = 0;
        g_last_rec_t265_r_fn = 0;
    }

    std::cout << "[RECORD] Started 2-second capture benchmark! Buffering frames in memory...\n";
    g_rec_start_time = std::chrono::steady_clock::now();
    g_rec_state = REC_RECORDING;
}

// -------------------------------------------------------------
// Mouse Callbacks for Controller Window
// -------------------------------------------------------------
void on_controller_mouse_click(int event, int x, int y, int flags, void*) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        // 1. Top Big Red Record Button
        if (RECT_CTRL_RECORD.contains(cv::Point(x, y))) {
            trigger_record_2s();
            return;
        }

        // 2. D435i Stream Toggle
        if (RECT_CTRL_D435_STREAM.contains(cv::Point(x, y))) {
            g_d435_enabled = !g_d435_enabled;
            std::cout << "[CTRL] D435i Stream: " << (g_d435_enabled ? "ON" : "OFF") << std::endl;
            return;
        }

        // 3. D435i IR Emitter Toggle
        if (RECT_CTRL_D435_EMITTER.contains(cv::Point(x, y))) {
            if (g_has_d435 && g_d435_stereo_sensor) {
                g_emitter_enabled = !g_emitter_enabled;
                try {
                    g_d435_stereo_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, g_emitter_enabled ? 1.0f : 0.0f);
                    std::cout << "[CTRL] D435i Emitter: " << (g_emitter_enabled ? "ON" : "OFF") << std::endl;
                } catch (const std::exception& e) {
                    std::cerr << "[D435i] Error setting emitter: " << e.what() << "\n";
                }
            }
            return;
        }

        // 4. D435i Table FPS Buttons
        int row_y_start = 214;
        int row_h = 26;
        for (size_t r = 0; r < D435_TABLE.size(); ++r) {
            int ry = row_y_start + r * row_h;
            for (size_t c = 0; c < D435_FPS_HEADERS.size(); ++c) {
                int cx = 160 + c * 56;
                cv::Rect btn_rect(cx + 2, ry + 2, 52, 22);
                if (btn_rect.contains(cv::Point(x, y))) {
                    int fps_val = D435_TABLE[r].fps_slots[c];
                    if (fps_val > 0) {
                        switch_d435_profile(D435_TABLE[r].width, D435_TABLE[r].height, fps_val);
                    }
                    return;
                }
            }
        }

        // 5. T265 Stream Toggle
        if (RECT_CTRL_T265_STREAM.contains(cv::Point(x, y))) {
            g_t265_enabled = !g_t265_enabled;
            std::cout << "[CTRL] T265 Stream: " << (g_t265_enabled ? "ON" : "OFF") << std::endl;
            return;
        }

        // 6. T265 Table FPS Buttons (848x800 | 30, 60, 90)
        int t265_y = 510;
        int t265_fps_vals[3] = {30, 60, 90};
        for (int c = 0; c < 3; ++c) {
            int cx = 160 + c * 56;
            cv::Rect btn_rect(cx + 2, t265_y + 2, 52, 22);
            if (btn_rect.contains(cv::Point(x, y))) {
                switch_t265_fps(t265_fps_vals[c]);
                return;
            }
        }

        // 7. Exposure Track Click / Drag
        if (RECT_CTRL_EXP_TRACK.contains(cv::Point(x, y))) {
            g_dragging_exp = true;
            float frac = (x - RECT_CTRL_EXP_TRACK.x) / static_cast<float>(RECT_CTRL_EXP_TRACK.width);
            frac = std::clamp(frac, 0.0f, 1.0f);
            float max_exp = (g_target_t265_fps.load() == 90) ? 10000.0f : 16000.0f;
            float val = 200.0f + frac * (max_exp - 200.0f);
            set_hardware_exposure(val);
            return;
        }

        // 8. Gain Track Click / Drag
        if (RECT_CTRL_GAIN_TRACK.contains(cv::Point(x, y))) {
            g_dragging_gain = true;
            float frac = (x - RECT_CTRL_GAIN_TRACK.x) / static_cast<float>(RECT_CTRL_GAIN_TRACK.width);
            frac = std::clamp(frac, 0.0f, 1.0f);
            float val = 1.0f + frac * 9.0f;
            set_hardware_gain(val);
            return;
        }
    } else if (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON)) {
        if (g_dragging_exp) {
            float frac = (x - RECT_CTRL_EXP_TRACK.x) / static_cast<float>(RECT_CTRL_EXP_TRACK.width);
            frac = std::clamp(frac, 0.0f, 1.0f);
            float max_exp = (g_target_t265_fps.load() == 90) ? 10000.0f : 16000.0f;
            float val = 200.0f + frac * (max_exp - 200.0f);
            set_hardware_exposure(val);
        } else if (g_dragging_gain) {
            float frac = (x - RECT_CTRL_GAIN_TRACK.x) / static_cast<float>(RECT_CTRL_GAIN_TRACK.width);
            frac = std::clamp(frac, 0.0f, 1.0f);
            float val = 1.0f + frac * 9.0f;
            set_hardware_gain(val);
        }
    } else if (event == cv::EVENT_LBUTTONUP) {
        g_dragging_exp = false;
        g_dragging_gain = false;
    }
}

// -------------------------------------------------------------
// UI Rendering Helpers
// -------------------------------------------------------------
void draw_bottom_left_badge(cv::Mat& img, const std::string& text, cv::Point origin) {
    int baseline = 0;
    double font_scale = 0.44;
    int thickness = 1;
    cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline);
    
    cv::Rect bg_rect(origin.x - 5, origin.y - text_size.height - 4, text_size.width + 10, text_size.height + baseline + 8);
    if (bg_rect.x >= 0 && bg_rect.y >= 0 && 
        bg_rect.x + bg_rect.width <= img.cols && bg_rect.y + bg_rect.height <= img.rows) {
        cv::rectangle(img, bg_rect, cv::Scalar(24, 24, 24), cv::FILLED);
        cv::rectangle(img, bg_rect, cv::Scalar(60, 60, 60), 1);
    }
    cv::putText(img, text, origin, cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(220, 220, 220), thickness, cv::LINE_AA);
}

void render_controller_window(cv::Mat& ctrl) {
    ctrl.setTo(cv::Scalar(22, 22, 25));

    // 1. Top Title Bar
    cv::Rect title_bar(0, 0, 520, 38);
    cv::rectangle(ctrl, title_bar, cv::Scalar(30, 30, 35), cv::FILLED);
    cv::line(ctrl, cv::Point(0, 38), cv::Point(520, 38), cv::Scalar(60, 60, 65), 1);
    cv::putText(ctrl, "REALSENSE STREAM CONTROLLER", cv::Point(120, 25),
                cv::FONT_HERSHEY_SIMPLEX, 0.52, cv::Scalar(240, 240, 240), 1, cv::LINE_AA);

    // 2. Top Big Red Record Button
    auto r_state = g_rec_state.load();
    if (r_state == REC_IDLE) {
        cv::rectangle(ctrl, RECT_CTRL_RECORD, cv::Scalar(30, 25, 140), cv::FILLED);
        cv::rectangle(ctrl, RECT_CTRL_RECORD, cv::Scalar(50, 50, 220), 2);
        cv::circle(ctrl, cv::Point(RECT_CTRL_RECORD.x + 30, RECT_CTRL_RECORD.y + 22), 8, cv::Scalar(40, 40, 240), cv::FILLED, cv::LINE_AA);
        cv::putText(ctrl, "RECORD 2s BENCHMARK (R)", cv::Point(RECT_CTRL_RECORD.x + 55, RECT_CTRL_RECORD.y + 28),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    } else if (r_state == REC_RECORDING) {
        double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - g_rec_start_time).count();
        double remaining = std::max(0.0, 2.0 - elapsed);
        cv::rectangle(ctrl, RECT_CTRL_RECORD, cv::Scalar(20, 20, 210), cv::FILLED);
        cv::rectangle(ctrl, RECT_CTRL_RECORD, cv::Scalar(255, 255, 255), 2);
        cv::rectangle(ctrl, cv::Rect(RECT_CTRL_RECORD.x + 22, RECT_CTRL_RECORD.y + 14, 16, 16), cv::Scalar(255, 255, 255), cv::FILLED);
        char buf[64];
        snprintf(buf, sizeof(buf), "RECORDING... %.1fs REMAINING", remaining);
        cv::putText(ctrl, buf, cv::Point(RECT_CTRL_RECORD.x + 55, RECT_CTRL_RECORD.y + 28),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    } else if (r_state == REC_SAVING) {
        cv::rectangle(ctrl, RECT_CTRL_RECORD, cv::Scalar(0, 130, 220), cv::FILLED);
        cv::rectangle(ctrl, RECT_CTRL_RECORD, cv::Scalar(255, 255, 255), 2);
        cv::putText(ctrl, "SAVING BUFFERED IMAGES TO DISK...", cv::Point(RECT_CTRL_RECORD.x + 45, RECT_CTRL_RECORD.y + 28),
                    cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    }

    // 3. Section 1: D435i Controls
    cv::putText(ctrl, "D435i PREFERENCES & RESOLUTION TABLE", cv::Point(20, 125),
                cv::FONT_HERSHEY_SIMPLEX, 0.44, cv::Scalar(0, 180, 255), 1, cv::LINE_AA);
    cv::line(ctrl, cv::Point(20, 132), cv::Point(500, 132), cv::Scalar(55, 55, 60), 1);

    auto draw_toggle_btn = [&](cv::Rect rect, bool state, const std::string& label) {
        cv::rectangle(ctrl, rect, cv::Scalar(32, 32, 36), cv::FILLED);
        cv::rectangle(ctrl, rect, state ? cv::Scalar(50, 180, 50) : cv::Scalar(70, 70, 75), 1);
        cv::circle(ctrl, cv::Point(rect.x + 16, rect.y + 15), 6, state ? cv::Scalar(40, 220, 40) : cv::Scalar(70, 70, 70), cv::FILLED, cv::LINE_AA);
        cv::putText(ctrl, label, cv::Point(rect.x + 30, rect.y + 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.40, state ? cv::Scalar(235, 235, 235) : cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
    };

    draw_toggle_btn(RECT_CTRL_D435_STREAM, g_d435_enabled.load(), "D435i Stream");
    draw_toggle_btn(RECT_CTRL_D435_EMITTER, g_emitter_enabled.load(), "IR Emitter");

    // Table Header (Res | FPS)
    cv::Rect d435_th(20, 186, 480, 24);
    cv::rectangle(ctrl, d435_th, cv::Scalar(36, 36, 42), cv::FILLED);
    cv::rectangle(ctrl, d435_th, cv::Scalar(65, 65, 75), 1);
    cv::putText(ctrl, "Res", cv::Point(26, 203), cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(210, 210, 210), 1, cv::LINE_AA);
    cv::line(ctrl, cv::Point(155, 186), cv::Point(155, 210), cv::Scalar(65, 65, 75), 1);

    for (size_t c = 0; c < D435_FPS_HEADERS.size(); ++c) {
        int cx = 160 + c * 56;
        std::string txt = std::to_string(D435_FPS_HEADERS[c]);
        int bl = 0;
        cv::Size sz = cv::getTextSize(txt, cv::FONT_HERSHEY_SIMPLEX, 0.40, 1, &bl);
        cv::putText(ctrl, txt, cv::Point(cx + (52 - sz.width) / 2, 203),
                    cv::FONT_HERSHEY_SIMPLEX, 0.40, cv::Scalar(210, 210, 210), 1, cv::LINE_AA);
    }

    // Table Rows
    int cur_w = g_d435_width.load();
    int cur_h = g_d435_height.load();
    int cur_fps = g_target_d435_fps.load();

    int row_y_start = 214;
    int row_h = 26;
    for (size_t r = 0; r < D435_TABLE.size(); ++r) {
        int ry = row_y_start + r * row_h;
        cv::Rect r_label_box(20, ry, 135, row_h - 2);
        cv::rectangle(ctrl, r_label_box, cv::Scalar(28, 28, 32), cv::FILLED);
        cv::rectangle(ctrl, r_label_box, cv::Scalar(48, 48, 55), 1);
        cv::putText(ctrl, D435_TABLE[r].label, cv::Point(26, ry + 17),
                    cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);

        for (size_t c = 0; c < D435_FPS_HEADERS.size(); ++c) {
            int cx = 160 + c * 56;
            cv::Rect cell_rect(cx + 2, ry + 2, 52, 22);
            int fps_val = D435_TABLE[r].fps_slots[c];

            if (fps_val > 0) {
                bool is_active = (cur_w == D435_TABLE[r].width && cur_h == D435_TABLE[r].height && cur_fps == fps_val);
                if (is_active) {
                    cv::rectangle(ctrl, cell_rect, cv::Scalar(35, 160, 45), cv::FILLED);
                    cv::rectangle(ctrl, cell_rect, cv::Scalar(100, 255, 120), 1);
                    std::string txt = std::to_string(fps_val);
                    int bl = 0;
                    cv::Size sz = cv::getTextSize(txt, cv::FONT_HERSHEY_SIMPLEX, 0.40, 1, &bl);
                    cv::putText(ctrl, txt, cv::Point(cell_rect.x + (cell_rect.width - sz.width) / 2, cell_rect.y + 16),
                                cv::FONT_HERSHEY_SIMPLEX, 0.40, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
                } else {
                    cv::rectangle(ctrl, cell_rect, cv::Scalar(42, 42, 48), cv::FILLED);
                    cv::rectangle(ctrl, cell_rect, cv::Scalar(75, 75, 85), 1);
                    std::string txt = std::to_string(fps_val);
                    int bl = 0;
                    cv::Size sz = cv::getTextSize(txt, cv::FONT_HERSHEY_SIMPLEX, 0.38, 1, &bl);
                    cv::putText(ctrl, txt, cv::Point(cell_rect.x + (cell_rect.width - sz.width) / 2, cell_rect.y + 15),
                                cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(210, 210, 210), 1, cv::LINE_AA);
                }
            } else {
                cv::rectangle(ctrl, cell_rect, cv::Scalar(24, 24, 28), cv::FILLED);
                cv::rectangle(ctrl, cell_rect, cv::Scalar(40, 40, 45), 1);
                cv::putText(ctrl, "-", cv::Point(cell_rect.x + 22, cell_rect.y + 15),
                            cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(65, 65, 75), 1, cv::LINE_AA);
            }
        }
    }

    // 4. Section 2: T265 Controls
    cv::putText(ctrl, "T265 PREFERENCES & REFLASH TABLE", cv::Point(20, 418),
                cv::FONT_HERSHEY_SIMPLEX, 0.44, cv::Scalar(0, 180, 255), 1, cv::LINE_AA);
    cv::line(ctrl, cv::Point(20, 425), cv::Point(500, 425), cv::Scalar(55, 55, 60), 1);

    draw_toggle_btn(RECT_CTRL_T265_STREAM, g_t265_enabled.load(), "T265 Stream");

    // Table Header
    cv::Rect t265_th(20, 482, 480, 24);
    cv::rectangle(ctrl, t265_th, cv::Scalar(36, 36, 42), cv::FILLED);
    cv::rectangle(ctrl, t265_th, cv::Scalar(65, 65, 75), 1);
    cv::putText(ctrl, "Res", cv::Point(26, 499), cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(210, 210, 210), 1, cv::LINE_AA);
    cv::line(ctrl, cv::Point(155, 482), cv::Point(155, 506), cv::Scalar(65, 65, 75), 1);
    cv::putText(ctrl, "FPS Options (Dynamic Bootloader Reflash)", cv::Point(170, 499),
                cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(180, 180, 180), 1, cv::LINE_AA);

    // T265 Table Row: 848x800 | 30, 60, 90
    int t265_y = 510;
    cv::Rect t265_lbl_box(20, t265_y, 135, 26);
    cv::rectangle(ctrl, t265_lbl_box, cv::Scalar(28, 28, 32), cv::FILLED);
    cv::rectangle(ctrl, t265_lbl_box, cv::Scalar(48, 48, 55), 1);
    cv::putText(ctrl, "848 x 800", cv::Point(26, t265_y + 18),
                cv::FONT_HERSHEY_SIMPLEX, 0.40, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);

    int cur_t265_fps = g_target_t265_fps.load();
    bool t265_switching = g_switching_t265.load();
    int t265_fps_vals[3] = {30, 60, 90};

    for (int c = 0; c < 3; ++c) {
        int cx = 160 + c * 56;
        cv::Rect cell_rect(cx + 2, t265_y + 2, 52, 22);
        int fps_val = t265_fps_vals[c];
        bool is_active = (cur_t265_fps == fps_val);

        if (is_active) {
            if (t265_switching) {
                cv::rectangle(ctrl, cell_rect, cv::Scalar(0, 140, 230), cv::FILLED);
                cv::rectangle(ctrl, cell_rect, cv::Scalar(255, 255, 255), 1);
            } else {
                cv::rectangle(ctrl, cell_rect, cv::Scalar(160, 120, 20), cv::FILLED);
                cv::rectangle(ctrl, cell_rect, cv::Scalar(240, 200, 60), 1);
            }
            std::string txt = std::to_string(fps_val);
            int bl = 0;
            cv::Size sz = cv::getTextSize(txt, cv::FONT_HERSHEY_SIMPLEX, 0.40, 1, &bl);
            cv::putText(ctrl, txt, cv::Point(cell_rect.x + (cell_rect.width - sz.width) / 2, cell_rect.y + 16),
                        cv::FONT_HERSHEY_SIMPLEX, 0.40, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        } else {
            cv::rectangle(ctrl, cell_rect, cv::Scalar(42, 42, 48), cv::FILLED);
            cv::rectangle(ctrl, cell_rect, cv::Scalar(75, 75, 85), 1);
            std::string txt = std::to_string(fps_val);
            int bl = 0;
            cv::Size sz = cv::getTextSize(txt, cv::FONT_HERSHEY_SIMPLEX, 0.38, 1, &bl);
            cv::putText(ctrl, txt, cv::Point(cell_rect.x + (cell_rect.width - sz.width) / 2, cell_rect.y + 15),
                        cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(210, 210, 210), 1, cv::LINE_AA);
        }
    }

    // 5. Exposure & Gain Sliders
    float cur_exp = g_exposure_f.load();
    float max_exp = (cur_t265_fps == 90) ? 10000.0f : 16000.0f;
    float exp_frac = std::clamp((cur_exp - 200.0f) / (max_exp - 200.0f), 0.0f, 1.0f);

    float cur_gain = g_gain_f.load();
    float gain_frac = std::clamp((cur_gain - 1.0f) / 9.0f, 0.0f, 1.0f);

    // Exposure Slider
    char exp_lbl[64];
    snprintf(exp_lbl, sizeof(exp_lbl), "Exposure: %.0f us", cur_exp);
    cv::putText(ctrl, exp_lbl, cv::Point(20, 570), cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);

    cv::rectangle(ctrl, RECT_CTRL_EXP_TRACK, cv::Scalar(35, 35, 40), cv::FILLED);
    cv::rectangle(ctrl, RECT_CTRL_EXP_TRACK, cv::Scalar(65, 65, 75), 1);
    int exp_fill_w = static_cast<int>(exp_frac * RECT_CTRL_EXP_TRACK.width);
    cv::rectangle(ctrl, cv::Rect(RECT_CTRL_EXP_TRACK.x, RECT_CTRL_EXP_TRACK.y, exp_fill_w, RECT_CTRL_EXP_TRACK.height),
                  cv::Scalar(180, 130, 20), cv::FILLED);
    cv::circle(ctrl, cv::Point(RECT_CTRL_EXP_TRACK.x + exp_fill_w, RECT_CTRL_EXP_TRACK.y + 8), 8, cv::Scalar(240, 240, 240), cv::FILLED);

    // Gain Slider
    char gain_lbl[64];
    snprintf(gain_lbl, sizeof(gain_lbl), "Gain: %.1fx", cur_gain);
    cv::putText(ctrl, gain_lbl, cv::Point(20, 614), cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);

    cv::rectangle(ctrl, RECT_CTRL_GAIN_TRACK, cv::Scalar(35, 35, 40), cv::FILLED);
    cv::rectangle(ctrl, RECT_CTRL_GAIN_TRACK, cv::Scalar(65, 65, 75), 1);
    int gain_fill_w = static_cast<int>(gain_frac * RECT_CTRL_GAIN_TRACK.width);
    cv::rectangle(ctrl, cv::Rect(RECT_CTRL_GAIN_TRACK.x, RECT_CTRL_GAIN_TRACK.y, gain_fill_w, RECT_CTRL_GAIN_TRACK.height),
                  cv::Scalar(40, 160, 40), cv::FILLED);
    cv::circle(ctrl, cv::Point(RECT_CTRL_GAIN_TRACK.x + gain_fill_w, RECT_CTRL_GAIN_TRACK.y + 8), 8, cv::Scalar(240, 240, 240), cv::FILLED);

    // 6. Footer Info
    cv::line(ctrl, cv::Point(20, 642), cv::Point(500, 642), cv::Scalar(50, 50, 55), 1);
    cv::putText(ctrl, "Hotkeys: R: Record | 1/2: Streams | E: Emitter | Q/ESC: Exit", cv::Point(22, 665),
                cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(140, 140, 140), 1, cv::LINE_AA);
    char stat_buf[128];
    snprintf(stat_buf, sizeof(stat_buf), "D435i: %dx%d @ %d FPS | T265: 848x800 @ %d FPS",
             cur_w, cur_h, cur_fps, cur_t265_fps);
    cv::putText(ctrl, stat_buf, cv::Point(22, 684),
                cv::FONT_HERSHEY_SIMPLEX, 0.36, cv::Scalar(0, 210, 255), 1, cv::LINE_AA);
}

// -------------------------------------------------------------
// Main Application
// -------------------------------------------------------------
int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "=========================================================\n";
    std::cout << "  RealSense Dual Stream Visualizer: D435i (2560x1520)    \n";
    std::cout << "=========================================================\n";

    try {
        g_rs_ctx = std::make_unique<rs2::context>();

        // Query available RealSense devices
        rs2::device d435_dev;
        rs2::device t265_dev;

        for (auto&& dev : g_rs_ctx->query_devices()) {
            std::string name = dev.get_info(RS2_CAMERA_INFO_NAME);
            std::cout << "Found device: " << name << " (S/N: " 
                      << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << ")\n";
            if (name.find("D435") != std::string::npos) {
                d435_dev = dev;
                g_has_d435 = true;
            } else if (name.find("T265") != std::string::npos) {
                t265_dev = dev;
                g_has_t265 = true;
            }
        }

        if (!g_has_d435 && !g_has_t265) {
            std::cerr << "Error: Neither D435i nor T265 device was detected!\n";
            return 1;
        }

        // -----------------------------------------------------
        // 1. Initialize D435i Pipeline (Default Fast: 848x100 @ 300 FPS)
        // -----------------------------------------------------
        if (g_has_d435) {
            g_d435_serial = d435_dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            std::cout << "\n[D435i] Initializing Infrared Stereo Streams (848x100 @ 300 FPS)...\n";
            rs2::config d435_cfg;
            d435_cfg.enable_device(g_d435_serial);
            d435_cfg.enable_stream(RS2_STREAM_INFRARED, 1, 848, 100, RS2_FORMAT_Y8, 300);
            d435_cfg.enable_stream(RS2_STREAM_INFRARED, 2, 848, 100, RS2_FORMAT_Y8, 300);

            // Locate Stereo Module sensor for emitter control
            for (auto&& s : d435_dev.query_sensors()) {
                if (s.supports(RS2_OPTION_EMITTER_ENABLED)) {
                    g_d435_stereo_sensor = s;
                    g_d435_stereo_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.0f); // Default OFF
                    std::cout << "[D435i] Infrared Emitter default: OFF\n";
                    break;
                }
            }

            g_d435_pipe = std::make_unique<rs2::pipeline>(*g_rs_ctx);
            g_d435_pipe->start(d435_cfg, d435_frame_callback);
            std::cout << "[D435i] Pipeline started successfully.\n";
        }

        // -----------------------------------------------------
        // 2. Initialize T265 Tracking Module (848x800 @ Detected FPS)
        // -----------------------------------------------------
        if (g_has_t265) {
            std::cout << "\n[T265] Initializing Fisheye Streams (848x800)...\n";
            g_t265_sensor = t265_dev.query_sensors()[0];

            // HARDCODE DISABLE AUTOEXPOSURE prior to opening/streaming
            if (g_t265_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
                g_t265_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.0f);
                std::cout << "[T265] Auto-Exposure HARDCODED to DISABLED (Manual Mode).\n";
            }

            // Detect available fisheye stream profile FPS (30, 60, or 90)
            int detected_fps = 30;
            std::vector<rs2::stream_profile> profiles_to_open;
            for (auto&& p : g_t265_sensor.get_stream_profiles()) {
                if (auto vp = p.as<rs2::video_stream_profile>()) {
                    if (vp.stream_type() == RS2_STREAM_FISHEYE) {
                        detected_fps = vp.fps();
                        profiles_to_open.push_back(p);
                    }
                }
            }
            g_target_t265_fps = detected_fps;
            std::cout << "[T265] Configured Fisheye Streams (848x800 @ " << g_target_t265_fps.load() << " FPS).\n";

            g_t265_sensor.open(profiles_to_open);
            g_t265_sensor.start(t265_frame_callback);

            // Set initial hardware exposure and gain values
            set_hardware_exposure(g_exposure_f);
            set_hardware_gain(g_gain_f);
            std::cout << "[T265] Stream started (Exp: " << g_exposure_f 
                      << " us, Gain: " << g_gain_f << "x).\n";
        }

        // -----------------------------------------------------
        // 3. Setup OpenCV Dual Windows (Stream + Separate Controller)
        // -----------------------------------------------------
        cv::namedWindow(STREAM_WINDOW, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(CONTROLLER_WINDOW, cv::WINDOW_AUTOSIZE);

        cv::setMouseCallback(CONTROLLER_WINDOW, on_controller_mouse_click, nullptr);

        // Position windows cleanly on desktop
        cv::moveWindow(STREAM_WINDOW, 0, 0);
        cv::moveWindow(CONTROLLER_WINDOW, 2040, 40);

        // Mats for Rendering: Fixed 2560 x 1520 Stream Canvas
        cv::Mat display_d435(D435_SLOT_HEIGHT, CANVAS_WIDTH, CV_8UC3, cv::Scalar(15, 15, 15));
        cv::Mat display_t265(T265_SLOT_HEIGHT, CANVAS_WIDTH, CV_8UC3, cv::Scalar(18, 18, 18));
        cv::Mat display_full(CANVAS_HEIGHT, CANVAS_WIDTH, CV_8UC3);
        cv::Mat controller_img(700, 520, CV_8UC3);

        // FPS Calculation timers (1 second window)
        auto last_fps_time = std::chrono::steady_clock::now();
        float fps_d435_left = 0.0f;
        float fps_d435_right = 0.0f;
        float fps_t265_left = 0.0f;
        float fps_t265_right = 0.0f;

        std::cout << "\n[OK] Visualizer running with Pop-Out Controller Window!\n";
        std::cout << "     Stream Window: 2560x1520 (Centered streams with black bars)\n";
        std::cout << "     Controller Window: 520x700\n";
        std::cout << "     Press 'r' or click 'RECORD 2s BENCHMARK' in controller.\n";
        std::cout << "     Press ESC or 'q' to exit.\n\n";

        // -----------------------------------------------------
        // 4. Main Rendering & Event Loop
        // -----------------------------------------------------
        while (g_running) {
            auto current_time = std::chrono::steady_clock::now();
            double elapsed_fps = std::chrono::duration<double>(current_time - last_fps_time).count();

            // Check if 2-second capture window has elapsed
            if (g_rec_state == REC_RECORDING) {
                double elapsed_rec = std::chrono::duration<double>(current_time - g_rec_start_time).count();
                if (elapsed_rec >= 2.0) {
                    g_rec_state = REC_SAVING;
                    std::thread(save_benchmark_footage, elapsed_rec).detach();
                }
            }

            // Update FPS readouts once every second
            if (elapsed_fps >= 1.0) {
                uint64_t d435_l = g_d435_left_cnt.exchange(0);
                uint64_t d435_r = g_d435_right_cnt.exchange(0);
                uint64_t t265_l = g_t265_left_cnt.exchange(0);
                uint64_t t265_r = g_t265_right_cnt.exchange(0);

                fps_d435_left  = d435_l / static_cast<float>(elapsed_fps);
                fps_d435_right = d435_r / static_cast<float>(elapsed_fps);
                fps_t265_left  = t265_l / static_cast<float>(elapsed_fps);
                fps_t265_right = t265_r / static_cast<float>(elapsed_fps);

                last_fps_time = current_time;
            }

            // --- Top: D435i Stream (2560 x 720 container, 2x 1280x720 slots) ---
            if (g_switching_d435.load()) {
                display_d435.setTo(cv::Scalar(22, 22, 30));
                char msg[128];
                snprintf(msg, sizeof(msg), "[ SWITCHING D435i TO %dx%d @ %d FPS... PLEASE WAIT ]",
                         g_d435_width.load(), g_d435_height.load(), g_target_d435_fps.load());
                int baseline = 0;
                cv::Size sz = cv::getTextSize(msg, cv::FONT_HERSHEY_SIMPLEX, 0.85, 2, &baseline);
                cv::putText(display_d435, msg, cv::Point((CANVAS_WIDTH - sz.width) / 2, D435_SLOT_HEIGHT / 2),
                            cv::FONT_HERSHEY_SIMPLEX, 0.85, cv::Scalar(0, 180, 255), 2, cv::LINE_AA);
            } else if (g_has_d435 && g_d435_enabled) {
                display_d435.setTo(cv::Scalar(10, 10, 10)); // Deep black letterbox background

                int in_w = g_d435_width.load();
                int in_h = g_d435_height.load();

                // Compute exact centering offsets within the 1280x720 slot
                int x_off = (D435_SLOT_WIDTH - in_w) / 2;
                int y_off = (D435_SLOT_HEIGHT - in_h) / 2;

                cv::Mat gray_d435_l, gray_d435_r;
                {
                    std::lock_guard<std::mutex> lock(g_d435_mutex);
                    gray_d435_l = g_d435_left_mat.clone();
                    gray_d435_r = g_d435_right_mat.clone();
                }

                // Left IR Camera (Slot: 0..1280)
                if (gray_d435_l.cols == in_w && gray_d435_l.rows == in_h) {
                    cv::Mat c_l;
                    cv::cvtColor(gray_d435_l, c_l, cv::COLOR_GRAY2BGR);
                    c_l.copyTo(display_d435(cv::Rect(0 + x_off, y_off, in_w, in_h)));
                }

                // Right IR Camera (Slot: 1280..2560)
                if (gray_d435_r.cols == in_w && gray_d435_r.rows == in_h) {
                    cv::Mat c_r;
                    cv::cvtColor(gray_d435_r, c_r, cv::COLOR_GRAY2BGR);
                    c_r.copyTo(display_d435(cv::Rect(D435_SLOT_WIDTH + x_off, y_off, in_w, in_h)));
                }

                // Vertical dividing line between Left and Right
                cv::line(display_d435, cv::Point(D435_SLOT_WIDTH, 0), cv::Point(D435_SLOT_WIDTH, D435_SLOT_HEIGHT), cv::Scalar(45, 45, 45), 1);
            } else {
                display_d435.setTo(cv::Scalar(20, 20, 22));
                std::string msg = g_has_d435 ? "[ D435i STREAM DISABLED - Toggle ON in Controller Window ]" 
                                             : "[ D435i NOT DETECTED ]";
                int baseline = 0;
                cv::Size sz = cv::getTextSize(msg, cv::FONT_HERSHEY_SIMPLEX, 0.85, 2, &baseline);
                cv::putText(display_d435, msg, cv::Point((CANVAS_WIDTH - sz.width) / 2, D435_SLOT_HEIGHT / 2), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.85, cv::Scalar(80, 80, 240), 2, cv::LINE_AA);
            }

            // D435i Bottom-Left Grey Badges per Camera
            if (g_has_d435) {
                int cur_w = g_d435_width.load();
                int cur_h = g_d435_height.load();
                int tgt_fps = g_target_d435_fps.load();

                // Column 1: Left IR
                char d435_l_text[128];
                snprintf(d435_l_text, sizeof(d435_l_text), "D435i | Left IR | R: %dx%d | FPS: %.1f (%d) | E: %s",
                         cur_w, cur_h, fps_d435_left, tgt_fps, g_emitter_enabled ? "ON" : "OFF");
                draw_bottom_left_badge(display_d435, d435_l_text, cv::Point(16, D435_SLOT_HEIGHT - 16));

                // Column 2: Right IR
                char d435_r_text[128];
                snprintf(d435_r_text, sizeof(d435_r_text), "D435i | Right IR | R: %dx%d | FPS: %.1f (%d) | E: %s",
                         cur_w, cur_h, fps_d435_right, tgt_fps, g_emitter_enabled ? "ON" : "OFF");
                draw_bottom_left_badge(display_d435, d435_r_text, cv::Point(D435_SLOT_WIDTH + 16, D435_SLOT_HEIGHT - 16));
            }

            // --- Bottom: T265 Stream (2560 x 800 container, 2x 1280x800 slots) ---
            if (g_switching_t265.load()) {
                display_t265.setTo(cv::Scalar(22, 22, 30));
                char msg[128];
                snprintf(msg, sizeof(msg), "[ RE-FLASHING T265 FIRMWARE TO %d FPS... PLEASE WAIT ]", g_target_t265_fps.load());
                int baseline = 0;
                cv::Size sz = cv::getTextSize(msg, cv::FONT_HERSHEY_SIMPLEX, 0.85, 2, &baseline);
                cv::putText(display_t265, msg, cv::Point((CANVAS_WIDTH - sz.width) / 2, T265_SLOT_HEIGHT / 2), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.85, cv::Scalar(0, 180, 255), 2, cv::LINE_AA);
            } else if (g_has_t265 && g_t265_enabled) {
                display_t265.setTo(cv::Scalar(10, 10, 10)); // Deep black letterbox background

                // T265 frame is 848x800, slot is 1280x800
                int t265_x_off = (T265_SLOT_WIDTH - 848) / 2; // 216 pixels black border left & right
                int t265_y_off = (T265_SLOT_HEIGHT - 800) / 2; // 0

                cv::Mat gray_t265_l, gray_t265_r;
                {
                    std::lock_guard<std::mutex> lock(g_t265_mutex);
                    gray_t265_l = g_t265_left_mat.clone();
                    gray_t265_r = g_t265_right_mat.clone();
                }

                // Left Fisheye Camera (Slot: 0..1280)
                if (gray_t265_l.cols == 848 && gray_t265_l.rows == 800) {
                    cv::Mat c_l;
                    cv::cvtColor(gray_t265_l, c_l, cv::COLOR_GRAY2BGR);
                    c_l.copyTo(display_t265(cv::Rect(0 + t265_x_off, t265_y_off, 848, 800)));
                }

                // Right Fisheye Camera (Slot: 1280..2560)
                if (gray_t265_r.cols == 848 && gray_t265_r.rows == 800) {
                    cv::Mat c_r;
                    cv::cvtColor(gray_t265_r, c_r, cv::COLOR_GRAY2BGR);
                    c_r.copyTo(display_t265(cv::Rect(T265_SLOT_WIDTH + t265_x_off, t265_y_off, 848, 800)));
                }

                // Vertical dividing line between Left and Right
                cv::line(display_t265, cv::Point(T265_SLOT_WIDTH, 0), cv::Point(T265_SLOT_WIDTH, T265_SLOT_HEIGHT), cv::Scalar(45, 45, 45), 1);
            } else {
                display_t265.setTo(cv::Scalar(22, 22, 25));
                std::string msg = g_has_t265 ? "[ T265 STREAM DISABLED - Toggle ON in Controller Window ]" 
                                             : "[ T265 NOT DETECTED ]";
                int baseline = 0;
                cv::Size sz = cv::getTextSize(msg, cv::FONT_HERSHEY_SIMPLEX, 0.85, 2, &baseline);
                cv::putText(display_t265, msg, cv::Point((CANVAS_WIDTH - sz.width) / 2, T265_SLOT_HEIGHT / 2), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.85, cv::Scalar(80, 80, 240), 2, cv::LINE_AA);
            }

            // T265 Bottom-Left Grey Badges per Camera
            if (g_has_t265) {
                // Column 1: Left Fisheye
                char t265_l_text[160];
                snprintf(t265_l_text, sizeof(t265_l_text), "T265 | Left Fisheye | R: 848x800 | FPS: %.1f (%d) | Exp: %.0f us | Gain: %.1fx | AE: OFF",
                         fps_t265_left, g_target_t265_fps.load(), g_exposure_f.load(), g_gain_f.load());
                draw_bottom_left_badge(display_t265, t265_l_text, cv::Point(16, T265_SLOT_HEIGHT - 16));

                // Column 2: Right Fisheye
                char t265_r_text[160];
                snprintf(t265_r_text, sizeof(t265_r_text), "T265 | Right Fisheye | R: 848x800 | FPS: %.1f (%d) | Exp: %.0f us | Gain: %.1fx | AE: OFF",
                         fps_t265_right, g_target_t265_fps.load(), g_exposure_f.load(), g_gain_f.load());
                draw_bottom_left_badge(display_t265, t265_r_text, cv::Point(T265_SLOT_WIDTH + 16, T265_SLOT_HEIGHT - 16));
            }

            // Combine vertically into 2560 x 1520 stream canvas
            display_d435.copyTo(display_full(cv::Rect(0, 0, CANVAS_WIDTH, D435_SLOT_HEIGHT)));
            display_t265.copyTo(display_full(cv::Rect(0, D435_SLOT_HEIGHT, CANVAS_WIDTH, T265_SLOT_HEIGHT)));

            // Horizontal dividing line between D435i and T265
            cv::line(display_full, cv::Point(0, D435_SLOT_HEIGHT - 1), cv::Point(CANVAS_WIDTH, D435_SLOT_HEIGHT - 1), cv::Scalar(0, 180, 255), 2);

            // Vertical full dividing line
            cv::line(display_full, cv::Point(CANVAS_WIDTH / 2, 0), cv::Point(CANVAS_WIDTH / 2, CANVAS_HEIGHT), cv::Scalar(50, 50, 50), 1);

            // Notification Banner on Stream Window (centered at width = 2560)
            {
                std::lock_guard<std::mutex> lock(g_banner_mutex);
                if (!g_banner_text.empty()) {
                    double banner_sec = std::chrono::duration<double>(current_time - g_banner_time).count();
                    if (banner_sec < 6.0) {
                        int baseline = 0;
                        cv::Size sz = cv::getTextSize(g_banner_text, cv::FONT_HERSHEY_SIMPLEX, 0.55, 1, &baseline);
                        cv::Rect b_rect(CANVAS_WIDTH / 2 - sz.width / 2 - 20, 14, sz.width + 40, 32);
                        cv::rectangle(display_full, b_rect, cv::Scalar(20, 60, 20), cv::FILLED);
                        cv::rectangle(display_full, b_rect, cv::Scalar(40, 180, 40), 1);
                        cv::putText(display_full, g_banner_text, cv::Point(b_rect.x + 20, b_rect.y + 22),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(100, 255, 100), 1, cv::LINE_AA);
                    }
                }
            }

            // Render both windows
            cv::imshow(STREAM_WINDOW, display_full);

            render_controller_window(controller_img);
            cv::imshow(CONTROLLER_WINDOW, controller_img);

            // User keyboard input handling (works in either window)
            int key = cv::waitKey(1);
            if (key == 27 || key == 'q' || key == 'Q') {
                g_running = false;
                break;
            } else if (key == '1') {
                g_d435_enabled = !g_d435_enabled;
                std::cout << "[HOTKEY] D435i Stream: " << (g_d435_enabled ? "ON" : "OFF") << std::endl;
            } else if (key == '2') {
                g_t265_enabled = !g_t265_enabled;
                std::cout << "[HOTKEY] T265 Stream: " << (g_t265_enabled ? "ON" : "OFF") << std::endl;
            } else if (key == '3') {
                switch_t265_fps(30);
            } else if (key == '6') {
                switch_t265_fps(60);
            } else if (key == '9') {
                switch_t265_fps(90);
            } else if (key == 'e' || key == 'E') {
                if (g_has_d435 && g_d435_stereo_sensor) {
                    g_emitter_enabled = !g_emitter_enabled;
                    try {
                        g_d435_stereo_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, g_emitter_enabled ? 1.0f : 0.0f);
                        std::cout << "[HOTKEY] D435i Emitter: " << (g_emitter_enabled ? "ON" : "OFF") << std::endl;
                    } catch (const std::exception& e) {
                        std::cerr << "[D435i] Error setting emitter: " << e.what() << "\n";
                    }
                }
            } else if (key == 'r' || key == 'R' || key == 32) { // 'r', 'R' or Spacebar
                trigger_record_2s();
            }
        }

        // -----------------------------------------------------
        // 5. Clean Resource Teardown
        // -----------------------------------------------------
        std::cout << "\nShutting down camera pipelines...\n";
        if (g_d435_pipe) {
            try { g_d435_pipe->stop(); } catch (...) {}
            std::cout << "[D435i] Pipeline stopped.\n";
        }
        if (g_has_t265 && g_t265_sensor) {
            try { g_t265_sensor.stop(); } catch (...) {}
            try { g_t265_sensor.close(); } catch (...) {}
            std::cout << "[T265] Sensor stopped and closed.\n";
        }
        cv::destroyAllWindows();
        std::cout << "[SUCCESS] Visualizer exited cleanly.\n";
        std::quick_exit(0);

    } catch (const std::exception& e) {
        std::cerr << "[FATAL EXCEPTION] " << e.what() << "\n";
        return 1;
    }

    return 0;
}
