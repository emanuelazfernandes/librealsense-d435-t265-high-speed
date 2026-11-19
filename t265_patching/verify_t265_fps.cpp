#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

int main(int argc, char* argv[]) {
    int target_fps = 90;
    if (argc > 1) {
        target_fps = std::stoi(argv[1]);
    }
    std::cout << "Testing T265 streaming at target FPS: " << target_fps << "\n";

    try {
        rs2::context ctx;
        rs2::device t265_dev;

        for (auto&& d : ctx.query_devices()) {
            if (std::string(d.get_info(RS2_CAMERA_INFO_NAME)).find("T265") != std::string::npos) {
                t265_dev = d;
                break;
            }
        }

        if (!t265_dev) {
            std::cerr << "Error: T265 device not found.\n";
            return 1;
        }

        std::cout << "Found T265: " << t265_dev.get_info(RS2_CAMERA_INFO_NAME)
                  << " (S/N: " << t265_dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << ")\n";

        rs2::sensor sensor = t265_dev.query_sensors()[0];

        // Disable auto-exposure and set manual exposure
        if (sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
            sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.0f);
            std::cout << "Disabled auto-exposure.\n";
        }
        if (sensor.supports(RS2_OPTION_EXPOSURE)) {
            sensor.set_option(RS2_OPTION_EXPOSURE, 2000.0f); // 2000 us
            std::cout << "Set manual exposure to 2000 us.\n";
        }
        if (sensor.supports(RS2_OPTION_GAIN)) {
            sensor.set_option(RS2_OPTION_GAIN, 2.5f);
            std::cout << "Set gain to 2.5x.\n";
        }

        std::vector<rs2::stream_profile> profiles_to_open;
        for (auto&& p : sensor.get_stream_profiles()) {
            if (auto vp = p.as<rs2::video_stream_profile>()) {
                if (vp.stream_type() == RS2_STREAM_FISHEYE && vp.fps() == target_fps) {
                    profiles_to_open.push_back(p);
                    std::cout << "Selected profile: Fisheye " << vp.stream_index()
                              << " " << vp.width() << "x" << vp.height()
                              << " @ " << vp.fps() << "Hz (" << vp.format() << ")\n";
                }
            }
        }

        if (profiles_to_open.empty()) {
            std::cerr << "Error: No fisheye profile found with FPS=" << target_fps << "\n";
            return 1;
        }

        std::atomic<uint64_t> f1_count{0};
        std::atomic<uint64_t> f2_count{0};
        uint64_t last_f1_num = 0;
        uint64_t last_f2_num = 0;

        std::cout << "Opening sensor with " << profiles_to_open.size() << " profiles...\n";
        sensor.open(profiles_to_open);

        sensor.start([&](const rs2::frame& f) {
            int idx = f.get_profile().stream_index();
            uint64_t num = f.get_frame_number();
            if (idx == 1) {
                if (num != last_f1_num) {
                    last_f1_num = num;
                    f1_count++;
                }
            } else if (idx == 2) {
                if (num != last_f2_num) {
                    last_f2_num = num;
                    f2_count++;
                }
            }
        });

        std::cout << "Sensor started! Measuring exact frame arrival rate for 10 seconds...\n\n";
        std::cout << "  Second | Fisheye 1 FPS | Fisheye 2 FPS\n";
        std::cout << "---------+---------------+---------------\n";
        for (int i = 1; i <= 10; ++i) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            uint64_t c1 = f1_count.exchange(0);
            uint64_t c2 = f2_count.exchange(0);
            std::cout << "   " << std::setw(2) << i << "s   |    " << std::setw(3) << c1 << " FPS    |    " << std::setw(3) << c2 << " FPS\n";
        }

        sensor.stop();
        sensor.close();
        std::cout << "---------+---------------+---------------\n";
        std::cout << "Test completed successfully.\n";

    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
