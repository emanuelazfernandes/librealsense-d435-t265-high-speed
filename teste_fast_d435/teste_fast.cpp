// Compile with:
// g++ teste_fast.cpp -o teste_fast -std=c++17 $(pkg-config --cflags --libs opencv4) -lrealsense2
//
// Run with:
// ./teste_fast

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    try {
        rs2::config cfg;

        cfg.enable_stream(RS2_STREAM_INFRARED, 1, 848, 100, RS2_FORMAT_Y8, 300);
        cfg.enable_stream(RS2_STREAM_INFRARED, 2, 848, 100, RS2_FORMAT_Y8, 300);

        rs2::pipeline pipe;
        rs2::pipeline_profile profile = pipe.start(cfg);
        rs2::device dev = profile.get_device();
        
        // Find the sensor that supports Emitter
        rs2::sensor emitter_sensor;
        bool emitter_found = false;
        for (rs2::sensor& s : dev.query_sensors()) {
            if (s.supports(RS2_OPTION_EMITTER_ENABLED)) {
                emitter_sensor = s;
                emitter_found = true;
                break;
            }
        }

        bool emitter_enabled = false;
        if (emitter_found) emitter_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);

        std::cout << "Streaming side-by-side (1696x100). Press 'e' to toggle emitter, ESC to exit." << std::endl;

        while (true) {
            rs2::frameset frames = pipe.wait_for_frames();
            rs2::video_frame ir_left = frames.get_infrared_frame(1);
            rs2::video_frame ir_right = frames.get_infrared_frame(2);

            if (!ir_left || !ir_right) continue;

            cv::Mat cv_left(
                cv::Size(ir_left.get_width(), ir_left.get_height()), 
                CV_8UC1,
                (void*) ir_left.get_data(),
                cv::Mat::AUTO_STEP
            );
            cv::Mat cv_right(
                cv::Size(ir_right.get_width(), ir_right.get_height()), 
                CV_8UC1,
                (void*) ir_right.get_data(),
                cv::Mat::AUTO_STEP
            );

            // Combine side-by-side
            cv::Mat combined;
            cv::hconcat(cv_left, cv_right, combined); 

            cv::imshow("Dual IR Stream", combined);

            // Handle user input
            int key = cv::waitKey(1);
            if (key == 27) break; // ESC
            if (key == 'e' || key == 'E') {
                if (emitter_found) {
                    emitter_enabled = !emitter_enabled;
                    emitter_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, emitter_enabled ? 1.f : 0.f);
                    std::cout << "Emitter: " << (emitter_enabled ? "ON" : "OFF") << std::endl;
                }
            }
        }

    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;

}
