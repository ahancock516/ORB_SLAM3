/**
 * @file mono_live_pi.cc
 * @brief ORB-SLAM3 monocular using live camera on Raspberry Pi.
 *
 * - Opens camera with OpenCV (V4L2 by default on Linux/RPi).
 * - Captures frames, timestamps with steady_clock.
 * - Optional grayscale conversion if settings require it.
 * - Rescales by SLAM.GetImageScale() from YAML.
 * - Feeds frames to ORB_SLAM3::System::TrackMonocular().
 * - Clean shutdown and keyframe trajectory save.
 *
 * Usage:
 *   ./mono_live_pi /path/to/ORBvoc.txt /path/to/Settings.yaml [--gray] [--gstreamer]
 *
 * Notes:
 * - If your YAML uses Camera.RGB: 0, either pass --gray or set it in code.
 * - If your Pi camera is ONLY accessible via libcamera, use --gstreamer.
 */

#include <iostream>
#include <chrono>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

#include <System.h>

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0]
                  << " path_to_vocabulary path_to_settings [--gray] [--gstreamer]\n";
        return 1;
    }

    const std::string voc_path = argv[1];
    const std::string settings_path = argv[2];

    bool forceGray = false;
    bool useGStreamer = false;
    for (int i = 3; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--gray") forceGray = true;
        else if (arg == "--gstreamer") useGStreamer = true;
    }

    cv::VideoCapture cap;
    if (useGStreamer) {
        // For libcamera-only setups (Bookworm): adjust width/height/fps as needed
        // Pi Camera (libcamera) → GStreamer → OpenCV
        // Format notes: BGR conversion happens in appsink
        std::string pipeline =
            "libcamerasrc ! video/x-raw,format=RGB,width=640,height=480,framerate=30/1 "
            "! videoconvert ! video/x-raw,format=BGR ! appsink drop=1";
        if (!cap.open(pipeline, cv::CAP_GSTREAMER)) {
            std::cerr << "Error: could not open GStreamer pipeline.\n";
            return 1;
        }
    } else {
        // V4L2 path (USB cam or Pi cam if /dev/video0 is available)
        if (!cap.open(0, cv::CAP_V4L2)) {
            std::cerr << "Error: could not open camera with V4L2. "
                         "If using libcamera-only, retry with --gstreamer.\n";
            return 1;
        }
        // Ask for a common mode (camera may choose nearest)
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        cap.set(cv::CAP_PROP_FPS, 30);
    }

    // Create SLAM (viewer on = true; turn off on headless if needed)
    ORB_SLAM3::System SLAM(voc_path, settings_path, ORB_SLAM3::System::MONOCULAR, true);
    const float imageScale = SLAM.GetImageScale();

    std::cout << "\n-------\nStart live monocular on Raspberry Pi...\n"
              << "Press 'q' or ESC to quit (window must be shown).\n";

    const auto t0 = std::chrono::steady_clock::now();

    cv::Mat frame, proc;
    while (true)
    {
        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "Error: failed to grab frame. Exiting.\n";
            break;
        }

        // Timestamp (seconds since start) using monotonic clock
        const double tframe =
            std::chrono::duration_cast<std::chrono::duration<double>>(
                std::chrono::steady_clock::now() - t0).count();

        // Optional grayscale to match YAML (Camera.RGB: 0)
        if (forceGray) {
            cv::cvtColor(frame, proc, cv::COLOR_BGR2GRAY);
        } else {
            proc = frame;
        }

        // Optional scaling from settings
        if (imageScale != 1.0f) {
            const int w = static_cast<int>(std::lround(proc.cols * imageScale));
            const int h = static_cast<int>(std::lround(proc.rows * imageScale));
            if (w > 0 && h > 0) cv::resize(proc, proc, cv::Size(w, h));
        }

        // Feed to ORB-SLAM3
        SLAM.TrackMonocular(proc, tframe);

        // (Optional) show preview so you can press 'q'
        cv::imshow("ORB-SLAM3 Live (Pi)", frame);
        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) break;
    }

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    return 0;
}