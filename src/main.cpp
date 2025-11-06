#include "ImageMatcher.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>

int main(int argc, char** argv) {
    // Choose input mode: "live" (default) or "stream <url>"
    std::string mode = "live";
    std::string stream_url;
    int camera_index = 0;
    if (argc >= 2) mode = argv[1];
    if (mode == "stream") {
        if (argc >= 3) {
            stream_url = argv[2];
        } else {
            stream_url = "http://10.116.88.38:80";
            std::cout << "Using default stream: " << stream_url << std::endl;
            std::cout << "Usage: " << argv[0] << " [live|stream] [stream_url]\n";
            return -1;
        }
    }
    else if (mode == "live") {
        if (argc >= 3) {
            // Safe conversion from argv[2] to int
            try {
                camera_index = std::stoi(argv[2]);
            } catch (const std::exception &e) {
                std::cerr << "Invalid camera index '" << argv[2] << "': " << e.what()
                          << ". Defaulting to 0.\n";
                camera_index = 0;
            }
        }
    }

    // Load target image
    ImageMatcher matcher("../target.jpg");
    // Speed mapping (smooth saturating)
    float k = 0.02;              // tune
    float v_max = 1.0;           // m/s or drone units
    float v_min = 0.05;
    cv::Point3f last_cmd_vx = cv::Point3f(0,0,0);
    cv::Point3f cmd_vx = cv::Point3f(0,0,0);

    cv::VideoCapture cap;

    if (mode == "live") {
        // Open local camera
        if (!cap.open(camera_index, cv::CAP_V4L2)) {
            std::cerr << "Warning: cv::CAP_V4L2 failed, trying default backend..." << std::endl;
            if (!cap.open(0)) {
                std::cerr << "Error: Cannot open local camera" << std::endl;
                return -1;
            }
        }
    } else {
        // Open IP stream (RTSP / HTTP MJPEG). Example usage:
        //   ./app stream "rtsp://user:pass@192.168.1.100:554/stream"
        //   ./app stream "http://192.168.1.100:8080/video"
        if (!cap.open(stream_url, cv::CAP_FFMPEG)) {
            std::cerr << "Warning: cv::CAP_FFMPEG failed, trying default backend..." << std::endl;
            if (!cap.open(stream_url)) {
                std::cerr << "Error: Cannot open stream: " << stream_url << std::endl;
                return -1;
            }
        }
        // prefer MJPEG/FourCC if stream provides it
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    if (!cap.isOpened()) {
        std::cerr << "Error: Cannot open camera/stream" << std::endl;
        return -1;
    }


    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // Get alignment direction (x, y, z)
        double displacement = matcher.getAlignmentDisplacement(frame);

        cv::Point3f direction = matcher.getAlignmentDirection();
        // cv::Point3f direction = cv::Point3f(0,0,0);

        // Speed mapping (smooth saturating)
        float v = v_max * (1.0 - std::exp(-k * displacement));
        v = clamp(v, v_min, v_max);

        // Compose command
        cmd_vx.x = v * direction.x;
        cmd_vx.y = v * direction.y;
        cmd_vx.z = v * direction.z;

        // Smooth commands
        float alpha_cmd = 0.3;
        cmd_vx.x = alpha_cmd * cmd_vx.x + (1.0 - alpha_cmd) * last_cmd_vx.x;
        cmd_vx.y = alpha_cmd * cmd_vx.y + (1.0 - alpha_cmd) * last_cmd_vx.y;
        cmd_vx.z = alpha_cmd * cmd_vx.z + (1.0 - alpha_cmd) * last_cmd_vx.z;


        // Choose color based on z motion
        cv::Scalar color = (direction.z < 0) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);

        // Draw arrow for x/y direction
        cv::Point center(frame.cols / 2, frame.rows / 2);
        cv::Point tip(
            static_cast<int>(center.x + displacement*direction.x ), // scale factor for visibility
            static_cast<int>(center.y + displacement*direction.y )
        );

        cv::arrowedLine(frame, center, tip, color, 2, cv::LINE_AA, 0, 0.3);

        // Display info text
        std::string text = "dx=" + std::to_string(cmd_vx.x) +
                           " dy=" + std::to_string(cmd_vx.y) +
                           " dz=" + std::to_string(cmd_vx.z);
        cv::putText(frame, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,255,255), 2);

        // Show live feed
        cv::imshow("Live Camera Alignment", frame);
        if (cv::waitKey(1) == 'q') break;

        // std::cout << "Show Image. "<< std::endl;
        // std::cout<< "Direction: " << direction << std::endl;

    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
