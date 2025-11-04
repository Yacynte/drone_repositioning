#include "ImageMatcher.h"
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Load target image
    ImageMatcher matcher("../target.jpg");
    // Speed mapping (smooth saturating)
    float k = 0.02;              // tune
    float v_max = 1.0;           // m/s or drone units
    float v_min = 0.05;
    cv::Point3f last_cmd_vx = cv::Point3f(0,0,0);
    cv::Point3f cmd_vx = cv::Point3f(0,0,0);

    // Open default camera
    // cv::VideoCapture cap(0, cv::CAP_V4L2);
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    if (!cap.isOpened()) {
        std::cerr << "Error: Cannot open camera" << std::endl;
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
