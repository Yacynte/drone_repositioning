#include "ImageMatcher.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>
#include "MetadataClient.h"
#include "RtspReader.h"
#include "AlgoLogger.hpp"
#include "Utils.hpp"
#include <cmath>
#include <sstream>
#include <iomanip>
#include <string>

int imgWidth = 640;
int imgHeight = 480;
bool unrealMode = true;
double error_ang = 1.0;
double threshold_error = 5e-2;;
// double error_dist = 1.0;
// double dist_threshold = 1e-3;
double dist_threshold = 0.1 * cv::norm(cv::Point2f(imgHeight, imgWidth));
double error_dist = imgWidth; // initial large displacement
using Clock = std::chrono::steady_clock;

// Convert a double to a string where the decimal point is replaced with '_'.
// Example: 11829.695398 -> "11829_695398"
static std::string timeToUnderscoreString(int precision = 6) {
    const auto t_now = Clock::now().time_since_epoch();
    double value = std::chrono::duration<double>(t_now).count();
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    std::string s = oss.str();
    // Trim trailing zeros after the decimal point
    auto pos = s.find('.');
    if (pos != std::string::npos) {
        while (!s.empty() && s.back() == '0') s.pop_back();
        if (!s.empty() && s.back() == '.') s.pop_back();
    }
    std::replace(s.begin(), s.end(), '.', '_');
    return s;
}
int main(int argc, char** argv) {
    
    std::string logFile = "AlgoLog_" + timeToUnderscoreString() + ".csv";
    AlgoLogger logger(logFile, /*write_header=*/true, /*flush_every_n=*/30);
    // Choose input mode: "live" (default) or "stream <url>"
    MetadataTcpClient client;
    
    // 1. Connect (Uses default 127.0.0.1:9001)
    if (!client.Connect()) {
        std::cerr << "Failed to connect to server. Ensure the server is running on port 9001." << std::endl;
        return 0;
    }
    else {
        std::cout << "Successfully connected to metadata server." << std::endl;
    }
    std::string mode = "live";
    std::string stream_url;
    int camera_index = 0;
    if (argc >= 2) mode = argv[1];
    if (mode == "stream") {
        if (argc >= 3) {
            stream_url = argv[2];
            std::cout << "Using stream URL: " << stream_url << std::endl;
        }
        else if (argc == 2)
        {
           stream_url = "rtsp://10.116.88.38:8554/mystream";
           std::cout << "Using default stream: " << stream_url << std::endl;
        }
        
        else {
            // stream_url = "rstp://172.28.240.1/mysream";
            std::cout << "Using default stream: " << stream_url << std::endl;
            std::cout << "Usage: " << argv[0] << " [live|stream] [stream_url]\n";
            return -1;
        }
    }
    else if (mode == "live") {
        if (argc >= 3) {
            // Safe conversion from argv[2] to int
            try {
                std::cout << "Using live mode with camera index: " << argv[2] << std::endl;
                camera_index = std::stoi(argv[2]);
            } catch (const std::exception &e) {
                std::cerr << "Invalid camera index '" << argv[2] << "': " << e.what()
                          << ". Defaulting to 0.\n";
                camera_index = 0;
            }
        }
    }

    // Load target image
    ImageMatcher matcher("../target.png");
    // Speed mapping (smooth saturating)
    // Smoothing factors
    int increment = 0;
    std::string data_to_send = "";
    float k = 0.02;              // tune
    float k_yaw = 0.02;
    float k_pitch = 0.02;
    float tau = 0.25f;
    double last_frame = 0;
    double time_now = 0 ;
    double old_img_ts = 0;

    float v_max_ = 20.0;           // cm/s or drone units
    float yaw_rate_max = 100;             // degrees/s
    float pitch_rate_max = 2;
    cv::Point3f direction = cv::Point3f(0,0,0);
    cv::Point3f last_cmd_vx = cv::Point3f(0,0,0);
    cv::Point3f cmd_vx = cv::Point3f(0,0,0);
    cv::Point3f last_angle_rate = cv::Point3f(0,0,0);
    cv::Point3f angle_rate_cmd = cv::Point3f(0,0,0);


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
        std::cout << "Stream / Camera can be opened" << std::endl;
        // prefer MJPEG/FourCC if stream provides it
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, imgWidth);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, imgHeight);
    if (!cap.isOpened()) {
        std::cerr << "Error: Cannot open camera/stream" << std::endl;
        return -1;
    }
    std::cout << " Camera/Stream opened" << std::endl;
    float diag = std::sqrt(imgHeight*imgHeight + imgWidth*imgWidth);
    // cv::Mat frame;
    RtspReader reader(stream_url,imgWidth, imgHeight);
    reader.start();
    cv::Mat frame;
    while (true) {
        // cap >> frame;
        if (!reader.getFrame(frame)) {
            std::cout << "No available frame" << std::endl;
            continue;}
        // When image arrives:
        double img_ts = AlgoLogger::nowSec();
        // std::cout << "Processing frame for alignment..." << std::endl;
        // Get alignment direction (x, y, z)
        // auto displacement = matcher.getAlignmentDisplacement(frame);
        cv::Point3f displacement;
        // auto displacement = matcher.getAlignmentDisplacementRansac(frame);
        // std::cout << " Displacement (pixels): " << displacement << std::endl;
        // std::cout << "Processing frame for Rotation..." << std::endl;
        auto [rotationMatrix, direction1]  = matcher.getAlignmentDirection(displacement, frame);

        // cv::Point3f direction = cv::Point3f(0,0,0);
        cv::Mat I = cv::Mat::eye(3, 3, rotationMatrix.type());
        cv::Mat diff = rotationMatrix - I;
        cv::Mat world_rotation_vec;
        cv::Rodrigues(rotationMatrix, world_rotation_vec);
        world_rotation_vec.convertTo(world_rotation_vec, CV_64F);
        YPRDeg rotation_vec = rotmatToYPRDeg_ZYX(rotationMatrix);

        cv::Mat axis = world_rotation_vec / cv::norm(world_rotation_vec);
        float angle = cv::norm(world_rotation_vec) * (180.0f / CV_PI); // degrees
        cv::Point3f rotation, translation;

        // usage
        double yawErr   = wrapDeg(rotation_vec.yaw);
        double pitchErr = wrapDeg(rotation_vec.pitch);
        double rollErr  = wrapDeg(rotation_vec.roll);
        
        // float yaw_rate_max = yaw_rate_max_ * (1.f - std::exp(- k_yaw * angle));
        float v_max = v_max_ * (1.f - std::exp(- k * cv::norm(displacement)));
        std::cout << "Max yaw rate: " << yaw_rate_max << ", Angle: " << angle << std::endl;
        double yawRateCmd   = tanhRate(yawErr,   yaw_rate_max, k_yaw);  // deg/s
        double pitchRateCmd = tanhRate(pitchErr, yaw_rate_max, k_yaw);
        double rollRateCmd  = tanhRate(rollErr,  yaw_rate_max, k_yaw);

        if (unrealMode){
            // rotation.x = axis.at<double>(2) * angle;
            // rotation.y = axis.at<double>(0) * angle;
            // rotation.z = -axis.at<double>(1) * angle;
            angle_rate_cmd.x = yawRateCmd;
            angle_rate_cmd.y = rollRateCmd;
            angle_rate_cmd.z = -pitchRateCmd;

            translation.x = direction1.z; 
            translation.y = direction1.x;
            translation.z = -direction1.y;
        }
        else{
            rotation.x = axis.at<double>(0);
            rotation.y = axis.at<double>(1);
            rotation.z = axis.at<double>(2);
            translation = displacement;
        }
        
        cmd_vx.x = v_max * std::tanh(k * translation.x);
        cmd_vx.y = v_max * std::tanh(k * translation.y);
        cmd_vx.z = v_max * std::tanh(k * translation.z);

        // yaw/pitch rate from angular error (separate gains!)
        // angle_rate_cmd.x = yaw_rate_max * std::tanh(k_yaw   * rotation.x);
        // angle_rate_cmd.y = pitch_rate_max * std::tanh(k_pitch * rotation.y);

        // if (old_img_ts != 0) dt = img_ts - old_img_ts;
        // dt-based smoothing
        // float alpha = 1.f - std::exp(- k_yaw * angle);
        float alpha = 0.3;

        cmd_vx = alpha * cmd_vx + (1 - alpha) * last_cmd_vx;
        angle_rate_cmd = alpha * angle_rate_cmd + (1 - alpha) * last_angle_rate;

        // std::cout << " rotation_x: " << angle_rate_cmd.x << " rotation_y: " << angle_rate_cmd.y << std::endl;
        // std::cout << " direction_x: " << direction.x << " direction_y: " << direction.y<< std::endl;
        // std::cout << " cmd_vx_x: " << cmd_vx.x << " cmd_vx_y: " << cmd_vx.y << " cmd_vx_z: " << cmd_vx.z << std::endl;
        // std::cout << "Displacement: " << translation.x << translation.y << translation.z << std::endl;
        std::cout << "Displacement: " << displacement.x << ", " << displacement.y << ", " << displacement.z << std::endl;
        std::cout << " Error: " << error_ang << " Error dist: " << error_dist << " YawErr: " << yawErr << " PitchErr: " << pitchErr << " RollErr: " << rollErr << std::endl;
        std::stringstream ss;
        // YawRate_cam, PitchRate_cam, YawRate_drone, drone_x, drone_y, drone_z \n 
        // ss << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "\n";
        // std::string data_0 = ss.str();  
        error_ang = cv::norm(diff);
        error_dist = cv::norm(displacement);
        double send_ts = 0;
        // if (error_dist > dist_threshold){
        //     ss << 0 << "," << 0 << "," << 0 << "," << cmd_vx.x << "," << cmd_vx.y << "," << cmd_vx.z << "," << "0" << "\n";
        //     data_to_send = ss.str();
        // }
        // else if (error_ang > threshold_error){
        //     ss << angle_rate_cmd.x << "," << angle_rate_cmd.y << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << "0" << "\n";
        //     data_to_send = ss.str();
        // }
        ss << angle_rate_cmd.x << "," << angle_rate_cmd.y << "," << angle_rate_cmd.z << "," << 0 << "," << 0 << "," << 0 << "," << "0" << "\n";
        data_to_send = ss.str();
        if (data_to_send != "" ){
            if (!client.SendMetadata(data_to_send)) 
            {
                std::cerr << "Failed to send data. Check connection." << std::endl;
                break;
            }
            send_ts = AlgoLogger::nowSec();
            std::cout << "Data sent: " << data_to_send << std::endl;
            // increment = 0;
            last_frame = img_ts;
            data_to_send = "";
        }  
        else { time_now = AlgoLogger::nowSec();}
        // optional parse (so you can graph cmd5 easily)
        auto parsed = AlgoLogger::parseCommand6(data_to_send);
        logger.log(img_ts, send_ts, angle_rate_cmd, cmd_vx, data_to_send, parsed);
        if( time_now - last_frame >= 1) {
            std::cout << "Arrived at destination" << std::endl;
            break;
        }
        last_cmd_vx = cmd_vx;
        last_angle_rate = angle_rate_cmd;
        // old_img_ts = img_ts;
        // std::cout << "Metadata sent: rotation_z=" << rotation.z << ", rotation_y=" << rotation.y << std::endl;
        // // Choose color based on z motion
        // cv::Scalar color = (direction.z < 0) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);

        // // Draw arrow for x/y direction
        // cv::Point center(frame.cols / 2, frame.rows / 2);
        // cv::Point tip(
        //     static_cast<int>(center.x + displacement*direction.x ), // scale factor for visibility
        //     static_cast<int>(center.y + displacement*direction.y )
        // );

        // cv::arrowedLine(frame, center, tip, color, 2, cv::LINE_AA, 0, 0.3);

        // // Display info text
        // std::string text = "dx=" + std::to_string(cmd_vx.x) +
        //                    " dy=" + std::to_string(cmd_vx.y) +
        //                    " dz=" + std::to_string(cmd_vx.z);
        // cv::putText(frame, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,255,255), 2);

        
        // // Show live feed
        // cv::imshow("Live Camera Alignment", frame);
        // if (cv::waitKey(1) == 'q') break;

        // // std::cout << "Show Image. "<< std::endl;
        // // std::cout<< "Direction: " << direction << std::endl;

    }

    cap.release();
    // cv::destroyAllWindows();
    return 0;
}
