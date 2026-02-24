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
#include <numeric>

int imgWidth = 1920;
int imgHeight = 1080;
bool unrealMode = true;
double error_ang = 1.0;
double threshold_error = 10e-2;;
double error_dist = 1.0;
// double dist_threshold = 30; // cm
// double dist_threshold = 5; //px
bool complete = false;
double dist_threshold = 0.15 * cv::norm(cv::Point2f(imgHeight, imgWidth));
// double error_dist = imgWidth; // initial large displacement


int main(int argc, char** argv) {

    auto flags = parseFlags(argc, argv);

    std::string mode      = getStr(flags, "--mode", "stream");  // default stream
    std::string stream_url      = getStr(flags, "--rtsp", "rtsp://10.116.88.38:8554/mystream");
    int camera_index      = getInt(flags, "--camera", 0);

    std::string cmd_ip    = getStr(flags, "--cmd_ip", "0.0.0.0");
    int cmd_port          = getInt(flags, "--cmd_port", 9020);

    std::string msg_ip    = getStr(flags, "--relay_ip", "0.0.0.0");
    int msg_port          = getInt(flags, "--relay_port", 9010);

    std::string log_path    = getStr(flags, "--log", "../logs/");
    std::string target_image_path = getStr(flags, "--target", "../target.png");

    std::cout << "[mode] " << mode << "\n";
    std::cout << "[net] cmd=" << cmd_ip << ":" << cmd_port
              << " msg=" << msg_ip << ":" << msg_port << "\n";

    
    std::string logFile = log_path + "AlgoLog_" + timeToUnderscoreString() + ".csv";
    AlgoLogger logger(logFile, /*write_header=*/true, /*flush_every_n=*/30);
    // Choose input mode: "live" (default) or "stream <url>"
    MetadataTcpClient client;

    if (client.StartConnectionHandler(cmd_ip, cmd_port, "command_handler")) client.startReceiver();
    else {
        std::cerr << "Failed to start metadata server connection handler. Ensure no other instance is running and port 9020 is available." << std::endl;
        return -1;
    }
    
    client.StartConnectionHandler(msg_ip, msg_port, "metadata_server");
    // 1. Connect (Uses default 127.0.0.1:9001)
    // while (!client.Connect(msg_ip, msg_port)) {
    //     std::cout << "Waiting for metadata server to be available on port " << msg_port << std::endl;
    //     usleep(1000 * 1000); // Sleep for 1 second before retrying
    // }
    std::cout << "Connected to metadata server" << std::endl;
    // std::string mode = "live";
    // std::string stream_url;
    // int camera_index = 0;

    // Load target image
    ImageMatcher matcher(target_image_path);
    // Speed mapping (smooth saturating)
    // Smoothing factors
    int increment = 0;
    std::string data_to_send = "";
    float k = 0.05;              // tune
    float k_yaw = 0.02;
    float k_pitch = 0.02;
    float tau = 0.25f;
    double last_frame_init = AlgoLogger::nowWallSec() + 86400;
    double last_frame = last_frame_init;
    double time_now = 0 ;
    double old_img_ts = 0;

    float v_max = 10.0;           // cm/s or drone units
    float w_max = 10.0;             // degrees/s
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
    } 
    else if (mode == "stream") {
        // Open IP stream (RTSP / HTTP MJPEG). Example usage:
        //   ./app stream "rtsp://user:pass@192.168.1.100:554/stream"
        //   ./app stream "http://192.168.1.100:8080/video"
        while (!cap.open(stream_url)) {
            std::cout << "Waiting for stream to be available: " << stream_url << std::endl;
            usleep(1000 * 1000); // Sleep for 1 second before retrying
            // if (!cap.open(stream_url, cv::CAP_FFMPEG)) {
            //     std::cerr << "Error: Cannot open stream: " << stream_url << std::endl;
            //     return -1;
            // }
        }
        std::cout << "Stream / Camera can be opened" << std::endl;
        // prefer MJPEG/FourCC if stream provides it
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    }
    else {
        std::cerr << "Unknown mode: " << mode << ". Use 'live' or 'stream <url>'." << std::endl;
        return -1;
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
    std::vector<int> direction_history;
    int switching = 0;
    while (true) {
        // std::cout << "Status of Start/Stop flags - Start: " << client.startRepositioning.load() << ", Stop: " << client.stopRepositioning.load() << std::endl;
        if (!client.startRepositioning.load()) {
            // std::cout << "Received command to start the Repositioning System" << std::endl;
            std::cout << "Waiting to start the Repositioning System" << std::endl;
            usleep(1000 * 1000); // Sleep for 1 second before checking again
            continue;
        }
        if (client.pauseRepositioning.load()) {
            std::cout << "Received command to pause repositioning" << std::endl;
            while (!client.resumeRepositioning.load()) {
                usleep(1000 * 100); // Sleep for 100ms before checking again
            }
            std::cout << "Resuming repositioning" << std::endl;
        }
        // cap >> frame;
        if (!reader.getFrame(frame)) {
            std::cout << "No available frame" << std::endl;
            continue;}
        // When image arrives:
        double img_ts = AlgoLogger::nowWallSec();
        // std::cout << "Processing frame for alignment..." << std::endl;
        // Get alignment direction (x, y, z)
        // auto displacement_ = matcher.getAlignmentDisplacement(frame);
        // cv::Point3f displacement1;
        auto displacement_ = matcher.getAlignmentDisplacementRansac(frame);
        std::cout << " Displacement (pixels): " << displacement_ << std::endl;
        // std::cout << "Processing frame for Rotation..." << std::endl;
        auto [rotationMatrix, direction1]  = matcher.getAlignmentDirection();
        direction1 *= 100;           // convert to cm (tune this scale factor based on actual camera FOV and target size)
        // cv::Point3f direction = cv::Point3f(0,0,0);
        cv::Mat I = cv::Mat::eye(3, 3, rotationMatrix.type());
        cv::Mat diff = rotationMatrix - I;
        cv::Mat world_rotation_vec;
        cv::Rodrigues(rotationMatrix, world_rotation_vec);
        world_rotation_vec.convertTo(world_rotation_vec, CV_64F);
        cv::Point3f rotation_vec = rotmatToYPRDeg_ZYX(rotationMatrix);

        cv::Mat axis = world_rotation_vec / cv::norm(world_rotation_vec);
        float angle = cv::norm(world_rotation_vec) * (180.0f / CV_PI); // degrees
        cv::Point3f rotation, translation, displacement;

        // // usage
        // double yawErr   = wrapDeg(rotation_vec.yaw);
        // double pitchErr = wrapDeg(rotation_vec.pitch);
        // double rollErr  = wrapDeg(rotation_vec.roll);
        
        // float yaw_rate_max = yaw_rate_max_ * (1.f - std::exp(- k_yaw * angle));
        // float v_maxX = v_max_ * (1.f - std::exp(- k * cv::norm(displacement)));
        // std::cout << "Max yaw rate: " << yaw_rate_max << ", Angle: " << angle << std::endl;
        // double yawRateCmd   = tanhRate(yawErr,   yaw_rate_max, k_yaw);  // deg/s
        // double pitchRateCmd = tanhRate(pitchErr, yaw_rate_max, k_yaw);
        // double rollRateCmd  = tanhRate(rollErr,  yaw_rate_max, k_yaw);

        if (unrealMode){
            // rotation.x = axis.at<double>(2) * angle;
            // rotation.y = axis.at<double>(0) * angle;
            // rotation.z = -axis.at<double>(1) * angle;
            // angle_rate_cmd.x = yawRateCmd;
            // angle_rate_cmd.y = rollRateCmd;
            // angle_rate_cmd.z = -pitchRateCmd;

            translation = ConvertCVToUE(direction1);
            displacement = ConvertCVToUE(displacement_);
            rotation = ConvertCVToUE(rotation_vec);
        }
        else{
            rotation.x = axis.at<double>(0);
            rotation.y = axis.at<double>(1);
            rotation.z = axis.at<double>(2);
            translation = displacement;
        }   

        float v_maxX = v_max ; // * (1.f - std::exp(- k * displacement.x));
        float v_maxY = v_max * (1.f - std::exp(- k * abs(displacement.y))) / (1.f - std::exp(- k * imgWidth )); // scale by max possible displacement
        float v_maxZ = v_max * (1.f - std::exp(- k * abs(displacement.z))) / (1.f - std::exp(- k * imgHeight )); // scale by max possible displacement

        float w_maxX = w_max * (1.f - std::exp(- k_yaw * abs(angle)));

        // std::cout << "v_maxX: " << v_maxX << ", v_maxY: " << v_maxY << ", v_maxZ: " << v_maxZ << std::endl;
        cmd_vx.x = v_maxX * std::tanh(k * translation.x);
        cmd_vx.y = v_maxY * std::tanh(k * translation.y);
        cmd_vx.z = v_maxZ * std::tanh(k * translation.z);
        // std::cout << "Raw cmd_vx before smoothing: " << cmd_vx << std::endl;

        angle_rate_cmd.x = w_maxX * std::tanh(k_yaw   * rotation.x);
        angle_rate_cmd.y = w_maxX * std::tanh(k_pitch * rotation.y);
        angle_rate_cmd.z = 0; // roll control is often less critical for drone reposition

        // cmd_vx = cv::Point3f(
        //     v_maxX * translation.x,     // translation is already normmalized by actual displacement, so this is roughly v_max * tanh(k * displacement)
        //     v_maxY * translation.y,    
        //     v_maxZ * translation.z
        // ); 

        // if (switching >= 5) { // if switched direction 5 times, likely oscillating near target
        //     cmd_vx.x = 0;
        //     std::cout << "Too much switching, stopping movement commands." << std::endl;
        // }
        int mean_previous_direction = (direction_history.empty() || direction_history.size() < 8) ? -2 : std::round(std::accumulate(direction_history.begin(), direction_history.end(), 0.0) / direction_history.size());
        // int current_direction = (displacement.x >= 0) ? 1 : -1;
        if (mean_previous_direction == 0) cmd_vx.x = 0; // if historically around target, stop sending x velocity to avoid oscillation
        // yaw/pitch rate from angular error (separate gains!)
        // angle_rate_cmd.x = yaw_rate_max * std::tanh(k_yaw   * rotation.x);
        // angle_rate_cmd.y = pitch_rate_max * std::tanh(k_pitch * rotation.y);

        // if (old_img_ts != 0) dt = img_ts - old_img_ts;
        // dt-based smoothing
        // float alpha = 1.f - std::exp(- k_yaw * angle);
        float alpha = 0.5; // fixed smoothing factor (tune this)

        cmd_vx = alpha * cmd_vx + (1 - alpha) * last_cmd_vx;
        // std::cout << "Smoothed cmd_vx: " << cmd_vx << std::endl;
        angle_rate_cmd = alpha * angle_rate_cmd + (1 - alpha) * last_angle_rate;
        error_ang = cv::norm(diff);
        // error_dist = cv::norm(displacement);
        // error_dist = abs(translation.x) + abs(translation.y) + abs(translation.z);
        error_dist = cv::norm(cv::Point2f(displacement.z, displacement.y));
        std::cout << "Error angle: " << error_ang << ", Error distance: " << error_dist << std::endl;
        // std::cout << " rotation_x: " << angle_rate_cmd.x << " rotation_y: " << angle_rate_cmd.y << std::endl;
        // std::cout << " direction_x: " << direction.x << " direction_y: " << direction.y<< std::endl;
        // std::cout << " cmd_vx_x: " << cmd_vx.x << " cmd_vx_y: " << cmd_vx.y << " cmd_vx_z: " << cmd_vx.z << std::endl;
        // std::cout << "Displacement px: " << displacement.x << ", " << displacement.y << ", " << displacement.z << std::endl;
        // std::cout << "Displacement: " << translation.x << ", " << translation.y << ", " << translation.z << std::endl;
        // std::cout << " Error: " << error_ang << " Error dist: " << error_dist << " YawErr: " << yawErr << " PitchErr: " << pitchErr << " RollErr: " << rollErr << std::endl;
        std::stringstream ss;
        // YawRate_cam, PitchRate_cam, YawRate_drone, drone_x, drone_y, drone_z \n 
        // ss << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "\n";
        // std::string data_0 = ss.str();  
        
        double send_ts = 0;
        if (!client.rotationOnly){
            if (error_dist > dist_threshold && abs(displacement.x) > 1.5e-1){
                if (abs(displacement.z) > 0.15 * imgHeight){
                    ss << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << cmd_vx.z << "," << "1" << "\n";
                    data_to_send = ss.str();
                }
                else if (error_dist > dist_threshold)
                {
                    ss << angle_rate_cmd.x << "," << 0 << "," << 0 << "," << cmd_vx.x << "," << cmd_vx.y << "," << 0 << "," << "1" << "\n";
                    data_to_send = ss.str();
                }
                // ss << 0 << "," << 0 << "," << 0 << "," << cmd_vx.x << "," << cmd_vx.y << "," << cmd_vx.z << "," << "0" << "\n";
                // data_to_send = ss.str();
                if (!client.SendMetadata(data_to_send)) 
                {
                    std::cerr << "Failed to send data. Check connection." << std::endl;
                    break;
                }
                send_ts = AlgoLogger::nowWallSec();
                std::cout << "Data sent: " << data_to_send << std::endl;
                // increment = 0;
                // last_frame = img_ts;
                // data_to_send = "";
                complete = true;
                last_frame = last_frame_init;
            }
        }
        if (!client.translationOnly){
            if (error_ang > threshold_error ){
                ss << angle_rate_cmd.x << "," << angle_rate_cmd.y << "," << angle_rate_cmd.z << "," << 0 << "," << 0 << "," << 0 << "," << "0" << "\n";
                data_to_send = ss.str();
                if (!client.SendMetadata(data_to_send)) 
                {
                    std::cerr << "Failed to send data. Check connection." << std::endl;
                    break;
                }
                send_ts = AlgoLogger::nowWallSec();
                std::cout << "Data sent: " << data_to_send << std::endl;
                // increment = 0;
                // last_frame = img_ts;
                // data_to_send = "";
                complete = true;
                last_frame = last_frame_init;
            }
        }
        // ss << angle_rate_cmd.x << "," << angle_rate_cmd.y << "," << angle_rate_cmd.z << "," << cmd_vx.x << "," << cmd_vx.y << "," << cmd_vx.z << "," << "1" << "\n";
        // data_to_send = ss.str();
        // // if (data_to_send != "" ){
        // if (error_ang > threshold_error && abs(error_dist) > dist_threshold){
        //     if (!client.SendMetadata(data_to_send)) 
        //     {
        //         std::cerr << "Failed to send data. Check connection." << std::endl;
        //         break;
        //     }
        //     send_ts = AlgoLogger::nowSec();
        //     std::cout << "Data sent: " << data_to_send << std::endl;
        //     // increment = 0;
        //     // last_frame = img_ts;
        //     // data_to_send = "";
        //     complete = true;
        // }  
        else{
            if (complete){
                last_frame = AlgoLogger::nowWallSec();
                complete = false;
            } 
        }
        // optional parse (so you can graph cmd5 easily)
        auto parsed = AlgoLogger::parseCommand6(data_to_send);
        logger.log(img_ts, send_ts, angle_rate_cmd, cmd_vx, data_to_send, parsed);
        time_now = AlgoLogger::nowWallSec();
        std::cout << "Last time: " << last_frame << ", time now: " << time_now <<std::endl;
        if( time_now - last_frame >= 2) {
            std::cout << "Arrived at destination" << std::endl;
            break;
        }
        last_cmd_vx = cmd_vx;
        last_angle_rate = angle_rate_cmd;
        direction_history.push_back((displacement.x >= 0) ? 1 : -1);
        if (direction_history.size() > 10) direction_history.erase(direction_history.begin());
        if (client.stopRepositioning.load()) {
            std::cout << "Received command to stop repositioning" << std::endl;
            break;
        }
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

    client.CloseSocket();
    client.stopReceiver();
    cap.release();
    // cv::destroyAllWindows();
    return 0;
}
