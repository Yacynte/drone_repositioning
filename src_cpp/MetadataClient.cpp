#include "MetadataClient.h"
#include <iostream>
#include <iostream>
#include <cstring>


MetadataTcpClient::MetadataTcpClient(Logger& logger): logger(logger) {}


bool MetadataTcpClient::Connect(const std::string& ip, int port)
{
    CloseSocket(); // Close any existing socket

    // 1. Create a socket (AF_INET = IPv4, SOCK_STREAM = TCP)
    client_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (client_socket == INVALID_SOCKET) {
        {
            std::ostringstream ss;
            ss << "Error at socket(): " << std::strerror(errno);
            logger.log("MetadataClient", ss.str());
        }
        return false;
    }

    // 2. Resolve the server address
    struct sockaddr_in server_addr;
    std::memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    
    // Convert IP string to network address structure
    if (inet_pton(AF_INET, ip.c_str(), &server_addr.sin_addr) <= 0) {
        logger.log("MetadataClient", "Invalid address/Address not supported");
        CloseSocket();
        return false;
    }

    // 3. Connect to the server
    int result = connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr));
    if (result == SOCKET_ERROR) {
        {
            std::ostringstream ss;
            ss << "Connection failed with error: " << std::strerror(errno);
            logger.log("MetadataClient", ss.str());
        }
        CloseSocket();
        return false;
    }

    // std::cout << "Successfully connected to server at " << ip << ":" << port << std::endl;
    {
        std::ostringstream ss;
        ss << "Successfully connected to server at " << ip << ":" << port;
        logger.log("MetadataClient", ss.str());
    }
    return true;
}

    
bool MetadataTcpClient::SendMetadata(const std::string& data_to_send)
    {
        if (client_socket == INVALID_SOCKET) {
            logger.log("MetadataClient", "Error: Not connected. Call Connect() first.");
            return false;
        }

        // Format the data as a string: "Alpha,Angle\n"
        // std::stringstream ss;
        // ss << roll << "," << pitch << "\n";
        // std::string data_to_send = ss.str();

        // // Convert string to char array and get size
        // const char* send_buf = data_to_send.c_str();
        // size_t send_len = data_to_send.length();

        // // Send the data
        // ssize_t bytes_sent = send(client_socket, send_buf, send_len, 0);

        

        // 2. Send the data
        const char* send_buf = data_to_send.c_str();
        size_t send_len = data_to_send.length();

        ssize_t bytes_sent = sendto(client_socket, send_buf, send_len, 0, (struct sockaddr*)&address, sizeof(address));

        if (bytes_sent == SOCKET_ERROR) {
            {
                std::ostringstream ss;
                ss << "Send failed with error: " << std::strerror(errno);
                logger.log("MetadataClient", ss.str());
            }
            CloseSocket(); // Treat send failure as connection loss
            return false;
        }

        if ((size_t)bytes_sent != send_len) {
            // Partial send occurred
            {
                std::ostringstream ss;
                ss << "Warning: Only sent " << bytes_sent << " of " << send_len << " bytes.";
                logger.log("MetadataClient", ss.str());
            }
        }
        
        // Success
        {
            std::ostringstream ss;
            ss << "Sent metadata: " << data_to_send;
            logger.log("MetadataClient", ss.str());
        }
        return true;
    }

/**
 * @brief Cleans up the socket resource.
 */
void MetadataTcpClient::CloseSocket()
    {
        if (client_socket != INVALID_SOCKET) {
            close(client_socket); // POSIX function to close a file descriptor
            client_socket = INVALID_SOCKET;
        }
        
    }

/**
 * @brief Checks if the client is currently connected.
 */
bool MetadataTcpClient::IsConnected() const
{
    return client_socket != INVALID_SOCKET;
}


void MetadataTcpClient::CloseConnectionhandler( int socket_to_close)
{
    if (socket_to_close != INVALID_SOCKET) {
        close(socket_to_close); // POSIX function to close a file descriptor
        socket_to_close = INVALID_SOCKET;
    } else if (socket_to_close == -2 ) {
        if (server_socket != INVALID_SOCKET) {
            close(server_socket); // POSIX function to close a file descriptor
            server_socket = INVALID_SOCKET;
        }
        if (client_socket != INVALID_SOCKET) {
            close(client_socket); // POSIX function to close a file descriptor
            client_socket = INVALID_SOCKET;
        }
    }
    
}

bool MetadataTcpClient::StartConnectionHandlerTCP(const std::string& ip, int port, const std::string client)
{
    int socet_to_close = (client == "command_handler") ? server_socket : client_socket;
    CloseConnectionhandler(socet_to_close); // Close any existing server socket
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        {
            std::ostringstream ss;
            ss << "socket failed: " << std::strerror(errno);
            logger.log("MetadataClient", ss.str());
        }
        return false;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);

    // Convert string IP to binary
    if (inet_pton(AF_INET, ip.c_str(), &address.sin_addr) <= 0) {
        {
            std::ostringstream ss;
            ss << "Invalid IP address: " << std::strerror(errno);
            logger.log("MetadataClient", ss.str());
        }
        close(server_fd);
        return -1;
    }

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        {
            std::ostringstream ss;
            ss << "bind failed: " << std::strerror(errno);
            logger.log("MetadataClient", ss.str());
        }
        close(server_fd);
        return -1;
    }

    listen(server_fd, 1);

    {
        std::ostringstream ss;
        ss << "Waiting on " << client << " at: " << ip << ":" << port;
        logger.log("MetadataClient", ss.str());
    }

    socklen_t addrlen = sizeof(address);
    if (client == "command_handler") {
        // std::cout << "Waiting for server to connect..." << std::endl;
        server_socket = accept(server_fd, (struct sockaddr*)&address, &addrlen);
        {
            std::ostringstream ss;
            ss << "server_socket fd = " << server_socket;
            logger.log("MetadataClient", ss.str());
        }
        if (server_socket < 0) {
            {
                std::ostringstream ss;
                ss << "accept failed: " << std::strerror(errno);
                logger.log("MetadataClient", ss.str());
            }
            return false;
        }
    } else if (client == "metadata_server") {
        // std::cout << "Waiting for client to connect..." << std::endl;
        client_socket = accept(server_fd, (struct sockaddr*)&address, &addrlen);
        {
            std::ostringstream ss;
            ss << "client_socket fd = " << client_socket;
            logger.log("MetadataClient", ss.str());
        }
        if (client_socket < 0) {
            {
                std::ostringstream ss;
                ss << "accept failed: " << std::strerror(errno);
                logger.log("MetadataClient", ss.str());
            }
            return false;
        }
    }
    
    
    close(server_fd); // No longer need the listening socket
    
    return true;
}

bool MetadataTcpClient::StartConnectionHandlerUDP(const std::string& ip, int port, const std::string client)
{
    // 1. Create a UDP socket (SOCK_DGRAM instead of SOCK_STREAM)
    int socet_to_close = client_socket;
    CloseConnectionhandler(socet_to_close); 

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        {
            std::ostringstream ss;
            ss << "socket failed: " << std::strerror(errno);
            logger.log("MetadataClient", ss.str());
        }
        return false;
    }

    // 2. Setup Address
    // sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);

    if (inet_pton(AF_INET, ip.c_str(), &address.sin_addr) <= 0) {
        {
            std::ostringstream ss;
            ss << "Invalid IP address: " << std::strerror(errno);
            logger.log("MetadataClient", ss.str());
        }
        close(sockfd);
        return false;
    }
    addr_len = sizeof(address);
    // 3. Bind the socket (UDP still needs this to receive)
    if (bind(sockfd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        {
            std::ostringstream ss;
            ss << "bind failed: " << std::strerror(errno);
            logger.log("MetadataClient", ss.str());
        }
        close(sockfd);
        return false;
    }

    // UDP does NOT use listen() or accept().
    // We are now ready to receive data using recvfrom().
    
    client_socket = sockfd;

    {
        std::ostringstream ss;
        ss << "UDP receiver ready on " << ip << ":" << port;
        logger.log("MetadataClient", ss.str());
    }
    return true;
}

static inline void trim_inplace(std::string& s) {
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' ' || s.back() == '\t'))
        s.pop_back();
    size_t i = 0;
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) i++;
    if (i) s.erase(0, i);
}

void MetadataTcpClient::receiveCommand() {
    char buf[1024];

    while (runRx) {
        // ssize_t n = recv(client_socket, buf, sizeof(buf), 0);
        // char buf[1024];
        // struct sockaddr_in sender_addr;
        // socklen_t addr_len = sizeof(sender_addr);
        ssize_t n = recvfrom(client_socket, buf, sizeof(buf), 0, (struct sockaddr*)&address, &addr_len);

        if (n == 0) {
            logger.log("MetadataClient", "Client disconnected.");
            break;
        }
        if (n < 0) {
            if (!runRx) break; // likely shutdown() triggered
            {
                std::ostringstream ss;
                ss << "recv failed: " << std::strerror(errno);
                logger.log("MetadataClient", ss.str());
            }
            break;
        }

        rxAccum.append(buf, buf + n);

        // Process complete lines
        size_t pos;
        while ((pos = rxAccum.find('\n')) != std::string::npos) {
            std::string cmd = rxAccum.substr(0, pos);
            rxAccum.erase(0, pos + 1);

            // std::cout << "RAW cmd bytes: ";
            // for (unsigned char c : cmd) std::cout << int(c) << ' ';
            // std::cout << "\nCMD='" << cmd << "'\n";


            trim_inplace(cmd);
            if (cmd.empty()) continue;

            // --- FIXED comparisons ---
            if (cmd == "start_repositioning" || cmd == "start") {
                startRepositioning.store(true);
                stopRepositioning.store(false); // reset stop flag
                logger.log("MetadataClient", "Received command to start repositioning");
            } else if (cmd == "stop_repositioning" || cmd == "stop") {
                stopRepositioning.store(true);
                // startRepositioning.store(false); // reset start flag
                logger.log("MetadataClient", "Received command to stop repositioning");
            } else if (cmd == "pause_repositioning" || cmd == "pause") {
                pauseRepositioning.store(true);
                resumeRepositioning.store(false); // reset resume flag
                logger.log("MetadataClient", "Received command to pause repositioning");
            } else if (cmd == "resume_repositioning" || cmd == "resume") {
                resumeRepositioning.store(true);
                pauseRepositioning.store(false); // reset pause flag
                logger.log("MetadataClient", "Received command to resume repositioning");
            } else if (cmd == "rotation_only") {
                rotationOnly.store(true);
                translationOnly.store(false); // reset translation-only flag
                logger.log("MetadataClient", "Received command to switch to rotation-only mode");
                // stopTranslation.store(true); // stop translation if switching to rotation-only
            } else if (cmd == "resume_both") {
                rotationOnly.store(false);
                translationOnly.store(false); // reset translation-only flag
                logger.log("MetadataClient", "Received command to resume both rotation and translation");
                // stopTranslation.store(true); // stop translation if switching to rotation-only
            } else if (cmd == "translation_only") {
                translationOnly.store(true);
                rotationOnly.store(false); // reset rotation-only flag
                logger.log("MetadataClient", "Received command to switch to translation-only mode");
                // stopRotation.store(true); // stop rotation if switching to translation-only
            } else {
                {
                    std::ostringstream ss;
                    ss << "Unknown command: '" << cmd << "'";
                    logger.log("MetadataClient", ss.str());
                }
            }
        }
    }

    runRx = false;
}


void MetadataTcpClient::startReceiver() {
    if (client_socket < 0) {
        logger.log("MetadataClient", "No client connected.");
        return;
    }
    runRx = true;
    rxThread = std::thread(&MetadataTcpClient::receiveCommand, this);
}

void MetadataTcpClient::stopReceiver() {
    runRx = false;

    // If recv() is blocking, shutting down the socket will unblock it.
    if (client_socket >= 0) {
        shutdown(client_socket, SHUT_RDWR);
    }
    CloseConnectionhandler(); // Ensure server socket is closed

    if (rxThread.joinable()) rxThread.join();
}

// --- Example Usage ---
static void TestClient()
{
    Logger testLogger("/tmp/MetadataClient_test.log");
    MetadataTcpClient client(testLogger);
    
    // 1. Connect (Uses default 127.0.0.1:9001)
    if (!client.Connect()) {
        testLogger.log("MetadataClient", "Failed to connect to server. Ensure the server is running on port 9001.");
        return;
    }

    // 2. Send some data
    float current_alpha = 0.5f;
    float current_angle = 90.0f;

    for (int i = 0; i < 5; ++i) 
    {
        current_alpha += 0.1f;
        current_angle += 10.0f;
        
        {
            std::ostringstream ss;
            ss << "Attempting to send: " << current_alpha << "," << current_angle;
            testLogger.log("MetadataClient", ss.str());
        }
        std::stringstream ss;
        ss << current_alpha << "," << current_angle << "\n";
        std::string data_to_send = ss.str();

        if (!client.SendMetadata(data_to_send)) 
        {
            testLogger.log("MetadataClient", "Failed to send data. Check connection.");
            break;
        }
        testLogger.log("MetadataClient", "Data sent successfully.");
        
        // Wait a moment before sending the next update (use usleep for cross-platform)
        usleep(1000 * 1000); // 1 second delay
    }
    
    testLogger.log("MetadataClient", "Test complete. Client shutting down.");
}


// Uncomment the main function below to test this client code standalone.
/*
int main()
{
    // You MUST run the C++ application (your game) that hosts the Metadata TCP Server
    // on port 9001 before running this test client.
    MetadataTcpClient::TestClient();
    return 0;
}
*/


// bool MetadataTcpClient::respositionFuncOld(cv::Point3f rotation_rate, cv::Point3f translation_rate, const cv::Point3f rot_error, cv::Point3f translation, std::string& data_to_send, bool simulation) {

//         static bool doingTrans =  rotationOnly ? false : true; // if starting in rotation-only mode, start with rotation; otherwise start with translation
//         static float minRot = 1.0f;
//         static float minTrans = 10.0f;
//         static bool hardStop = false;
//         // static float targetRot = std::min(10.0f, std::max(minRot, rot_error/2)); // start with half the initial error, but cap to 10 to avoid long waits
//         static cv::Point3f targetRot;
//         targetRot.x = std::clamp(std::abs(rot_error.x) / 2.0f, minRot, 10.0f);
//         targetRot.y = std::clamp(std::abs(rot_error.y) / 2.0f, minRot, 10.0f);
//         targetRot.z = std::clamp(std::abs(rot_error.z) / 2.0f, minRot, 10.0f);
//         static cv::Point3f targetTrans;
//         if (simulation){
//             targetTrans.x = std::clamp(std::abs(translation.x) / 2.0f, minTrans, 100.0f);
//             targetTrans.y = std::clamp(std::abs(translation.y) / 2.0f, minTrans, 100.0f);
//             targetTrans.z = std::clamp(std::abs(translation.z) / 2.0f, minTrans, 100.0f);
//         }
//         else{
//             targetTrans.x = std::clamp(std::abs(translation.x) / 2.0f, minTrans, 100.0f);
//             targetTrans.y = std::clamp(std::abs(translation.y) / 2.0f, minTrans, 100.0f);
//             targetTrans.z = std::clamp(std::abs(translation.z) / 2.0f, minTrans, 100.0f);
//         }
//         static StopDetector detector;
//         static int increment_switch_x = 0;
//         static int increment_switch_y = 0;
//         static int increment_switch_z = 0;
//         static bool oscillate_x = false;
//         static bool oscillate_y = false;
//         static bool oscillate_z = false;
//         // std::string data_to_send = "";
//         // if ((rot_error < minRot) && (trans_error < minTrans)) return true;
//         float trans_error = cv::norm(translation);
//         if (doingTrans && !rotationOnly && trans_error >= minTrans) {
//             auto [oscillate_x, oscillate_y, oscillate_z] = detector.update(translation/trans_error);
//             if (oscillate_x) increment_switch_x++;
//             else increment_switch_x = 0;
//             if (oscillate_x) increment_switch_y++;
//             else increment_switch_y = 0;
//             if (oscillate_z) increment_switch_z++;
//             else increment_switch_z = 0;
//         }

//         if (doingTrans && !rotationOnly){
//             if ((increment_switch_x > 2) ||  (!simulation && (std::abs(translation.x) < minTrans))) {
//                 translation_rate.x = 0;
//                 translation.x = 0;
//                 // doingTrans = false;
//                 if (std::abs(translation.x) < 20) oscillate_x = true;
//             }
//             if (increment_switch_y > 2 || ((std::abs(translation.x) < minTrans)&&simulation) || (!simulation && (std::abs(translation.y) < minTrans))) {
//                 translation_rate.y = 0;
//                 translation.y = 0;
//                 // doingTrans = false;
//                 if (std::abs(translation.y) < 20) oscillate_y = true;
//             }
//             if (increment_switch_z > 2 || ((std::abs(translation.z) < minTrans)&& simulation) ) {
//                 translation_rate.z = 0;
//                 translation.z = 0;
//                 // doingTrans = false;
//                 if (std::abs(translation.z) < 20) oscillate_z = true;
//             }
//         }
//         if (hardStop){
//             translation = cv::Point3f(0.0f, 0.0f, 0.0f);
//             translation_rate = cv::Point3f(0.0f, 0.0f, 0.0f);
//             // transError = 0;
//             doingTrans = false;
//             // translationOnly = false;
//             // rotationOnly = true;
//         }

//         {
//             std::ostringstream ss;
//             ss << "Increment switch: " << increment_switch_x << ", " << increment_switch_y << ", " << increment_switch_z;
//             logger.log("MetadataClient", ss.str());
//         }

//         trans_error = cv::norm(translation);
              
//         if (!doingTrans && !translationOnly  && !simulation) { 
//             if ((std::abs(rot_error.y) < targetRot.y) && (std::abs(rot_error.z) < targetRot.z) ) {
//                 doingTrans = rotationOnly ? false : true; // if in rotation-only mode, stay in rotation; otherwise switch to translation
//                 targetRot.x = std::max(targetRot.x/2, minRot);
//                 targetRot.y = std::max(targetRot.y/2, minRot);
//                 targetRot.z = std::max(targetRot.z/2, minRot);
//                 increment_switch_x = 0; 
//                 increment_switch_y = 0;
//                 increment_switch_z = 0;
//             }
//             else {
//                 std::stringstream ss;
//                 float roll = 0;
//                 float pitch = rotation_rate.y;
//                 float yaw = rotation_rate.z;
//                 // if (rot_error.x < targetRot.x) roll = 0;
//                 if (std::abs(rot_error.y) < targetRot.y) pitch = 0;
//                 if (std::abs(rot_error.z) < targetRot.z) yaw = 0;
//                 ss << roll << "," << pitch << "," << yaw << "," << 0 << "," << 0 << "," << 0 << "," << "0" << "\n";
//                 data_to_send = ss.str();
//                 if (!MetadataTcpClient::SendMetadata(data_to_send)){
//                     logger.log("MetadataClient", "Could not send data");
//                 }
//             }
//         }
//         if (!doingTrans && !translationOnly  && simulation) { 
//             if ((std::abs(rot_error.y) < targetRot.y) && (std::abs(rot_error.x) < targetRot.x) ) {
//                 doingTrans = rotationOnly ? false : true; // if in rotation-only mode, stay in rotation; otherwise switch to translation
//                 targetRot.x = std::max(targetRot.x/2, minRot);
//                 targetRot.y = std::max(targetRot.y/2, minRot);
//                 targetRot.z = std::max(targetRot.z/2, minRot);
//                 increment_switch_x = 0; 
//                 increment_switch_y = 0;
//                 increment_switch_z = 0;
//             }
//             else {
//                 std::stringstream ss;
//                 float roll = 0;
//                 float pitch = rotation_rate.x;
//                 float yaw = rotation_rate.y;
//                 // if (rot_error.x < targetRot.x) roll = 0;
//                 if (std::abs(rot_error.x) < targetRot.x) pitch = 0;
//                 if (std::abs(rot_error.y) < targetRot.y) yaw = 0;
//                 ss << roll << "," << pitch << "," << yaw << "," << 0 << "," << 0 << "," << 0 << "," << "0" << "\n";
//                 data_to_send = ss.str();
//                 if (!MetadataTcpClient::SendMetadata(data_to_send)){
//                     logger.log("MetadataClient", "Could not send data");
//                 }
//             }
//         }
//         else if(doingTrans && !rotationOnly) {
            
//             if ((std::abs(translation.x) < targetTrans.x) && (std::abs(translation.y) < targetTrans.y) && (std::abs(translation.z) < targetTrans.z) ) {
//                 doingTrans = translationOnly ? true : false; // if in translation-only mode, stay in translation; otherwise switch to rotation
//                 targetTrans.x = std::max(targetTrans.x/2, minTrans);
//                 targetTrans.y = std::max(targetTrans.y/2, minTrans);
//                 targetTrans.z = std::max(targetTrans.z/2, minTrans);
//             }
//             else {
//                 std::stringstream ss;
//                 ss << 0 << "," << 0 << "," << 0 << "," << translation_rate.x << "," << translation_rate.y << "," << translation_rate.z << "," << "1" << "\n";
//                 data_to_send = ss.str();
//                 if (!MetadataTcpClient::SendMetadata(data_to_send)){
//                     logger.log("MetadataClient", "Could not send data");
//                 }
//             }
//         }
//         // std::cout << "Target rotation error " << targetRot << " and Target translation error " << targetTrans << std::endl;
//         // std::cout << "Rotation error " << rot_error << " and translation error " << trans_error << std::endl;
//         hardStop = oscillate_x && oscillate_y && oscillate_z;
//         bool rot_err = (std::abs(rot_error.y) <= minRot) && (std::abs(rot_error.z) <= minRot);
//         bool trans_err_bool = (std::abs(translation.x) <= minTrans) && (std::abs(translation.y) <= minTrans) && (std::abs(translation.z) <= minTrans);
//         if (hardStop) trans_err_bool = true;
//         bool state = ( rot_err || translationOnly ) && (trans_err_bool || rotationOnly );
//         // hardStop = oscillate_x && oscillate_y && oscillate_z && state;
//         // std::cout << "rotation state: " << rot_err << " and state of result: " << state << std::endl;
//         return state;
//     }


bool MetadataTcpClient::respositionFunc(cv::Point3f rotation_rate, cv::Point3f translation_rate, const cv::Point3f rot_error, cv::Point3f trans_error, std::string& data_to_send, bool simulation) {
    // static int64 time_init = static_cast<float>(cv::getTickCount());
    // int64 time_now = static_cast<float>(cv::getTickCount());
    // double time_diff = (time_now - time_init) / cv::getTickFrequency();
    static float minRot = 1.0f;
    static float minTrans = 1.0f;
    static bool hardStop = false;
    float roll = 0;
    // float pitch = 0;
    // float yaw = 0;
    float pitch = rotation_rate.y;
    float yaw = rotation_rate.z;
    if (std::abs(rot_error.y) < minRot) pitch = 0;
    if (std::abs(rot_error.z) < minRot) yaw = 0;
 
    float x = translation_rate.x;
    float y = translation_rate.y;
    float z = translation_rate.z;
    // if (rot_error.x < targetRot.x) roll = 0;
    
    if (std::abs(trans_error.x) < minTrans) x = 0;
    if (std::abs(trans_error.y) < minTrans) y = 0;
    if (std::abs(trans_error.z) < minTrans) z = 0;
    
    std::stringstream ss;
    ss << roll << "," << pitch << "," << yaw << "," << x << "," << y << "," << z << "," << "0" << "\n";
    data_to_send = ss.str();
    if (!MetadataTcpClient::SendMetadata(data_to_send)){
        logger.log("MetadataClient", "Could not send data");
    }
    bool rot_err = pitch == 0 && yaw == 0;
    bool trans_err_bool = x == 0 && y == 0 && z == 0;
    return rot_err && trans_err_bool;
}
