#include "MetadataClient.h"
#include <iostream>
#include <iostream>
#include <cstring>


MetadataTcpClient::MetadataTcpClient(Logger& logger): logger(logger) {}


// Client-side TCP connect (used by TestClient() below; the live pipeline instead
// uses StartConnectionHandlerTCP/UDP to accept incoming connections).
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

    {
        std::ostringstream ss;
        ss << "Successfully connected to server at " << ip << ":" << port;
        logger.log("MetadataClient", ss.str());
    }
    return true;
}

    
// Sends data_to_send as a UDP datagram to `address` (the peer StartConnectionHandlerUDP
// last received from/bound to). Despite using sendto(), this relies on client_socket
// being a connected/bound socket set up by StartConnectionHandlerUDP or Connect().
bool MetadataTcpClient::SendMetadata(const std::string& data_to_send)
    {
        if (client_socket == INVALID_SOCKET) {
            logger.log("MetadataClient", "Error: Not connected. Call Connect() first.");
            return false;
        }

        // Send the data
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


// Closes socket_to_close directly if a valid fd is passed; if it's -2 (the default),
// closes both server_socket and client_socket instead.
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

// Binds and listens on ip:port (TCP), then blocks accepting exactly one incoming
// connection: stored in server_socket if client == "command_handler", or in
// client_socket otherwise ("metadata_server"). Not used by the live pipeline
// (main.cpp uses the UDP variant below for the command handler); kept for a
// TCP-based command channel if that's preferred over UDP.
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

// UDP equivalent of StartConnectionHandlerTCP: binds a UDP socket on ip:port
// (no listen/accept needed for UDP) and stores it in client_socket. This is what
// main.cpp actually uses for the "command_handler" channel — receiveCommand() then
// reads datagrams from it via recvfrom(). Note: unlike the TCP version, the `client`
// parameter is accepted but unused here (always goes to client_socket regardless).
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

// Strips trailing whitespace/newlines and leading spaces/tabs from s in place.
static inline void trim_inplace(std::string& s) {
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' ' || s.back() == '\t'))
        s.pop_back();
    size_t i = 0;
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) i++;
    if (i) s.erase(0, i);
}

// Runs on rxThread (started by startReceiver()): reads UDP datagrams into rxAccum,
// splits on '\n' into individual commands, and updates the atomic flags
// (startRepositioning, stopRepositioning, etc.) that main.cpp's loop polls. Exits
// when runRx is cleared (by stopReceiver()) or the peer disconnects/errors.
void MetadataTcpClient::receiveCommand() {
    char buf[1024];

    while (runRx) {
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


// Spawns rxThread running receiveCommand(). No-op if client_socket isn't set up yet
// (call StartConnectionHandlerUDP/TCP first).
void MetadataTcpClient::startReceiver() {
    if (client_socket < 0) {
        logger.log("MetadataClient", "No client connected.");
        return;
    }
    runRx = true;
    rxThread = std::thread(&MetadataTcpClient::receiveCommand, this);
}

// Signals receiveCommand() to stop, unblocks it if it's mid-recv (shutdown() on the
// socket), and joins rxThread.
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
// Standalone smoke test for Connect()/SendMetadata() against a listening TCP server.
// NOTE: this is a free function, not a MetadataTcpClient member — the commented-out
// main() below calls it as "MetadataTcpClient::TestClient()", which won't compile as
// written; drop the "MetadataTcpClient::" qualifier (or make this a static member) if
// you want to actually run this test.
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

// Converts rate/error into a "roll,pitch,yaw,vx,vy,vz,0" command string: zeroes out
// each axis once its error falls under the minRot/minTrans deadband (so the drone
// doesn't jitter around the target), sends it via SendMetadata(), and returns true
// once every axis is zeroed (i.e. within tolerance on both rotation and translation).
// Note: `simulation` is accepted (main.cpp passes unrealTest here) but currently
// unused — the deadband/command logic is the same for both simulation and hardware.
bool MetadataTcpClient::respositionFunc(cv::Point3f rotation_rate, cv::Point3f translation_rate, const cv::Point3f rot_error, cv::Point3f trans_error, std::string& data_to_send, bool simulation) {
    static float minRot = 1.0f;
    static float minTrans = 1.0f;
    float roll = 0;
    float pitch = rotation_rate.y;
    float yaw = rotation_rate.z;
    if (std::abs(rot_error.y) < minRot) pitch = 0;
    if (std::abs(rot_error.z) < minRot) yaw = 0;

    float x = translation_rate.x;
    float y = translation_rate.y;
    float z = translation_rate.z;

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
