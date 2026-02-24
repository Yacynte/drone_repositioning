#include "MetadataClient.h"
#include <iostream>
#include <iostream>
#include <cstring>



bool MetadataTcpClient::Connect(const std::string& ip, int port)
{
    CloseSocket(); // Close any existing socket

    // 1. Create a socket (AF_INET = IPv4, SOCK_STREAM = TCP)
    client_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (client_socket == INVALID_SOCKET) {
        std::cerr << "Error at socket(): " << std::strerror(errno) << std::endl;
        return false;
    }

    // 2. Resolve the server address
    struct sockaddr_in server_addr;
    std::memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    
    // Convert IP string to network address structure
    if (inet_pton(AF_INET, ip.c_str(), &server_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address/Address not supported" << std::endl;
        CloseSocket();
        return false;
    }

    // 3. Connect to the server
    int result = connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr));
    if (result == SOCKET_ERROR) {
        std::cerr << "Connection failed with error: " << std::strerror(errno) << std::endl;
        CloseSocket();
        return false;
    }

    std::cout << "Successfully connected to server at " << ip << ":" << port << std::endl;
    return true;
}

    
bool MetadataTcpClient::SendMetadata(const std::string& data_to_send)
    {
        if (client_socket == INVALID_SOCKET) {
            std::cerr << "Error: Not connected. Call Connect() first." << std::endl;
            return false;
        }

        // Format the data as a string: "Alpha,Angle\n"
        // std::stringstream ss;
        // ss << roll << "," << pitch << "\n";
        // std::string data_to_send = ss.str();

        // Convert string to char array and get size
        const char* send_buf = data_to_send.c_str();
        size_t send_len = data_to_send.length();

        // Send the data
        ssize_t bytes_sent = send(client_socket, send_buf, send_len, 0);

        if (bytes_sent == SOCKET_ERROR) {
            std::cerr << "Send failed with error: " << std::strerror(errno) << std::endl;
            CloseSocket(); // Treat send failure as connection loss
            return false;
        }

        if ((size_t)bytes_sent != send_len) {
            // Partial send occurred
            std::cerr << "Warning: Only sent " << bytes_sent << " of " << send_len << " bytes." << std::endl;
        }
        
        // Success
        // std::cout << "Sent metadata: "<< data_to_send << std::endl;
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


void MetadataTcpClient::CloseConnectionhandler()
{
    if (server_socket != INVALID_SOCKET) {
        close(server_socket); // POSIX function to close a file descriptor
        server_socket = INVALID_SOCKET;
    }
    
}

bool MetadataTcpClient::StartConnectionHandler(const std::string& ip, int port)
{
    CloseConnectionhandler(); // Close any existing server socket
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("socket failed");
        return false;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);

    // Convert string IP to binary
    if (inet_pton(AF_INET, ip.c_str(), &address.sin_addr) <= 0) {
        perror("Invalid IP address");
        close(server_fd);
        return -1;
    }

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("bind failed");
        close(server_fd);
        return -1;
    }

    listen(server_fd, 1);

    std::cout << "Waiting on " << ip << ":" << port << std::endl;

    socklen_t addrlen = sizeof(address);
    server_socket = accept(server_fd, (struct sockaddr*)&address, &addrlen);
    std::cout << "server_socket fd = " << server_socket << std::endl;
    close(server_fd); // No longer need the listening socket
    if (server_socket < 0) {
        perror("accept failed");
        return false;
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
        ssize_t n = recv(server_socket, buf, sizeof(buf), 0);

        if (n == 0) {
            std::cerr << "Client disconnected.\n";
            break;
        }
        if (n < 0) {
            if (!runRx) break; // likely shutdown() triggered
            perror("recv failed");
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
                // std::cout << "Received command to start repositioning" << std::endl;
            } else if (cmd == "stop_repositioning" || cmd == "stop") {
                stopRepositioning.store(true);
                startRepositioning.store(false); // reset start flag
                // std::cout << "Received command to stop repositioning" << std::endl;
            } else if (cmd == "pause_repositioning" || cmd == "pause") {
                pauseRepositioning.store(true);
                resumeRepositioning.store(false); // reset resume flag
            } else if (cmd == "resume_repositioning" || cmd == "resume") {
                resumeRepositioning.store(true);
                pauseRepositioning.store(false); // reset pause flag
            } else if (cmd == "rotation_only") {
                rotationOnly.store(true);
                translationOnly.store(false); // reset translation-only flag
                // stopTranslation.store(true); // stop translation if switching to rotation-only
            } else if (cmd == "translation_only") {
                translationOnly.store(true);
                rotationOnly.store(false); // reset rotation-only flag
                // stopRotation.store(true); // stop rotation if switching to translation-only
            } else {
                std::cerr << "Unknown command: '" << cmd << "'\n";
            }
        }
    }

    runRx = false;
}


void MetadataTcpClient::startReceiver() {
    if (server_socket < 0) {
        std::cerr << "No client connected.\n";
        return;
    }
    runRx = true;
    rxThread = std::thread(&MetadataTcpClient::receiveCommand, this);
}

void MetadataTcpClient::stopReceiver() {
    runRx = false;

    // If recv() is blocking, shutting down the socket will unblock it.
    if (server_socket >= 0) {
        shutdown(server_socket, SHUT_RDWR);
    }
    CloseConnectionhandler(); // Ensure server socket is closed

    if (rxThread.joinable()) rxThread.join();
}

// --- Example Usage ---
static void TestClient()
{
    MetadataTcpClient client;
    
    // 1. Connect (Uses default 127.0.0.1:9001)
    if (!client.Connect()) {
        std::cerr << "Failed to connect to server. Ensure the server is running on port 9001." << std::endl;
        return;
    }

    // 2. Send some data
    float current_alpha = 0.5f;
    float current_angle = 90.0f;

    for (int i = 0; i < 5; ++i) 
    {
        current_alpha += 0.1f;
        current_angle += 10.0f;
        
        std::cout << "Attempting to send: " << current_alpha << "," << current_angle << std::endl;
        std::stringstream ss;
        ss << current_alpha << "," << current_angle << "\n";
        std::string data_to_send = ss.str();

        if (!client.SendMetadata(data_to_send)) 
        {
            std::cerr << "Failed to send data. Check connection." << std::endl;
            break;
        }
        std::cout << "Data sent successfully." << std::endl;
        
        // Wait a moment before sending the next update (use usleep for cross-platform)
        usleep(1000 * 1000); // 1 second delay
    }
    
    std::cout << "Test complete. Client shutting down." << std::endl;
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