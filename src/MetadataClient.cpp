#include "MetadataClient.h"
#include <iostream>


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

    
bool MetadataTcpClient::SendMetadata(float roll, float pitch)
    {
        if (client_socket == INVALID_SOCKET) {
            std::cerr << "Error: Not connected. Call Connect() first." << std::endl;
            return false;
        }

        // Format the data as a string: "Alpha,Angle\n"
        std::stringstream ss;
        ss << roll << "," << pitch << "\n";
        std::string data_to_send = ss.str();

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
        std::cout << "Sent metadata: "<< data_to_send << std::endl;
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

        if (!client.SendMetadata(current_alpha, current_angle)) 
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