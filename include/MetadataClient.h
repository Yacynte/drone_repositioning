#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <cstring>  // For std::strerror

// --- POSIX Sockets Headers ---
#include <sys/socket.h> // For socket(), connect(), send(), etc.
#include <netinet/in.h> // For sockaddr_in structure
#include <arpa/inet.h>  // For inet_pton()
#include <unistd.h>     // For close()
#include <cerrno>       // For errno

// Define invalid socket and error check based on POSIX conventions
#define INVALID_SOCKET -1
#define SOCKET_ERROR   -1

/**
 * @brief Manages the client connection to the metadata TCP server using POSIX sockets.
 */
class MetadataTcpClient 
{
private:
    // POSIX socket file descriptor
    int client_socket = INVALID_SOCKET;
    // Constants
    // int DEFAULT_PORT = 9001; // Default port for metadata server
    // std::string DEFAULT_IP = "127.0.0.1";
    bool IsConnected() const;  

public:
    /**
     * @brief Constructor. No special initialization required for POSIX.
     */
    MetadataTcpClient() = default;
    void CloseSocket();

    /**
     * @brief Destructor. Cleans up the socket.
     */
    ~MetadataTcpClient()
    {
        CloseSocket();
    }
    bool Connect(const std::string& ip = "172.28.240.1", int port = 9001);
    bool SendMetadata(float alpha, float angle);
    /**
     * @brief Attempts to connect to the metadata server.
     * @param ip The target IP address (e.g., "127.0.0.1").
     * @param port The target port (e.g., 9001).
     * @return True on successful connection, false otherwise.
     */
    /**
     * @brief Sends formatted metadata (alpha and angle) to the connected server.
     * * The format used is a comma-separated string: "Alpha,Angle\n".
     * @param alpha The alpha value (float).
     * @param angle The angle value (float).
     * @return True if the data was sent successfully, false otherwise.
     */

};