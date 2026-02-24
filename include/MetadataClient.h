#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <cstring>  // For std::strerror
#include "AlgoLogger.hpp"
// --- POSIX Sockets Headers ---
#include <sys/socket.h> // For socket(), connect(), send(), etc.
#include <netinet/in.h> // For sockaddr_in structure
#include <arpa/inet.h>  // For inet_pton()
#include <unistd.h>     // For close()
#include <cerrno>       // For errno
#include <unistd.h>     // close()
#include <sys/socket.h> // recv()
#include <thread>
#include <atomic>
#include <mutex>
#include <string>

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
    int server_socket = INVALID_SOCKET;
    // Constants
    // int DEFAULT_PORT = 9001; // Default port for metadata server
    // std::string DEFAULT_IP = "127.0.0.1";
    bool IsConnected() const;  

public:
    void startReceiver();
    void stopReceiver();
    void CloseConnectionhandler();

    // int server_socket = -1; // connected client socket

    // flags (atomic = safe to write/read from different threads)
    std::atomic<bool> startRepositioning{false};
    std::atomic<bool> stopRepositioning{false};
    std::atomic<bool> pauseRepositioning{false};
    std::atomic<bool> resumeRepositioning{false};
    std::atomic<bool> rotationOnly{false};
    std::atomic<bool> translationOnly{false};
    std::atomic<bool> stopRotation{false};
    std::atomic<bool> stopTranslation{false};

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
    // bool Connect(const std::string& ip = "192.168.0.200", int port = 9010);
    // bool StartConnectionHandler(const std::string& ip = "192.168.0.200", int port = 9020);
    bool Connect(const std::string& ip = "127.0.0.1", int port = 9010);
    bool StartConnectionHandler(const std::string& ip = "127.0.0.1", int port = 9020);
    void receiveCommand();
    // bool Connect(const std::string& ip = "127.0.0.1", int port = 9010);
    std::atomic<bool> runRx{false};
    std::thread rxThread;

    // for assembling lines across recv() calls
    std::string rxAccum;
    bool SendMetadata(const std::string& data_to_send);
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