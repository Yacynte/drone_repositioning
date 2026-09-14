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
#include "Logger.h"

// Define invalid socket and error check based on POSIX conventions
#define INVALID_SOCKET -1
#define SOCKET_ERROR   -1

/**
 * @brief Manages the two POSIX socket connections used to talk to the outside world:
 * a "command_handler" connection the operator/Unreal Engine uses to start/stop/pause
 * repositioning (received on a background thread, see startReceiver()/receiveCommand()),
 * and a "metadata_server" connection used to send computed drone commands out via
 * SendMetadata(). Command state is exposed as atomics so main.cpp's loop thread can
 * read it safely while receiveCommand() runs on rxThread.
 */
class MetadataTcpClient
{
private:
    Logger& logger;
    // POSIX socket file descriptor
    int client_socket = INVALID_SOCKET;
    int server_socket = INVALID_SOCKET;
    struct sockaddr_in address{};
    socklen_t addr_len ;
    bool IsConnected() const;

public:
    // Spawns rxThread running receiveCommand() to consume incoming commands in the background.
    void startReceiver();
    // Signals rxThread to stop and joins it.
    void stopReceiver();
    // Closes the given handler socket: -2 = both, 0 = server_socket, 1 = client_socket.
    void CloseConnectionhandler(int socket_to_close = -2);
    // Converts the current rotation/translation error and rate into a 7-field CSV
    // command ("roll,pitch,yaw,vx,vy,vz,state\n"), writes it to data_to_send, and sends
    // it via SendMetadata(). Returns true once both rotation and translation errors are
    // within tolerance (i.e. the drone has arrived at the target).
    bool respositionFunc(cv::Point3f rotation_rate, cv::Point3f translation_rate, const cv::Point3f rot_error,
                        cv::Point3f translation, std::string& data_to_send, bool simulation);

    // flags (atomic = safe to write/read from different threads)
    std::atomic<bool> startRepositioning{false};
    std::atomic<bool> stopRepositioning{false};
    std::atomic<bool> pauseRepositioning{false};
    std::atomic<bool> resumeRepositioning{false};
    std::atomic<bool> rotationOnly{false};
    std::atomic<bool> translationOnly{false};
    std::atomic<bool> stopRotation{false};
    std::atomic<bool> stopTranslation{false};

    explicit MetadataTcpClient(Logger& logger);
    void CloseSocket();

    ~MetadataTcpClient()
    {
        CloseSocket();
    }

    // Connects client_socket (TCP) to ip:port. Used for the outgoing metadata connection.
    bool Connect(const std::string& ip = "127.0.0.1", int port = 9010);
    // Listens on ip:port (TCP) and accepts a single incoming connection, stored either
    // in server_socket ("command_handler") or client_socket ("metadata_server").
    bool StartConnectionHandlerTCP(const std::string& ip = "127.0.0.1", int port = 9020, const std::string client = "command_handler");
    // UDP equivalent of StartConnectionHandlerTCP: binds a UDP socket on ip:port that
    // receiveCommand() reads datagrams from.
    bool StartConnectionHandlerUDP(const std::string& ip = "127.0.0.1", int port = 9020, const std::string client = "command_handler");
    // Background-thread loop: reads newline-terminated commands (start/stop/pause/resume/...)
    // and updates the atomic flags above. Runs until runRx is cleared by stopReceiver().
    void receiveCommand();
    std::atomic<bool> runRx{false};
    std::thread rxThread;

    // for assembling lines across recv() calls
    std::string rxAccum;
    // Sends data_to_send (already-formatted command string) over client_socket.
    bool SendMetadata(const std::string& data_to_send);
};