import socket
import threading
from queue import Queue
import time


class ReceiveMsg:
    def __init__(self):
        self.message_queue = Queue()
        self.server_socket = None
        self.client_socket = None
        self.server_running = False
        self.client_connected = False
        self.lock = threading.Lock()

    def start_server(self, port=5000, host='0.0.0.0'):
        """Start a server on the specified port to receive messages from clients."""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((host, port))
            self.server_socket.listen(1)
            self.server_running = True
            print(f"Server started on {host}:{port}")
            
            # Start accepting connections in a separate thread
            threading.Thread(target=self._accept_connections, daemon=True).start()
        except Exception as e:
            print(f"Error starting server: {e}")

    def _accept_connections(self):
        """Accept incoming client connections and receive messages."""
        while self.server_running:
            try:
                client_conn, client_addr = self.server_socket.accept()
                print(f"Client connected from {client_addr}")
                threading.Thread(
                    target=self._receive_from_client,
                    args=(client_conn, client_addr),
                    daemon=True
                ).start()
            except Exception as e:
                if self.server_running:
                    print(f"Error accepting connection: {e}")

    def _receive_from_client(self, client_conn, client_addr):
        """Receive messages from a connected client and add them to the queue."""
        try:
            while self.server_running:
                message = client_conn.recv(1024).decode('utf-8')
                if message:
                    with self.lock:
                        self.message_queue.put(message)
                    print(f"Message from {client_addr}: {message}")
                else:
                    break
        except Exception as e:
            print(f"Error receiving from client {client_addr}: {e}")
        finally:
            client_conn.close()

    def connect_to_server(self, ip, port):
        """Connect to a server with the specified IP and port."""
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((ip, port))
            self.client_connected = True
            print(f"Connected to server at {ip}:{port}")
        except Exception as e:
            print(f"Error connecting to server: {e}")
            self.client_connected = False

    def print_from_queue(self):
        """Print the next message from the queue without removing it."""
        try:
            if not self.message_queue.empty():
                message = self.message_queue.queue[0]  # Peek at the message
                print(f"Queue message: {message}")
                return message
            else:
                print("Queue is empty")
                return None
        except Exception as e:
            print(f"Error accessing queue: {e}")
            return message

    def send_to_server(self, message):
        """Send a message to the connected server and dequeue the message."""
        if not self.client_connected or self.client_socket is None:
            print("Not connected to a server")
            return False
        
        try:
            self.client_socket.sendall(message.encode('utf-8'))
            print(f"Sent to server: {message}")
            
            # Dequeue the message after sending
            if not self.message_queue.empty():
                dequeued = self.message_queue.get()
                print(f"Dequeued message: {dequeued}")
            
            return True
        except Exception as e:
            print(f"Error sending to server: {e}")
            return False

    def get_from_queue(self):
        """Get and remove the next message from the queue."""
        try:
            if not self.message_queue.empty():
                return self.message_queue.get()
            else:
                return None
        except Exception as e:
            print(f"Error getting from queue: {e}")
            return None

    def stop_server(self):
        """Stop the server."""
        self.server_running = False
        if self.server_socket:
            self.server_socket.close()
        print("Server stopped")

    def disconnect_from_server(self):
        """Disconnect from the connected server."""
        self.client_connected = False
        if self.client_socket:
            self.client_socket.close()
        print("Disconnected from server")

    def queue_size(self):
        """Return the current size of the message queue."""
        return self.message_queue.qsize()

    def parse_motion_command(self, message):
        """Parse motion command message into rotation, translation, and target components.
        
        Expected format: rotation_x,rotation_y,rotation_z,translation_x,translation_y,translation_z,target
        Where target: 0 = gimbal, 1 = drone
        
        Returns: dict with keys 'rotation', 'translation', 'target' or None if parsing fails
        """
        try:
            # Clean the message (remove newlines and extra whitespace)
            message = message.strip()
            
            # Split by comma
            values = message.split(',')
            
            if len(values) < 7:
                print(f"Error: Expected at least 7 values, got {len(values)}")
                return None
            
            # Parse rotation (first 3 values)
            rotation = {
                'x': float(values[0]),
                'y': float(values[1]),
                'z': float(values[2])
            }
            
            # Parse translation (next 3 values)
            translation = {
                'x': float(values[3]),
                'y': float(values[4]),
                'z': float(values[5])
            }
            
            # Parse target flag (last value)
            target_flag = int(values[6])
            target = 'drone' if target_flag == 1 else 'gimbal'
            
            result = {
                'rotation': rotation,
                'translation': translation,
                'target': target,
                'target_flag': target_flag
            }
            
            return result
        except (ValueError, IndexError) as e:
            print(f"Error parsing motion command: {e}")
            return None
    
class MsgHandler:
    def __init__(self):
        self.server_ip = '10.116.88.38'
        self.client_ip = "127.0.0.1"
        self.server_port = 9001
        self.client_port = 9010
        self.threads = []  # Track all threads
        # Start a server
        self.msg_handler = ReceiveMsg()
        handler_thread = threading.Thread(target=self.__start_handler, daemon=False)
        self.threads.append(handler_thread)
        handler_thread.start()

    def __start_handler(self):
        

        # Connect as client to another server
        self.msg_handler.connect_to_server(self.server_ip, self.server_port)

        # Start server
        self.msg_handler.start_server(port=self.client_port, host=self.client_ip)
        while True:
            message = self.msg_handler.get_from_queue()  # Get and remove
            if message:
                self.msg_handler.send_to_server(message)
    

    def get_msg(self):
        # Send messages and dequeue
        msg = self.msg_handler.print_from_queue()  # View next message
        if msg is not None:
            result = self.msg_handler.parse_motion_command(msg)
            return result
        else:
            return None

    def wait_for_threads(self):
        """Wait for all threads to complete before exiting."""
        for thread in self.threads:
            if thread.is_alive():
                thread.join()
        print("All threads completed")

if __name__ == '__main__':
    msg_handler = MsgHandler()
    result = msg_handler.get_msg()
    print("Result: ", result)
    msg_handler.wait_for_threads()  # Wait for all threads before exiting
    


    

    
    
    
    