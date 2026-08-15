import socket
import json
import threading

class RTSPController:
    def __init__(self, ip_drone="127.0.0.1", port_drone=9001, ip_controller="127.0.0.1", port_controller=9020):
        self.address_drone = (ip_drone, port_drone)
        self.sock_drone = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.address_controller = (ip_controller, port_controller)
        self.sock_controller = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.arrived_target = False
        self.is_receiving = False
        self.thread = None

    def _send(self, command, target, params):
        data = {"command": command, "target": target, "params": params}
        message = json.dumps(data).encode('utf-8')
        print(f"Sending to drone: {data}")
        self.sock_drone.sendto(message, self.address_drone)

    def _send_command(self, command: str):
        """Send a command to the controller script."""
        print(f"Sending command to controller: {command}")
        message = command + "\n"  # Add newline to indicate end of command
        self.sock_controller.sendto(message.encode('utf-8'), self.address_controller)

    def _receive_loop(self):
        """The background loop that runs until is_receiving is set to False."""
        while self.is_receiving:
            try:
                # We use settimeout so the loop doesn't block indefinitely
                self.sock_controller.settimeout(0.5) 
                data, _ = self.sock_controller.recvfrom(1024)
                # Clean the message (remove newlines and extra whitespace)
                message = data.decode('utf-8').strip()
                print(f"Received from controller: {message}")
                # Split by comma
                values = message.split(',')
                target = "gimbal"
                if int(values[6]) == -1:
                    # self.arrived_target = True arrived
                    target = "arrived"
                if int(values[6]) == 1:
                    target = "drone"
                params = {"roll": float(values[0]), "pitch": float(values[1]), "yaw": float(values[2]), "x": float(values[3]), "y": float(values[4]), "z": float(values[5])}
                self.set_location_relative(**params, target=target)
                if int(values[6]) == -1:
                    self.arrived_target = True
                    
            except socket.timeout:
                continue
            except Exception as e:
                if self.is_receiving:
                    print(f"Receiver error: {e}")

    def start_receiving_controls(self):
        if not self.is_receiving:
            self.is_receiving = True
            self.thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.thread.start()
            print("Background receiver started.")

    def stop_receiving_controls(self):
        self.is_receiving = False
        if self.thread:
            self.thread.join()
            print("Background receiver stopped.")

    def set_location(self, x: float=0.0, y: float=0.0, z: float=0.0, yaw: float=0.0, pitch: float=0.0, roll: float=0.0, 
                     rel_yaw: float=0.0, rel_pitch: float=0.0, rel_roll: float=0.0, target="drone", pose = "transOnly"):
        params = {"x": x, "y": y, "z": z, "roll": roll, "pitch": pitch, "yaw": yaw, 
                  "rel_yaw": rel_yaw, "rel_pitch": rel_pitch, "rel_roll": rel_roll, "pose": pose}
        self._send("location", target, params)

    def set_location_relative(self, x: float=0.0, y: float=0.0, z: float=0.0, roll: float=0.0, 
                              pitch: float=0.0, yaw: float=0.0, target="drone"):
        params = {"x": x, "y": y, "z": z, "roll": roll, "pitch": pitch, "yaw": yaw}
        self._send("setPose", target, params)

    def set_rain(self, active: bool, intensity: float = 0.0, target="drone"):
        params = {"value": active, "intensity": intensity}
        self._send("rain", target, params)

    def set_snow(self, active: bool, intensity: float = 0.0, target="drone"):
        params = {"value": active, "intensity": intensity}
        self._send("snow", target, params)

    def start_stream(self, active: bool, target="drone"):
        params = {"value": active}
        self._send("startStream", target, params)

    def start_controller(self):
        """Send a command to the controller script."""
        self._send_command("start")

    def stop_controller(self):
        """Send a command to the controller script."""
        self._send_command("stop")
    
    def send_command(self, command: str):
        """Send a command to the controller script."""
        self._send_command(command)