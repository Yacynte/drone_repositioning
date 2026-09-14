import time
from controls import RTSPController

# ip_drone="10.116.88.38", port_drone=9001

ctrl = RTSPController(ip_drone="127.0.0.1", port_drone=9001, 
                      ip_controller="172.28.243.111", port_controller=9020)

# target_locations = [{"x": -12000.0, "y": -5000.0, "z": 100.0},      # Central Sunken Plaza
#                     {"x": 8500.0, "y": 12000.0, "z": 1000.0},       # Tree-Lined Promenade
#                     {"x": -22000.0, "y": 15000.0, "z": 4500.0},     # High-Rise Planter Terraces
#                     {"x": -35000.0, "y": 25000.0, "z": 1500.0},     # The Highway / Freeway Zone
#                     {"x": 10000.0, "y": -15000.0, "z": 15000.0},    # High-Rise Rooftop View
#                     {"x": -5000.0, "y": -8000.0, "z": 500.0}]       # 


target_locations = [{"x": -12000.0, "y": -5000.0, "z": 1000.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},      # Central Sunken Plaza          /no
                        {"x": 8500.0, "y": 12000.0, "z": 1000.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},       # Tree-Lined Promenade          /bo
                        #{"x": -22000.0, "y": 15000.0, "z": 4500.0, "roll": 0.0, "pitch": -20.0, "yaw": 0.0},     # High-Rise Planter Terraces    /zes
                        {"x": -35000.0, "y": 25000.0, "z": 1500.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}]#,     # The Highway / Freeway Zone    /no
                        #{"x": 10000.0, "y": -15000.0, "z": 1500.0, "roll": 0.0, "pitch": -20.0, "yaw": 0.0},    # High-Rise Rooftop View        /zes too high
                        #{"x": -5000.0, "y": -8000.0, "z": 1500.0, "roll": 0.0, "pitch": -20.0, "yaw": -90.0}]       # Pedestrian / Mass AI Plaza    /zes angle adjust


ctrl.start_stream(True)
ctrl.start_receiving_controls()

# ctrl.send_command("rotation_only")
ctrl.start_controller()

# for target in target_locations:
ctrl.arrived_target = False  # Reset the flag for each new target
# ctrl.set_location(target["x"], target["y"], target["z"], target="drone")
while not ctrl.arrived_target:
    # Send a command to the controller script
    # ctrl._send_command("move_forward")
    
    # Wait for a short duration before sending the next command
    time.sleep(1)
ctrl.send_command("stop")  # Stop the controller after reaching the target
