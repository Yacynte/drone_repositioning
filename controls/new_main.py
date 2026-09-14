import time
import asyncio
from pynput import keyboard
from controls import RTSPController
import random

# Use a container class to ensure we are modifying the same object
class KeyState:
    def __init__(self):
        self.keys = set()

# Initialize the shared state
state = KeyState()

def update_key_state(key, pressed):
    k = None
    if hasattr(key, 'char') and key.char is not None:
        k = key.char.lower()
    elif hasattr(key, 'name'):
        k = key.name
    
    if k:
        if pressed:
            # Update the shared set
            state.keys.add(k)
            # print(f"DEBUG: Added {k}. Set now: {state.keys}")
        else:
            state.keys.discard(k)


def jitter_location(target, **ranges):
    """
    ranges: keyword args like x=500, y=500, z=200, roll=2, pitch=5, yaw=10
    Only keys you pass in get jittered; others stay unchanged.
    """
    result = dict(target)
    for key, r in ranges.items():
        if key == 'z':
            val = random.randint(50, r)
        elif key == 'rel_yaw' or key == 'rel_roll' or key == 'rel_pitch':
            val = random.randint(1, r)
        else:
            val = random.randint(100, r)
        val_new = val if random.choice([True, False]) else -val
        result[key] = target[key] + val_new
    return result


# ip_drone="10.116.88.38", port_drone=9001

async def send_commands():

    ctrl = RTSPController(ip_drone="127.0.0.1", port_drone=9001, 
                        ip_controller="172.28.243.111", port_controller=9020)

    # target_locations = [#{"x": -12000.0, "y": -5000.0, "z": 100.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},      # Central Sunken Plaza          /no
    #                     {"x": 35890.0, "y": 26720.0, "z": 219.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},       # Tree-Lined Promenade          /bo
    #                     {"x": -55000.0, "y": 35000.0, "z": 700.0, "roll": 0.0, "pitch": -20.0, "yaw": -15.0},     # High-Rise Planter Terraces    /zes
    #                     {"x": -31000.0, "y": 26000.0, "z": 850.0, "roll": 0.0, "pitch": -20.0, "yaw": -225.0},     # The Highway / Freeway Zone    /no
    #                     {"x": 18000.0, "y": -18000.0, "z": 1000.0, "roll": 0.0, "pitch": -20.0, "yaw": -180.0},    # High-Rise Rooftop View        /zes too high
    #                     {"x": -21000.0, "y": -21000.0, "z": 950.0, "roll": 0.0, "pitch": -20.0, "yaw": -90.0}]       # Pedestrian / Mass AI Plaza    /zes angle adjust

    target_locations = [{"x": 356523.0, "y": -178343.0, "z": -178000.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "rel_roll": 0.0, "rel_pitch": -10.0, "rel_yaw": 30.0},      # Central Sunken Plaza          /no
                        {"x": 240383.0, "y": -195983.0, "z": -178000.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "rel_roll": 0.0, "rel_pitch": -5.0, "rel_yaw": 0.0},       # Tree-Lined Promenade          /bo
                        # {"x": -327033.0, "y": 7407.0, "z": -182000.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "rel_roll": 0.0, "rel_pitch": -15.0, "rel_yaw": 0.0},     # High-Rise Planter Terraces    /zes
                        #{"x": -35000.0, "y": 25000.0, "z": 1500.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "rel_roll": 0.0, "rel_pitch": 0.0, "rel_yaw": 0.0},     # The Highway / Freeway Zone    /no
                        {"x": 356523.0, "y": 3547.0, "z": -178000.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "rel_roll": 0.0, "rel_pitch": -20.0, "rel_yaw": 0.0},    # High-Rise Rooftop View        /zes too high
                        {"x": 330023.0, "y": -80363.0, "z": -175000.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "rel_roll": 0.0, "rel_pitch": -10.0, "rel_yaw": 10.0},
                        {"x": -330023.0, "y": 35277.0, "z": -178000.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "rel_roll": 0.0, "rel_pitch": 0.0, "rel_yaw": 0.0}]
    
    # target_locations = [#{"x": -100.0, "y": -5000.0, "z": 100.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},      # Central Sunken Plaza          /no
    #                     #{"x": 35890.0, "y": 26720.0, "z": 219.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}]       # Tree-Lined Promenade          /bo
    #                     {"x": -22000.0, "y": 15000.0, "z": 4500.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "rel_roll": 0.0, "rel_pitch": -20.0, "rel_yaw": 0.0},     # High-Rise Planter Terraces    /zes
    #                     #{"x": -35000.0, "y": 25000.0, "z": 1500.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},     # The Highway / Freeway Zone    /no
    #                     {"x": 10000.0, "y": -15000.0, "z": 1500.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "rel_roll": 0.0, "rel_pitch": -20.0, "rel_yaw": 0.0},    # High-Rise Rooftop View        /zes too high
    #                     {"x": -5000.0, "y": -9000.0, "z": 1500.0, "roll": 0.0, "pitch": 0.0, "yaw": -60.0, "rel_roll": 0.0, "rel_pitch": -20.0, "rel_yaw": -30.0},#]       # Pedestrian / Mass AI Plaza    /zes angle adjust
    #                     # {"x": -32000.0, "y": 25000.0, "z": 4500.0, "roll": 0.0, "pitch": -20.0, "yaw": -45.0}]
    #                     {"x": -55000.0, "y": 35000.0, "z": 1500.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "rel_roll": 0.0, "rel_pitch": -20.0, "rel_yaw": 10.0},     # High-Rise Planter Terraces    /zes
    #                     {"x": -31000.0, "y": 25000.0, "z": 1500.0, "roll": 0.0, "pitch": 0.0, "yaw": -200.0, "rel_roll": 0.0, "rel_pitch": -20.0, "rel_yaw": -25.0},     # The Highway / Freeway Zone    /no
    #                     {"x": 17000.0, "y": -18000.0, "z": 1000.0, "roll": 0.0, "pitch": 0.0, "yaw": -150.0, "rel_roll": 0.0, "rel_pitch": -20.0, "rel_yaw": -25.0},    # High-Rise Rooftop View        /zes too high
    #                     {"x": -21000.0, "y": -21000.0, "z": 1500.0, "roll": 0.0, "pitch": 0.0, "yaw": -60.0, "rel_roll": 0.0, "rel_pitch": -20.0, "rel_yaw": -30.0}]
    
    ctrl.start_stream(True)
    # ctrl.start_receiving_controls()

    # ctrl.send_command("rotation_only")
    # ctrl.start_controller()

    # for target in target_locations:
    # ctrl.arrived_target = False  # Reset the flag for each new target
    # ctrl.set_location(target["x"], target["y"], target["z"], target="drone")
    target_locations_iter = iter(target_locations)
    # print(f"First location: {target_locations[0]}")
    # print(f"Current keys status: {state.keys}")
    arrived = False
    start = True
    SENTINEL = object()
    while not arrived:
        if 'n' in state.keys:
            params = next(target_locations_iter, SENTINEL)
            if params is SENTINEL:
                print("All target locations have been sent!")
                arrived = True  # or break, or whatever exit logic you want
            else:
                # print(f"current params {params}")
                # noisy_target = jitter_location(params, x=150, y=100, z=200, rel_yaw=20, rel_pitch=10)
                ctrl.set_location(**params, pose="rotationToo")
            await asyncio.sleep(3)  
            if start:
                # ctrl.send_command("rotation_only")
                ctrl.start_controller()
                start = False
            # params = next(target_locations_iter)
            # print(f"current params {params}")
            # ctrl.set_location(**params)
            # print("Sent next location")
            # CRITICAL: Remove 'n' from keys so it doesn't trigger again 
            # until the key is physically released and pressed again
            state.keys.discard('n')

        # while not ctrl.arrived_target:
        #     # Send a command to the controller script
        #     # ctrl._send_command("move_forward")
            
        # Wait for a short duration before sending the next command
        # print("In while loop")
        # time.sleep(0.01)
        await asyncio.sleep(0.01)
    ctrl.set_location_relative(target="final")
    await asyncio.sleep(0.1)
    ctrl.send_command("stop")  # Stop the controller after reaching the target




if __name__ == "__main__":
    loop = asyncio.new_event_loop()
    # ... setup listeners with state.keys ...
    # Pass the actual state object's method to the listener
    listener = keyboard.Listener(
        on_press=lambda k: loop.call_soon_threadsafe(update_key_state, k, True),
        on_release=lambda k: loop.call_soon_threadsafe(update_key_state, k, False)
    )
    listener.start()
    loop.run_until_complete(send_commands())