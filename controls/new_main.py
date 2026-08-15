import time
import asyncio
from pynput import keyboard
from controls import RTSPController
import subprocess
import os
import glob

# Use a container class to ensure we are modifying the same object
class KeyState:
    def __init__(self):
        self.keys = set()

# Initialize the shared state
state = KeyState()



# ip_drone = "10.116.88.38"
ip_drone = "172.28.240.1"
port_drone = 9001
# ip_controller = "172.28.243.111"
ip_controller = "127.0.0.1"
port_controller=9020
script_dir = os.path.dirname(os.path.abspath(__file__))
binary_dir = os.path.join(os.getcwd(), "build")
binary_path = os.path.join(binary_dir, "ImageMatcher")
image_folder = os.path.join(script_dir, "imagesGT")
image_paths = sorted(glob.glob(os.path.join(image_folder, "*.png")))
print("binary path: ", binary_path )
print(f"Image folder: {image_folder}")


def update_key_state(key, pressed):
    print(f"CALLBACK FIRED: key={key}, pressed={pressed}")
    k = None
    if hasattr(key, 'char') and key.char is not None:
        k = key.char.lower()
    elif hasattr(key, 'name'):
        k = key.name
    
    if k:
        if pressed:
            # Update the shared set
            state.keys.add(k)
            print(f"DEBUG: Added {k}. Set now: {state.keys}")
        else:
            state.keys.discard(k)


async def send_commands():

    ctrl = RTSPController(ip_drone=ip_drone, port_drone=port_drone, 
                        ip_controller=ip_controller, port_controller=port_controller)

    target_locations = [#{"x": -12000.0, "y": -5000.0, "z": 100.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},      # Central Sunken Plaza          /no
                        #{"x": 8500.0, "y": 12000.0, "z": 1000.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},       # Tree-Lined Promenade          /bo
                        {"x": -22000.0, "y": 15000.0, "z": 4500.0, "roll": 0.0, "pitch": -20.0, "yaw": 0.0},     # High-Rise Planter Terraces    /zes
                        #{"x": -35000.0, "y": 25000.0, "z": 1500.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},     # The Highway / Freeway Zone    /no
                        {"x": 10000.0, "y": -15000.0, "z": 1500.0, "roll": 0.0, "pitch": -20.0, "yaw": 0.0},    # High-Rise Rooftop View        /zes too high
                        {"x": -5000.0, "y": -8000.0, "z": 1500.0, "roll": 0.0, "pitch": -20.0, "yaw": -90.0}]       # Pedestrian / Mass AI Plaza    /zes angle adjust

    # ctrl.start_stream(True)
    # ctrl.start_receiving_controls()

    # # ctrl.send_command("rotation_only")
    # ctrl.start_controller()

    # for target in target_locations:
    # ctrl.arrived_target = False  # Reset the flag for each new target
    # ctrl.set_location(target["x"], target["y"], target["z"], target="drone")
    target_locations_iter = iter(target_locations)
    target_images_iter = iter(image_paths[1:])
    print(f"First location: {target_locations[0]}")
    print(f"First image: {image_paths[1]}")
    # print(f"Current keys status: {state.keys}")
    arrived = False
    SENTINEL = object()
    while not arrived:
        if 'n' in state.keys or ctrl.arrived_target:

            params = next(target_locations_iter, SENTINEL)
            if params is SENTINEL:
                print("All target locations have been sent!")
                arrived = True  # or break, or whatever exit logic you want
            else:
                # print(f"current params {params}")
                ctrl.set_location(**params, pose="rotationToo")
            await asyncio.sleep(2)
            image = next(target_images_iter, SENTINEL)
            if image is SENTINEL:
                print("Out of target images!")
                arrived = True
                continue
            # ./ImageMatcher --unreal 1 --target "../targets/Capture_005.png" --imgHeight 1080 --imgWidth 1920 --rtsp "tcp://10.116.88.38:9000"
            proc = subprocess.Popen([binary_path, "--imgWidth", "1920", "--imgHeight", "1080", "--unreal", "1", 
                                     "--target", image, "--rtsp", "tcp://10.116.88.38:9000"],
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL )
            await asyncio.sleep(1)
            ctrl.start_stream(True)
            ctrl.start_receiving_controls()
            ctrl.start_controller()
            # CRITICAL: Remove 'n' from keys so it doesn't trigger again 
            # until the key is physically released and pressed again
            state.keys.discard('n')
            # ctrl.arrived_target = False
            # ctrl.is_receiving = False

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
    print("Listener running:", listener.running)
    print("Listener is alive:", listener.is_alive())
    loop.run_until_complete(send_commands())