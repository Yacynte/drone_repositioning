import time
import asyncio
from pynput import keyboard
from controls import RTSPController
import subprocess
import os
import glob
import random

# Use a container class to ensure we are modifying the same object
class KeyState:
    def __init__(self):
        self.keys = set()

# Initialize the shared state
state = KeyState()



# ip_drone = "10.116.88.38"
# ip_drone = "172.28.240.1"
ip_drone = "127.0.0.1"
port_drone = 9001
ip_controller = "172.28.243.111"
# ip_controller = "127.0.0.1"
port_controller=9020 
home_dir = "/home/user/drone_repositioning"
binary_path =  f"{home_dir}/build/ImageMatcher"
logs_path = f"{home_dir}/controls/logData4/"
# wsl_image_folder = f"{home_dir}/controls/imagesGT4"
wsl_image_folder = f"{home_dir}/target_sim"
onnx_path = f"{home_dir}/src_py/matches_onnx.py"
venv = f"{home_dir}/.venv/bin/python"
script_dir_ = os.path.dirname(os.path.abspath(__file__))
script_dir = os.path.join(os.getcwd(), "data")
# binary_path = os.path.join(binary_dir, "ImageMatcher")
image_folder = os.path.join(script_dir, "imagesGT4")
# image_paths = sorted(glob.glob(os.path.join(image_folder, "*.png")))
image_filenames  = sorted(os.path.basename(p) for p in glob.glob(os.path.join(image_folder, "*.png")))
# then build the WSL path per image:
# wsl_image_path = f"{wsl_image_folder}/{image_filenames_iter_value}"
print("binary path: ", binary_path )
print(f"Image folder: {image_folder}")
# for filename in image_filenames:
#     wsl_image_path = f"{wsl_image_folder}/{filename}"


def update_key_state(key, pressed):
    # print(f"CALLBACK FIRED: key={key}, pressed={pressed}")
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
        # if key == 'z':
        #     val = random.randint(50, r)
        if key == 'rel_yaw' or key == 'rel_roll' or key == 'rel_pitch':
            val = random.randint(5, r)
        else:
            val = random.randint(100, r)
        val_new = val if random.choice([True, False]) else -val
        result[key] = target[key] + val_new
    return result

async def send_commands():

    ctrl = RTSPController(ip_drone=ip_drone, port_drone=port_drone, 
                        ip_controller=ip_controller, port_controller=port_controller)

    # target_locations = [#{"x": -12000.0, "y": -5000.0, "z": 100.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},      # Central Sunken Plaza          /no
    #                     #{"x": 8500.0, "y": 12000.0, "z": 1000.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},       # Tree-Lined Promenade          /bo
    #                     {"x": -22000.0, "y": 15000.0, "z": 4500.0, "roll": 0.0, "pitch": -20.0, "yaw": 0.0},     # High-Rise Planter Terraces    /zes
    #                     #{"x": -35000.0, "y": 25000.0, "z": 1500.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},     # The Highway / Freeway Zone    /no
    #                     {"x": 10000.0, "y": -15000.0, "z": 1500.0, "roll": 0.0, "pitch": -20.0, "yaw": 0.0},    # High-Rise Rooftop View        /zes too high
    #                     {"x": -5000.0, "y": -8000.0, "z": 1500.0, "roll": 0.0, "pitch": -20.0, "yaw": -90.0},#]       # Pedestrian / Mass AI Plaza    /zes angle adjust
    #                     # {"x": -32000.0, "y": 25000.0, "z": 4500.0, "roll": 0.0, "pitch": -20.0, "yaw": -45.0}]
    #                     {"x": -55000.0, "y": 35000.0, "z": 1500.0, "roll": 0.0, "pitch": -20.0, "yaw": -15.0},     # High-Rise Planter Terraces    /zes
    #                     {"x": -31000.0, "y": 26000.0, "z": 1500.0, "roll": 0.0, "pitch": -20.0, "yaw": -225.0},     # The Highway / Freeway Zone    /no
    #                     {"x": 18000.0, "y": -18000.0, "z": 1000.0, "roll": 0.0, "pitch": -20.0, "yaw": -180.0},    # High-Rise Rooftop View        /zes too high
    #                     {"x": -21000.0, "y": -21000.0, "z": 1500.0, "roll": 0.0, "pitch": -20.0, "yaw": -90.0}]
    
    
    target_locations = [{"x": 356523.0, "y": -178343.0, "z": -178000.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "rel_roll": 0.0, "rel_pitch": -10.0, "rel_yaw": 30.0},      # Central Sunken Plaza          /no
                            {"x": 240383.0, "y": -195983.0, "z": -178000.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "rel_roll": 0.0, "rel_pitch": -5.0, "rel_yaw": 0.0},       # Tree-Lined Promenade          /bo
                            # {"x": -327033.0, "y": 7407.0, "z": -182000.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "rel_roll": 0.0, "rel_pitch": -15.0, "rel_yaw": 0.0},     # High-Rise Planter Terraces    /zes
                            #{"x": -35000.0, "y": 25000.0, "z": 1500.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "rel_roll": 0.0, "rel_pitch": 0.0, "rel_yaw": 0.0},     # The Highway / Freeway Zone    /no
                            {"x": 356523.0, "y": 3547.0, "z": -178000.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "rel_roll": 0.0, "rel_pitch": -20.0, "rel_yaw": 0.0},    # High-Rise Rooftop View        /zes too high
                            {"x": 330023.0, "y": -80363.0, "z": -175000.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "rel_roll": 0.0, "rel_pitch": -10.0, "rel_yaw": 10.0},
                            {"x": -330023.0, "y": 35277.0, "z": -178000.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "rel_roll": 0.0, "rel_pitch": 0.0, "rel_yaw": 0.0}]
    
    
    # ctrl.start_stream(True)
    # ctrl.start_receiving_controls()

    # # ctrl.send_command("rotation_only")
    # ctrl.start_controller()

    # for target in target_locations:
    # ctrl.arrived_target = False  # Reset the flag for each new target
    # ctrl.set_location(target["x"], target["y"], target["z"], target="drone")
    target_locations_iter = iter(target_locations)
    target_images_iter = iter(image_filenames)
    print(f"First location: {target_locations[0]}")
    print(f"First image: {image_filenames[0]}")
    # print(f"Current keys status: {state.keys}")
    arrived = False
    SENTINEL = object()
    ctrl.start_stream(True)
    # image = next(target_images_iter, SENTINEL)
    # wsl_image_path = f"{wsl_image_folder}/{image}"
    # print(f"wsl image 1: {wsl_image_path}")
    # proc = subprocess.Popen(["wsl", binary_path, "--imgWidth", "1920", "--imgHeight", "1080", "--unreal", "1", 
    #                                  "--target", wsl_image_path, "--rtsp", "tcp://10.116.88.38:9000"],
    #                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL )
    # await asyncio.sleep(5)
    # ctrl.start_receiving_controls()
    # ctrl.start_controller()

    # result = subprocess.run(["wsl", binary_path, "--imgWidth", "1920", "--imgHeight", "1080", "--unreal", "1", 
    #                          "--target", wsl_image_path, "--rtsp", "tcp://10.116.88.38:9000"],
    #                             capture_output=True,text=True
    #                         )
    # print(result.stdout)
    # print(result.stderr)
    # state.keys.discard('n')
    # ctrl.arrived_target = False
    proc = None
    while not arrived:
        if 's' in state.keys:
            ctrl.stop_controller()
        elif 'n' in state.keys or ctrl.arrived_target:
            if proc is not None:
                # proc.wait()
                proc.terminate()
            ctrl.stop_controller()
            params = next(target_locations_iter, SENTINEL)
            if params is SENTINEL:
                print("All target locations have been sent!")
                arrived = True  # or break, or whatever exit logic you want
                break
            else:
                # print(f"current params {params}")
                noisy_target = jitter_location(params, x=200, y=200, z=200, rel_yaw=20, rel_pitch=10)
                # await asyncio.sleep(5)
                ctrl.set_location(**noisy_target, pose="rotationToo")
            await asyncio.sleep(5)
            image = next(target_images_iter, SENTINEL)
            wsl_image_path = f"{wsl_image_folder}/{image}"
            if image is SENTINEL:
                print("Out of target images!")
                arrived = True
                continue
            # ./ImageMatcher --unreal 1 --target "../targets/Capture_005.png" --imgHeight 1080 --imgWidth 1920 --rtsp "tcp://10.116.88.38:9000"
            proc = subprocess.Popen(["wsl", binary_path, "--imgWidth", "1920", "--imgHeight", "1080", "--unreal", "1", "--onnx_matches", onnx_path,
                                     "--target", wsl_image_path, "--log", logs_path, "--rtsp", "tcp://10.116.88.38:9000"])
                                    # stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL )
            await asyncio.sleep(5)
            
            ctrl.start_receiving_controls()
            ctrl.start_controller()
            # CRITICAL: Remove 'n' from keys so it doesn't trigger again 
            # until the key is physically released and pressed again
            state.keys.discard('n')
            ctrl.arrived_target = False
            
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