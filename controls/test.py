import asyncio
import struct
import time
from pynput import keyboard

# Global thread-safe state
keys = set()
last_space_state = False
drone_ip = '192.168.178.54' 
drone_port = 8888 

def update_key_state(key, pressed):
    """Safely updates keys in the main thread event loop."""
    try:
        if hasattr(key, 'char') and key.char is not None:
            k = key.char.lower()
        elif key == keyboard.Key.space:
            k = 'space'
        else:
            return
            
        if pressed:
            keys.add(k)
        else:
            keys.discard(k)
    except Exception:
        pass

async def send_rc(writer, channels):
    # Header 0x01 + 8 uint16
    data = struct.pack('<B8H', 0x01, *channels)
    writer.write(data)
    await writer.drain()

async def request_status(writer):
    writer.write(struct.pack('<B', 0x02))
    await writer.drain()

async def send_stop(writer):
    writer.write(struct.pack('<B', 0x03))
    await writer.drain()

async def send_commands(host, port):
    # Flight variables
    roll, pitch, yaw = 1500, 1500, 1500
    throttle = 950
    target_throttle = 950
    takeoff_mode = False
    aux1_arm = 1000
    angle = 1500
    ramp_speed = 5
    is_armed = False
    
    global last_space_state

    try:
        reader, writer = await asyncio.open_connection(host, port)
        print(f"Connected to {host}:{port}")
    except Exception as e:
        print(f"Connection failed: {e}")
        return

    # Safety initialization
    channels = [roll, pitch, throttle, yaw, aux1_arm, angle, 1000, 1000]
    await send_rc(writer, channels)
    print("Ready to send commands. Use WASD for pitch/roll, QE for yaw, RF for throttle, SPACE to arm/disarm.")

    try:
        last_status_time = 0.0
        while True:
            # 1. Non-blocking ARM/DISARM Toggle with Safety Reset
            current_space = 'space' in keys
            if current_space and not last_space_state:
                is_armed = not is_armed
                aux1_arm = 1400 if is_armed else 1000
                
                # CRITICAL SAFETY: Reset throttle to idle if disarming
                if not is_armed:
                    throttle = 950
                    takeoff_mode = False
                
                print(f"--- {'ARMED' if is_armed else 'DISARMED'} ---")
            last_space_state = current_space

            # 2. Flight Logic (Centered inputs)
            pitch = 1600 if 'w' in keys else (1400 if 's' in keys else 1500)
            roll  = 1400 if 'a' in keys else (1600 if 'd' in keys else 1500)
            yaw   = 1400 if 'q' in keys else (1600 if 'e' in keys else 1500)

            # 3. Manual Throttle Logic (Only adjusts if armed)
            if is_armed:
                if 'r' in keys: throttle = min(2000, throttle + 10)
                if 'f' in keys: throttle = max(1000, throttle - 10)

            # 4. Takeoff/Landing Logic
            if 't' in keys and is_armed:
                takeoff_mode = True
                target_throttle = 1350  
            elif 'g' in keys:
                takeoff_mode = False
                target_throttle = 950 if not is_armed else 1000 

            if takeoff_mode and is_armed:
                if throttle < target_throttle:
                    throttle = min(target_throttle, throttle + ramp_speed)
                elif throttle > target_throttle:
                    throttle = max(target_throttle, throttle - ramp_speed)

            # 5. Status polling rate tracking (Uses monotonic floats to prevent multi-firing)
            current_time = time.monotonic()
            if 'l' in keys and (current_time - last_status_time > 0.5):
                await request_status(writer)
                last_status_time = current_time

            # Emergency Stop
            if 'x' in keys:
                print("Emergency Stop Triggered!")
                await send_stop(writer)
                break

            # 6. Pack and Send at ~50Hz
            channels = [roll, pitch, int(throttle), yaw, aux1_arm, angle, 1000, 1000]
            await send_rc(writer, channels)
            
            await asyncio.sleep(0.02) 

    except Exception as e:
        print(f"\nSender Error: {e}")
    finally:
        print("Closing connection...")
        writer.close()
        await writer.wait_closed()

if __name__ == "__main__":
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    # Thread-safe integration with pynput using loop.call_soon_threadsafe
    def pressed_callback(key):
        loop.call_soon_threadsafe(update_key_state, key, True)

    def released_callback(key):
        loop.call_soon_threadsafe(update_key_state, key, False)

    listener = keyboard.Listener(on_press=pressed_callback, on_release=released_callback)
    listener.start()
    
    try:
        loop.run_until_complete(send_commands(drone_ip, drone_port))
    except KeyboardInterrupt:
        print("\nExiting cleanly...")
    finally:
        loop.close()