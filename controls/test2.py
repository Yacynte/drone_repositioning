import asyncio
from pynput import keyboard

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
            print(f"DEBUG: Added {k}. Set now: {state.keys}")
        else:
            state.keys.discard(k)

async def send_commands():
    # Use state.keys here
    print(f"DEBUG: Starting loop, current set: {state.keys}")
    
    while True:
        if 'n' in state.keys:
            print("Detected 'n' inside the loop!")
            # ... your logic ...
            state.keys.discard('n')
        
        await asyncio.sleep(0.1)

# Main block
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