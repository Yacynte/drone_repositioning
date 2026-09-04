import cv2
import time

# Open with V4L2 backend
cap = cv2.VideoCapture("/dev/v4l/by-id/usb-UltraSemi_USB3_Video_20210623-video-index0", cv2.CAP_V4L2)

# Set buffer size to 1 to prevent queue lag
# cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

# Try a safer resolution/format combo first
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
if not cap.isOpened():
    print("Error: Could not open video device 0.")
else:
    # print("Camera opened successfully. Reading test frame...")
    print('Camera opened successfully. Warming up sensor...')
    # Give the driver time to initialize
    time.sleep(1.0)
    # Flush out initial empty/black frames
    for i in range(10):
        ret, frame = cap.read()

    ret, frame = cap.read()
    if ret:
        print(f"Frame captured successfully! Shape: {frame.shape}")
        cv2.imwrite("test.png", frame)
    else:
        print("Failed to grab frame. Check if another process is using /dev/video0.")

for i in range(1):
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break
        
    # Save using a standard image format extension like .jpg or .png
    cv2.imwrite(f"test_imgs/frame_{i:03d}.png", frame)
    print("Saved image ", i)

cap.release()