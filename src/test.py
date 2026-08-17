import cv2

# Open with V4L2 backend
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

# Set buffer size to 1 to prevent queue lag
# cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

# Try a safer resolution/format combo first
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
if not cap.isOpened():
    print("Error: Could not open video device 0.")
else:
    print("Camera opened successfully. Reading test frame...")
    ret, frame = cap.read()
    if ret:
        print(f"Frame captured successfully! Shape: {frame.shape}")
    else:
        print("Failed to grab frame. Check if another process is using /dev/video0.")

cap.release()