import cv2
import time

DEVICE = 1
DURATION = 10

cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc("M", "J", "P", "G"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    raise RuntimeError("Could not open camera")

print("Camera opened")
print("Configured:",
      int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
      "x",
      int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
      "@",
      cap.get(cv2.CAP_PROP_FPS),
      "FPS")

frames = 0
start = time.perf_counter()

while time.perf_counter() - start < DURATION:
    ret, frame = cap.read()

    if not ret:
        print("Frame read failed")
        continue

    frames += 1

elapsed = time.perf_counter() - start
cap.release()

print()
print(f"Frames:  {frames}")
print(f"Time:    {elapsed:.3f} s")
print(f"Actual FPS: {frames / elapsed:.2f}")