import time
import cv2
import numpy as np
import onnxruntime as ort

session = ort.InferenceSession(
    "weights/superpoint.onnx",
    providers=["CUDAExecutionProvider", "CPUExecutionProvider"]
)

print(session.get_providers())

img = cv2.imread("target_lab2.jpg", cv2.IMREAD_GRAYSCALE)
img = cv2.resize(img, (640, 480))
img = img.astype(np.float32) / 255.0
tensor = img[None, None]

# Warmup
for _ in range(5):
    session.run(None, {"image": tensor})

times = []

for _ in range(20):
    t0 = time.perf_counter()
    session.run(None, {"image": tensor})
    times.append(time.perf_counter() - t0)

print("Average:", np.mean(times) * 1000, "ms")
print("FPS:", 1 / np.mean(times))