import time
import numpy as np
import cv2
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

# ── Load engine ──────────────────────────────────────────────────────────────
logger = trt.Logger(trt.Logger.WARNING)
with open("weights/superpoint_fp16.trt", "rb") as f:
    engine = trt.Runtime(logger).deserialize_cuda_engine(f.read())

context = engine.create_execution_context()

# ── Prepare input first ───────────────────────────────────────────────────────
WIDTH = 1920
HEIGHT = 1080
MAX_KP = 2048

img    = cv2.imread("target_lab2.jpg", cv2.IMREAD_GRAYSCALE)
img    = cv2.resize(img, (WIDTH, HEIGHT))
tensor = img.astype(np.float32) / 255.0
tensor = np.ascontiguousarray(tensor[None, None])  # (1, 1, H, W)

# set input shape so TRT knows output shapes
input_name = engine.get_tensor_name(0)
context.set_input_shape(input_name, tensor.shape)

# ── Allocate buffers AFTER setting input shape ────────────────────────────────
bindings = {}
for i in range(engine.num_io_tensors):
    name  = engine.get_tensor_name(i)
    shape = list(context.get_tensor_shape(name))   # now concrete
    dtype = trt.nptype(engine.get_tensor_dtype(name))

    shape = [MAX_KP if d == -1 else d for d in shape]
    print(f"  {name}: {shape} {dtype}")

    host_mem   = np.empty(shape, dtype=dtype)
    device_mem = cuda.mem_alloc(host_mem.nbytes)
    bindings[name] = (host_mem, device_mem)
    context.set_tensor_address(name, int(device_mem))

stream = cuda.Stream()

# copy input to GPU
cuda.memcpy_htod_async(bindings[input_name][1], tensor, stream)

# ── Warmup ────────────────────────────────────────────────────────────────────
for _ in range(10):
    context.execute_async_v3(stream.handle)
    stream.synchronize()

# ── Benchmark ─────────────────────────────────────────────────────────────────
times = []
for _ in range(100):
    t0 = time.perf_counter()
    context.execute_async_v3(stream.handle)
    stream.synchronize()
    times.append(time.perf_counter() - t0)

print(f"\nmean:   {np.mean(times)*1000:.2f} ms")
print(f"median: {np.median(times)*1000:.2f} ms")
print(f"FPS:    {1/np.mean(times):.1f}")

# ── Read outputs ──────────────────────────────────────────────────────────────
for name in list(bindings.keys())[1:]:   # skip input
    host, device = bindings[name]
    cuda.memcpy_dtoh(host, device)
    print(f"{name}: {host.shape} | sample: {host.flat[:3]}")