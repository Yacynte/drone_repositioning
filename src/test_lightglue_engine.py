import time
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

NUM_KP   = 2048
DESC_DIM = 256

# ── Load engine ───────────────────────────────────────────────────────────────
logger = trt.Logger(trt.Logger.WARNING)
with open("weights/lightglue_fp16.trt", "rb") as f:
    engine = trt.Runtime(logger).deserialize_cuda_engine(f.read())

context = engine.create_execution_context()

# ── Print I/O info ────────────────────────────────────────────────────────────
print("Tensors:")
for i in range(engine.num_io_tensors):
    name  = engine.get_tensor_name(i)
    shape = context.get_tensor_shape(name)
    dtype = engine.get_tensor_dtype(name)
    mode  = "INPUT" if engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT else "OUTPUT"
    print(f"  [{mode}] {name}: {list(shape)} {dtype}")

# ── Allocate buffers ──────────────────────────────────────────────────────────
bindings = {}
for i in range(engine.num_io_tensors):
    name  = engine.get_tensor_name(i)
    shape = list(context.get_tensor_shape(name))
    shape = [NUM_KP if d == -1 else d for d in shape]
    dtype = trt.nptype(engine.get_tensor_dtype(name))

    host_mem   = np.empty(shape, dtype=dtype)
    device_mem = cuda.mem_alloc(host_mem.nbytes)
    bindings[name] = (host_mem, device_mem, shape)
    context.set_tensor_address(name, int(device_mem))

stream = cuda.Stream()

# ── Dummy inputs ──────────────────────────────────────────────────────────────
inputs = {
    'kpts0':   np.random.rand(1, NUM_KP, 2).astype(np.float32),
    'kpts1':   np.random.rand(1, NUM_KP, 2).astype(np.float32),
    'desc0':   np.random.rand(1, NUM_KP, DESC_DIM).astype(np.float32),
    'desc1':   np.random.rand(1, NUM_KP, DESC_DIM).astype(np.float32),
    'scores0': np.random.rand(1, NUM_KP).astype(np.float32),
    'scores1': np.random.rand(1, NUM_KP).astype(np.float32),
}

# copy inputs to GPU
for name, array in inputs.items():
    host, device, _ = bindings[name]
    np.copyto(host, array)
    cuda.memcpy_htod_async(device, host, stream)

stream.synchronize()

# ── Warmup ────────────────────────────────────────────────────────────────────
print("\nWarming up...")
for _ in range(10):
    context.execute_async_v3(stream.handle)
    stream.synchronize()

# ── Benchmark ─────────────────────────────────────────────────────────────────
print("Benchmarking...")
times = []
for _ in range(100):
    t0 = time.perf_counter()
    context.execute_async_v3(stream.handle)
    stream.synchronize()
    times.append(time.perf_counter() - t0)

print(f"\nmean:   {np.mean(times)*1000:.2f} ms")
print(f"median: {np.median(times)*1000:.2f} ms")
print(f"min:    {np.min(times)*1000:.2f} ms")
print(f"max:    {np.max(times)*1000:.2f} ms")
print(f"FPS:    {1/np.mean(times):.1f}")

# ── Read outputs ──────────────────────────────────────────────────────────────
print("\nOutputs:")
for i in range(engine.num_io_tensors):
    name = engine.get_tensor_name(i)
    if engine.get_tensor_mode(name) == trt.TensorIOMode.OUTPUT:
        host, device, shape = bindings[name]
        cuda.memcpy_dtoh_async(host, device, stream)
        stream.synchronize()
        print(f"  {name}: {host.shape} | sample: {host.flat[:5]}")