import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import numpy as np

class TRTSession:
    """Drop-in replacement for ort.InferenceSession using TensorRT."""

    def __init__(self, engine_path):
        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f:
            self.engine  = trt.Runtime(logger).deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()
        self.stream  = cuda.Stream()

        # collect input/output names in order
        self.input_names  = []
        self.output_names = []
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.input_names.append(name)
            else:
                self.output_names.append(name)

        print(f"[TRTSession] Loaded: {engine_path}")
        print(f"  inputs:  {self.input_names}")
        print(f"  outputs: {self.output_names}")

        # preallocate output buffers
        self._out_host   = {}
        self._out_device = {}
        self._alloc_outputs()

    def _alloc_outputs(self):
        MAX_KP = 512  # match your engine
        for name in self.output_names:
            shape = list(self.context.get_tensor_shape(name))
            shape = [MAX_KP if d == -1 else d for d in shape]
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            host   = np.empty(shape, dtype=dtype)
            device = cuda.mem_alloc(host.nbytes)
            self._out_host[name]   = host
            self._out_device[name] = device
            self.context.set_tensor_address(name, int(device))

    def run(self, output_names, input_feed: dict):
        # ── upload inputs ─────────────────────────────────────────────────────
        in_devices = {}
        for name, array in input_feed.items():
            array = np.ascontiguousarray(array)
            device = cuda.mem_alloc(array.nbytes)
            cuda.memcpy_htod_async(device, array, self.stream)
            self.context.set_tensor_address(name, int(device))
            in_devices[name] = device

        # ── set output addresses ──────────────────────────────────────────────
        for name in self.output_names:
            self.context.set_tensor_address(name, int(self._out_device[name]))

        # ── inference ─────────────────────────────────────────────────────────
        self.context.execute_async_v3(self.stream.handle)
        self.stream.synchronize()

        # ── download outputs ──────────────────────────────────────────────────
        results = []
        for name in self.output_names:
            cuda.memcpy_dtoh_async(self._out_host[name], self._out_device[name], self.stream)
        self.stream.synchronize()

        for name in self.output_names:
            results.append(self._out_host[name].copy())

        # free input device memory
        for device in in_devices.values():
            device.free()

        return results