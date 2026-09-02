import onnxruntime as ort

print("ORT version:", ort.__version__)
print("ORT location:", ort.__file__)
print("Available providers:", ort.get_available_providers())