# drone_repositioning

Vision-based drone repositioning: a C++ app (`ImageMatcher`) reads a live camera/RTSP
stream, matches it against a target image via a Python SuperPoint + LightGlue ONNX
pipeline, and sends repositioning commands to a drone or an Unreal Engine simulation.

## Layout

- `src_cpp/`, `include/` — C++ app (`ImageMatcher`): capture, matching, command/metadata I/O.
- `src_py/` — ONNX export scripts and the matcher process (`matches_onnx.py`) launched by `ImageMatcher`.
- `src_test_py/` — the launcher (`launchRepositioning.py`) and its supporting processes (`flagctl.py`, `receive_msg.py`), plus standalone dev/debug tools for the ONNX/TensorRT pipeline.
- `controls/` — drone/simulation controller (`controls.py`, `new_main.py`) driven over UDP.
- `weights/` — exported ONNX models (`superpoint.onnx`, `lightglue_patched.onnx`).

## Build (C++)

Requires OpenCV and Eigen3.

```bash
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
```

This produces `build/ImageMatcher`.

## Python environment

```bash
# Install uv if you don't have it
curl -LsSf https://astral.sh/uv/install.sh | sh
source $HOME/.local/bin/env
uv sync --extra torch-cuda
uv pip install onnx==1.22.0 onnxruntime==1.27.0 onnxscript==0.7.1
```

CPU-only machines: `pip install onnxruntime`. CUDA machines: `pip install onnxruntime-gpu`.

### Exporting the ONNX models

LightGlue-ONNX is used to export the SuperPoint + LightGlue pipeline:

```bash
git clone --recursive https://github.com/fabio-sim/LightGlue-ONNX.git
cd LightGlue-ONNX
pip install -e .
python dynamo.py export \
    --extractor-type superpoint \
    --output weights/superpoint_lightglue.onnx \
    --num-keypoints 1024 --height 1080 --width 1920
```

On a Jetson Orin, sync the environment with CUDA/TensorRT support instead:

```bash
git clone https://github.com/fabio-sim/LightGlue-ONNX.git
cd LightGlue-ONNX
uv sync --no-group cpu --group cuda --group export --group trt --extra torch-cuda
uv run lightglue-onnx export \
    --extractor-type superpoint \
    --output weights/superpoint_lightglue.onnx \
    --num-keypoints 1024
```

The resulting `superpoint.onnx` / `lightglue_patched.onnx` belong in `weights/`.

## Running

The launcher starts `ImageMatcher`, the interactive command console (`flagctl.py`), and
the metadata relay to Unreal Engine (`receive_msg.py`):

```bash
python src_test_py/launchRepositioning.py --mode live --target target_lab3.png
```

`flagctl.py` opens in its own tmux session (default name `flagctl`); attach to it with:

```bash
tmux attach -t flagctl
```

Run `python src_test_py/launchRepositioning.py --help` for all options (RTSP URL, camera
index, network ports, Unreal Engine relay target, etc).

To run `ImageMatcher` directly instead of through the launcher:

```bash
./build/ImageMatcher --mode live --target target_lab3.png --imgHeight 1080 --imgWidth 1920
```

## Unreal Engine side

Start repositioning from the Unreal Engine console:

```
engel-test-init --stream "rtsp://192.168.0.10:8554/test" --target_image "controls/imagesGT1/GT1_lab.png" --unreal 0
```

## Jetson Orin notes

Check your Python/JetPack version and grab the matching `onnxruntime-gpu` wheel:

```bash
python3 --version          # e.g. 3.10
dpkg -l | grep jetpack     # e.g. JetPack 6.1 → CUDA 12.6
# https://github.com/microsoft/onnxruntime/releases
# onnxruntime_gpu-<version>-cp<pyver>-cp<pyver>-linux_aarch64.whl
pip install \
  https://github.com/microsoft/onnxruntime/releases/download/v1.20.1/onnxruntime_gpu-1.20.1-cp310-cp310-linux_aarch64.whl \
  opencv-python numpy

sudo timedatectl set-ntp on
sudo usermod -aG video $USER
```

The USB camera can suspend under load; disable autosuspend:

```bash
# Temporary (until reboot)
echo -1 | sudo tee /sys/module/usbcore/parameters/autosuspend

# Permanent — add to /etc/rc.local or a udev rule
echo 'options usbcore autosuspend=-1' | sudo tee /etc/modprobe.d/usb-autosuspend.conf
```
