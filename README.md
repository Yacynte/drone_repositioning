# drone_repositioning

build debug
`cmake -DCMAKE_BUILD_TYPE=Debug ..`
`make`


run `python src/lanchRepositioning.py`
then in a new terminal, attach tmux with `tmux attach -t flagctl`
engel-test-init --stream "rtsp://192.168.0.10:8554/test" --target_image "controls/imagesGT1/GT1_lab.png" --unreal 0

cd drone_repositioning
pip install onnx onnxruntime-gpu
git clone https://github.com/fabio-sim/LightGlue-ONNX.git

or gitclone -r recursive
cd LightGlue-ONNX

pip install -e .
python dynamo.py export --extractor-type superpoint --output weights/superpoint_lightglue.onnx --num-keypoints 1024 --height 1080 --width 1920

install Eigen. opencvc++


# Install uv if you don't have it
curl -LsSf https://astral.sh/uv/install.sh | sh
source $HOME/.local/bin/env
uv sync --extra torch-cuda
uv pip install \
    onnx==1.22.0 \
    onnxruntime==1.27.0 \
    onnxscript==0.7.1

# Clone and sync the environment for your Orin (includes CUDA/TRT support)
git clone https://github.com/fabio-sim/LightGlue-ONNX.git
cd LightGlue-ONNX
uv sync --no-group cpu --group cuda --group export --group trt --extra torch-cuda

# Export the pipelin
uv run lightglue-onnx export \
    --extractor-type superpoint \
    --output weights/superpoint_lightglue.onnx \
    --num-keypoints 1024

for pc (no gpu)
<!-- pip install onnxruntime  -->

for cuda gpu
<!-- pip install onnxruntime-gpu  -->

for getson orin nx
# Confirm your Python and JetPack versions first
python3 --version          # e.g. 3.10
dpkg -l | grep jetpack     # e.g. JetPack 6.1 → CUDA 12.6

# Download the matching aarch64 wheel from ORT releases
# https://github.com/microsoft/onnxruntime/releases
# Pattern: onnxruntime_gpu-<version>-cp<pyver>-cp<pyver>-linux_aarch64.whl

pip install \
  https://github.com/microsoft/onnxruntime/releases/download/v1.20.1/onnxruntime_gpu-1.20.1-cp310-cp310-linux_aarch64.whl \
  opencv-python numpy


sudo timedatectl set-ntp on
sudo usermod -aG video $USER
./build/ImageMatcher --mode "live" --target "target_lab3.png" --imgHeight 1080 --imgWidth 1920

# Temporary (until reboot)
echo -1 | sudo tee /sys/module/usbcore/parameters/autosuspend

# Permanent — add to /etc/rc.local or a udev rule
echo 'options usbcore autosuspend=-1' | sudo tee /etc/modprobe.d/usb-autosuspend.conf