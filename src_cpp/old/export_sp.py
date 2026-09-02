import torch
import torch.nn as nn
from pathlib import Path
from lightglue import SuperPoint

class SuperPointEncoder(nn.Module):
    def __init__(self, max_num_keypoints=1024):
        super().__init__()
        self.extractor = SuperPoint(max_num_keypoints=max_num_keypoints).eval()

    def forward(self, image: torch.Tensor):
        # Extract features
        feats = self.extractor.extract(image, resize=None)
        return feats['keypoints'], feats['descriptors'], feats['scores']

def export_superpoint(output_path: str, height: int, width: int, max_num_keypoints: int):
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    model = SuperPointEncoder(max_num_keypoints=max_num_keypoints).eval()
    dummy_input = torch.zeros(1, 1, height, width, dtype=torch.float32)

    print(f"[Export] Exporting SuperPoint to {output_path}...")
    
    torch.onnx.export(
        model,
        (dummy_input,),
        str(output_path),
        input_names=["image"],
        output_names=["keypoints", "descriptors", "scores"],
        dynamic_axes={
            "image": {2: "height", 3: "width"},
            "keypoints": {0: "num_keypoints"},
            "descriptors": {0: "num_keypoints"},
            "scores": {0: "num_keypoints"},
        },
        opset_version=18,
        do_constant_folding=True,
    )
    print(f"[Export] Successfully saved SuperPoint ONNX model.")

if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--height', type=int, default=1080)
    p.add_argument('--width', type=int, default=1920)
    p.add_argument('--keypoints', type=int, default=1024)
    p.add_argument('--output', default='weights/superpoint.onnx')
    args = p.parse_args()

    export_superpoint(args.output, args.height, args.width, args.keypoints)