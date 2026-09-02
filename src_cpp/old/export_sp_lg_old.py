# export_sp_lg.py
import torch
import torch.nn as nn
from pathlib import Path
from lightglue import LightGlue, SuperPoint
from lightglue.utils import rbd, load_image

class SuperPointLightGluePipeline(nn.Module):
    """
    Wraps SuperPoint + LightGlue into a single ONNX-exportable module.
    Inputs:  image0 (1,1,H,W), image1 (1,1,H,W)  — float32, [0,1]
    Outputs: mkpts0 (M,2), mkpts1 (M,2), scores (M,)
    """
    def __init__(self, target_image_path, max_num_keypoints=2048):
        super().__init__()
        # ── Models ───────────────────────────────────────────────────────
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.extractor = SuperPoint(max_num_keypoints=max_num_keypoints).eval().to(self.device)
        self.matcher   = LightGlue(features='superpoint').eval().to(self.device)

        # ── Target image ─────────────────────────────────────────────────
        target = load_image(target_image_path).to(self.device)
        with torch.no_grad():
            self.target_features = self.extractor.extract(target)

    def forward(self, image: torch.Tensor):
        # SuperPoint expects (1,1,H,W), outputs dict with 'keypoints','descriptors','scores'
        cur_features = self.extractor.extract(image, resize=None)

        # LightGlue
        matches_dict = self.matcher({'image0': cur_features, 'image1': self.target_features})

        cur_f, tgt_f, res = rbd(cur_features), rbd(self.target_features), rbd(matches_dict)
        
        mkpts0  = cur_f['keypoints'][res['matches'][:, 0]]
        mkpts1  = tgt_f['keypoints'][res['matches'][:, 1]]
        scores  = res['scores']
        
        # Remove batch dim
        # kpts0   = feats0['keypoints'][0]        # (N0, 2)
        # kpts1   = feats1['keypoints'][0]        # (N1, 2)
        # matches = matches_dict['matches'][0]    # (M, 2)  indices into kpts0/kpts1
        # scores  = matches_dict['scores'][0]     # (M,)

        # mkpts0 = kpts0[matches[:, 0]]           # (M, 2)
        # mkpts1 = kpts1[matches[:, 1]]           # (M, 2)

        return mkpts0, mkpts1, scores


def export_model(target_image_path:str, output_path: str, height: int, width: int, max_num_keypoints: int):
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True,exist_ok=True)

    print()
    print("=" * 70)
    print("SuperPoint + LightGlue ONNX export")
    print("=" * 70)
    print(f"Image size       : {width} x {height}")
    print(f"Max keypoints    : {max_num_keypoints}")
    print(f"Output           : {output_path}")
    print()
    print("Inputs:")
    print("  target_image_path")
    print("  image [1, 1, H, W]")
    print()
    print("Outputs:")
    print("  mkpts0 [M, 2]")
    print("  mkpts1 [M, 2]")
    print("  scores [M]")
    print("=" * 70)
    print()

    model = SuperPointLightGluePipeline(target_image_path= target_image_path, max_num_keypoints=max_num_keypoints ).eval()
    dummy0 = "groundTruths_pnec/clear/Capture_001.png"
    dummy1 = torch.zeros(1, 1, height, width, dtype=torch.float32)

    # ---------------------------------------------------------
    # Export
    # ---------------------------------------------------------

    print("[Export] Starting ONNX export...")

    torch.onnx.export(
        model,
        (dummy0, dummy1),
        str(output_path),

        input_names=["target_path","image",],

        output_names=[ "mkpts0", "mkpts1", "scores",],

        # Keep H/W fixed.
        #
        # Only M needs to be dynamic because LightGlue may
        # return a different number of matches for every pair.
        dynamic_shapes={
            'mkpts0':  {0: 'num_matches'},
            'mkpts1':  {0: 'num_matches'},
            'scores':  {0: 'num_matches'},
        },
        # opset_version=17,

        # do_constant_folding=True,

        # Use the modern exporter.
        dynamo=True,
    )

    print()
    print(f"[Export] Saved: {output_path}")

if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--height',   type=int, default=1080)
    p.add_argument('--width',    type=int, default=1920)
    p.add_argument('--keypoints',type=int, default=2048)
    p.add_argument('--target_image',type=str, default="groundTruths_pnec/clear/Capture_001.png")
    p.add_argument('--output',   default='weights/superpoint_lightglue.onnx', help="Output ONNX path.")
    args = p.parse_args()

    export_model(
        target_image_path= args.target_image,
        output_path=args.output,
        height=args.height,
        width=args.width,
        max_num_keypoints=args.keypoints,
    )