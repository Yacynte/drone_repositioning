# export_sp_lg.py
import torch
import torch.nn as nn
from pathlib import Path
from lightglue import LightGlue, SuperPoint
from lightglue.utils import rbd


class SuperPointExporter(nn.Module):
    """Input: image (1,1,H,W) → keypoints (1,N,2), descriptors (1,N,256), scores (1,N)"""
    def __init__(self, max_num_keypoints: int):
        super().__init__()
        self.device = torch.device('cpu')
        self.sp = SuperPoint(max_num_keypoints=max_num_keypoints).eval().to(self.device)

    def forward(self, image: torch.Tensor):
        features = self.sp.extract(image, resize=None)
        # Return flattened tensors instead of a dict to make ONNX export straightforward
        return features['keypoints'], features['descriptors'], features['keypoint_scores']


class LightGlueExporter(nn.Module):
    """
    Inputs: 
        kpts0 (1,N,2), desc0 (1,N,256), scores0 (1,N)  ← current frame
        kpts1 (1,M,2), desc1 (1,M,256), scores1 (1,M)  ← target (cached)
    Outputs: mkpts0 (K,2), mkpts1 (K,2), mscores (K,)
    """
    def __init__(self):
        super().__init__()
        self.device =torch.device('cpu')
        self.lg = LightGlue(
            features='superpoint',
            depth_confidence=-1,   # disables dynamic control flow for ONNX compatibility
            width_confidence=-1,
        ).eval().to(self.device)

    def forward(
        self, 
        kpts0: torch.Tensor, desc0: torch.Tensor, scores0: torch.Tensor,
        kpts1: torch.Tensor, desc1: torch.Tensor, scores1: torch.Tensor
    ):
        # Reconstruct internal dictionary format expected by LightGlue module
        features0 = {'keypoints': kpts0, 'descriptors': desc0, 'keypoint_scores': scores0}
        features1 = {'keypoints': kpts1, 'descriptors': desc1, 'keypoint_scores': scores1}

        results = self.lg({'image0': features0, 'image1': features1})

        # cur_f, tgt_f, res = rbd(features0), rbd(features1), rbd(results)
        
        # matches = res['matches']
        # # Fallback or direct indexing safe for tracing
        # mkpts0 = cur_f['keypoints'][matches[:, 0]] if matches.numel() > 0 else torch.zeros(0, 2, device=self.device)
        # mkpts1 = tgt_f['keypoints'][matches[:, 1]] if matches.numel() > 0 else torch.zeros(0, 2, device=self.device)
        # scores = res['scores'] if 'scores' in res else torch.zeros(0, device=self.device)
        # Use rbd() safely as intended by LightGlue to strip batch dimensions cleanly for tracing
        cur_f, tgt_f, res = rbd(features0), rbd(features1), rbd(results)
        
        matches = res['matches']
        scores = res['scores']

        # Trace-safe index extraction using 1D index tensor splitting
        m_idx0 = matches[:, 0]
        m_idx1 = matches[:, 1]

        mkpts0 = cur_f['keypoints'][m_idx0]
        mkpts1 = tgt_f['keypoints'][m_idx1]
        return mkpts0, mkpts1, scores


def export_superpoint(out_dir: Path, height: int, width: int, max_kp: int):
    out_path = out_dir / "superpoint.onnx"
    model = SuperPointExporter(max_kp).eval()
    dummy = torch.zeros(1, 1, height, width, dtype=torch.float32)

    with torch.no_grad():
        torch.onnx.export(
            model, (dummy,), str(out_path),
            input_names  = ['image'],
            output_names = ['keypoints', 'descriptors', 'scores'],
            dynamic_axes = {
                'image':       {2: 'height', 3: 'width'},
                'keypoints':   {0: 'batch', 1: 'num_keypoints'},
                'descriptors': {0: 'batch', 1: 'num_keypoints'},
                'scores':      {0: 'batch', 1: 'num_keypoints'},
            },
            opset_version=16, do_constant_folding = True, dynamo=False,
        )
    print(f"[SP] Saved: {out_path}")


def export_lightglue(out_dir: Path, max_kp: int):
    out_path = out_dir / "lightglue.onnx"
    model = LightGlueExporter().eval()

    N = max_kp
    dummy_kpts   = torch.zeros(1, N, 2,   dtype=torch.float32)
    dummy_desc   = torch.zeros(1, N, 256, dtype=torch.float32)
    dummy_scores = torch.zeros(1, N,      dtype=torch.float32)

    with torch.no_grad():
        torch.onnx.export(
            model,
            (dummy_kpts, dummy_desc, dummy_scores,
             dummy_kpts, dummy_desc, dummy_scores),
            str(out_path),
            input_names  = ['kpts0', 'desc0', 'scores0', 'kpts1', 'desc1', 'scores1'],
            output_names = ['mkpts0', 'mkpts1', 'mscores'],
            dynamic_axes = {
                'kpts0':   {0: 'batch', 1: 'num_kp0'},
                'desc0':   {0: 'batch', 1: 'num_kp0'},
                'scores0': {0: 'batch', 1: 'num_kp0'},
                'kpts1':   {0: 'batch', 1: 'num_kp1'},
                'desc1':   {0: 'batch', 1: 'num_kp1'},
                'scores1': {0: 'batch', 1: 'num_kp1'},
                'mkpts0':  {0: 'num_matches'},
                'mkpts1':  {0: 'num_matches'},
                'mscores': {0: 'num_matches'},
            },
            opset_version       = 18,   # dynamo exporter already used 18 — keep consistent
            do_constant_folding = True,
            dynamo              = False, # ← explicit
        )
    print(f"[LG] Saved: {out_path}")


if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--height',     type=int, default=1080)
    p.add_argument('--width',      type=int, default=1920)
    p.add_argument('--keypoints',  type=int, default=2048)
    p.add_argument('--output_dir', type=str, default='weights')
    args = p.parse_args()

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    export_superpoint(out_dir, args.height, args.width, args.keypoints)
    export_lightglue(out_dir, args.keypoints)
    print("\nDone. Models saved to:", out_dir)