import argparse
from pathlib import Path

import torch
import torch.nn as nn

from lightglue import LightGlue


class LightGlueIndependent(nn.Module):
    """
    Standalone LightGlue ONNX wrapper.

    Inputs:
        kpts0   [B, N, 2]
        desc0   [B, N, 256]
        scores0 [B, N]

        kpts1   [B, M, 2]
        desc1   [B, M, 256]
        scores1 [B, M]

    Outputs:
        matches0           [B, N]
        matches1           [B, M]
        matching_scores0   [B, N]
        matching_scores1   [B, M]

    N and M are dynamic.
    """

    def __init__(self):
        super().__init__()

        self.lg = LightGlue(
            features="superpoint",
            depth_confidence=-1,
            width_confidence=-1,
        ).eval()

    def forward(
        self,
        kpts0,
        desc0,
        scores0,
        mask0,
        kpts1,
        desc1,
        scores1,
        mask1,
    ):
        # 1. Extract the boolean mask for the first batch item
        m = mask1[0]  # Shape: [2048]

        # 2. Index and re-add the batch dimension [None, :, :] to restore shape [1, N_valid, D]
        filtered_kpts0 = kpts0[0][m].unsqueeze(0)     # Shape: [1, N_valid, 2]
        filtered_desc0 = desc0[0][m].unsqueeze(0)     # Shape: [1, N_valid, 256]
        filtered_scores0 = scores0[0][m].unsqueeze(0) # Shape: [1, N_valid]0

        # 1. Extract the boolean mask for the first batch item
        m1 = mask1[0]  # Shape: [2048]

        # 2. Index and re-add the batch dimension [None, :, :] to restore shape [1, N_valid, D]
        filtered_kpts1 = kpts1[0][m1].unsqueeze(0)     # Shape: [1, N_valid, 2]
        filtered_desc1 = desc1[0][m1].unsqueeze(0)     # Shape: [1, N_valid, 256]
        filtered_scores1 = scores1[0][m1].unsqueeze(0) # Shape: [1, N_valid]
        
        features0 = {
            "keypoints": filtered_kpts0,
            "keypoint_scores": filtered_scores0,
            "descriptors": filtered_desc0,
        }

        features1 = {
            "keypoints": filtered_kpts1,
            "keypoint_scores": filtered_scores1,
            "descriptors": filtered_desc1,
        }

        result = self.lg(
            {
                "image0": features0,
                "image1": features1,
            }
        )

        return (
            result["matches0"],
            result["matches1"],
            result["matching_scores0"],
            result["matching_scores1"],
        )


def export_model(
    output: Path,
    max_keypoints: int,
):
    print("Creating LightGlue...")

    model = LightGlueIndependent().eval()

    print("Creating dummy inputs...")

    kpts0 = torch.randn(
        1, max_keypoints, 2,
        dtype=torch.float32,
    )

    desc0 = torch.randn(
        1, max_keypoints, 256,
        dtype=torch.float32,
    )

    scores0 = torch.rand(
        1, max_keypoints,
        dtype=torch.float32,
    )

    mask0 = torch.randint(
        0, 2, (1, max_keypoints),
        dtype=torch.bool,
    )

    kpts1 = torch.randn(
        1, max_keypoints, 2,
        dtype=torch.float32,
    )

    desc1 = torch.randn(
        1, max_keypoints, 256,
        dtype=torch.float32,
    )

    scores1 = torch.rand(
        1, max_keypoints,
        dtype=torch.float32,
    )

    mask1 = torch.randint(
        0, 2, (1, max_keypoints),
        dtype=torch.bool,
    )

    inputs = (
        kpts0,
        desc0,
        scores0,
        mask0,
        kpts1,
        desc1,
        scores1,
        mask1,
    )

    # --------------------------------------------------------
    # PyTorch sanity test
    # --------------------------------------------------------

    print("Running PyTorch test...")

    with torch.no_grad():
        outputs = model(*inputs)

    names = [
        "matches0",
        "matches1",
        "matching_scores0",
        "matching_scores1",
    ]

    for name, value in zip(names, outputs):
        print(
            f"  {name:20s}",
            tuple(value.shape),
            value.dtype,
        )

    # --------------------------------------------------------
    # ONNX export
    # --------------------------------------------------------

    print("\nExporting ONNX...")

    output.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    torch.onnx.export(
        model,
        inputs,
        str(output),

        input_names=[
            "kpts0",
            "desc0",
            "scores0",
            "mask0",
            "kpts1",
            "desc1",
            "scores1",
            "mask1",
        ],

        output_names=[
            "matches0",
            "matches1",
            "matching_scores0",
            "matching_scores1",
        ],

        dynamic_axes=None,

        opset_version=18,
        do_constant_folding=True,
        # dynamo=False,
    )

    print(f"\nSaved: {output}")


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--output",
        type=Path,
        default=Path(
            "weights/lightglue_static.onnx"
        ),
    )

    parser.add_argument(
        "--max_keypoints",
        type=int,
        default=2048,
    )

    # parser.add_argument(
    #     "--n1",
    #     type=int,
    #     default=511,
    # )

    args = parser.parse_args()

    export_model(
        args.output,
        args.max_keypoints,
    )


if __name__ == "__main__":
    main()