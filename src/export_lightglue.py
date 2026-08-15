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
        kpts1,
        desc1,
        scores1,
    ):
        features0 = {
            "keypoints": kpts0,
            "keypoint_scores": scores0,
            "descriptors": desc0,
        }

        features1 = {
            "keypoints": kpts1,
            "keypoint_scores": scores1,
            "descriptors": desc1,
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
    n0: int,
    n1: int,
):
    print("Creating LightGlue...")

    model = LightGlueIndependent().eval()

    print("Creating dummy inputs...")

    kpts0 = torch.randn(
        1, n0, 2,
        dtype=torch.float32,
    )

    desc0 = torch.randn(
        1, n0, 256,
        dtype=torch.float32,
    )

    scores0 = torch.rand(
        1, n0,
        dtype=torch.float32,
    )

    kpts1 = torch.randn(
        1, n1, 2,
        dtype=torch.float32,
    )

    desc1 = torch.randn(
        1, n1, 256,
        dtype=torch.float32,
    )

    scores1 = torch.rand(
        1, n1,
        dtype=torch.float32,
    )

    inputs = (
        kpts0,
        desc0,
        scores0,
        kpts1,
        desc1,
        scores1,
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
            "kpts1",
            "desc1",
            "scores1",
        ],

        output_names=[
            "matches0",
            "matches1",
            "matching_scores0",
            "matching_scores1",
        ],

        dynamic_axes={
            "kpts0": {
                0: "batch",
                1: "num_keypoints0",
            },
            "desc0": {
                0: "batch",
                1: "num_keypoints0",
            },
            "scores0": {
                0: "batch",
                1: "num_keypoints0",
            },

            "kpts1": {
                0: "batch",
                1: "num_keypoints1",
            },
            "desc1": {
                0: "batch",
                1: "num_keypoints1",
            },
            "scores1": {
                0: "batch",
                1: "num_keypoints1",
            },

            "matches0": {
                0: "batch",
                1: "num_keypoints0",
            },
            "matches1": {
                0: "batch",
                1: "num_keypoints1",
            },
            "matching_scores0": {
                0: "batch",
                1: "num_keypoints0",
            },
            "matching_scores1": {
                0: "batch",
                1: "num_keypoints1",
            },
        },

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
            "weights/lightglue_independent.onnx"
        ),
    )

    parser.add_argument(
        "--n0",
        type=int,
        default=512,
    )

    parser.add_argument(
        "--n1",
        type=int,
        default=511,
    )

    args = parser.parse_args()

    export_model(
        args.output,
        args.n0,
        args.n1,
    )


if __name__ == "__main__":
    main()