import argparse
from pathlib import Path

import torch
import torch.nn as nn

from lightglue import LightGlue


class StaticLightGlue(nn.Module):
    """
    Static LightGlue wrapper.

    Inputs
    ------
    kpts0:
        [1, N, 2] float32
    kpts1:
        [1, N, 2] float32

    desc0:
        [1, N, 256] float32

    desc1:
        [1, N, 256] float32

    scores0:
        [1, N] float32

    scores1:
        [1, N] float32

    Outputs
    -------
    matches0:
        [1, N] int64
        Index of matching keypoint in image1, or -1.

    matches1:
        [1, N] int64
        Index of matching keypoint in image0, or -1.

    matching_scores0:
        [1, N] float32

    matching_scores1:
        [1, N] float32
    """

    def __init__(self, num_keypoints=1024):
        super().__init__()

        self.num_keypoints = num_keypoints

        # Disable adaptive pruning / early stopping.
        #
        # This is important for a static graph.
        self.matcher = LightGlue(
            features="superpoint",
            depth_confidence=-1,
            width_confidence=-1,
        ).eval()

    def forward(
        self,
        kpts0,
        kpts1,
        desc0,
        desc1,
        scores0,
        scores1,
    ):
        feats0 = {
            "keypoints": kpts0,
            "descriptors": desc0,
            "keypoint_scores": scores0,
        }

        feats1 = {
            "keypoints": kpts1,
            "descriptors": desc1,
            "keypoint_scores": scores1,
        }

        out = self.matcher(
            {
                "image0": feats0,
                "image1": feats1,
            }
        )

        return (
            out["matches0"],
            out["matches1"],
            out["matching_scores0"],
            out["matching_scores1"],
        )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--keypoints",
        type=int,
        default=1024,
        help="Fixed number of keypoints per image.",
    )

    parser.add_argument(
        "--output",
        type=str,
        default="weights/lightglue_static_1024.onnx",
    )

    parser.add_argument(
        "--opset",
        type=int,
        default=17,
    )

    args = parser.parse_args()

    N = args.keypoints

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    print("=" * 70)
    print("Static LightGlue ONNX export")
    print("=" * 70)

    print(f"Keypoints : {N}")
    print("Batch     : 1")
    print("Descriptor: 256")
    print(f"Output    : {output}")
    print(f"Opset     : {args.opset}")
    print()

    model = StaticLightGlue(
        num_keypoints=N
    ).eval()

    # ------------------------------------------------------------
    # Fixed-shape dummy inputs
    # ------------------------------------------------------------

    kpts0 = torch.zeros(
        1, N, 2,
        dtype=torch.float32,
    )

    kpts1 = torch.zeros(
        1, N, 2,
        dtype=torch.float32,
    )

    desc0 = torch.randn(
        1, N, 256,
        dtype=torch.float32,
    )

    desc1 = torch.randn(
        1, N, 256,
        dtype=torch.float32,
    )

    scores0 = torch.ones(
        1, N,
        dtype=torch.float32,
    )

    scores1 = torch.ones(
        1, N,
        dtype=torch.float32,
    )

    inputs = (
        kpts0,
        kpts1,
        desc0,
        desc1,
        scores0,
        scores1,
    )

    # ------------------------------------------------------------
    # Test PyTorch model first
    # ------------------------------------------------------------

    print("Running PyTorch test...")

    with torch.no_grad():
        outputs = model(*inputs)

    for i, x in enumerate(outputs):
        print(
            f"  output {i}: "
            f"shape={tuple(x.shape)}, "
            f"dtype={x.dtype}"
        )

    print()

    # ------------------------------------------------------------
    # Export
    # ------------------------------------------------------------

    print("Exporting ONNX...")

    torch.onnx.export(
        model,
        inputs,
        str(output),

        input_names=[
            "keypoints0",
            "keypoints1",
            "descriptors0",
            "descriptors1",
            "scores0",
            "scores1",
        ],

        output_names=[
            "matches0",
            "matches1",
            "matching_scores0",
            "matching_scores1",
        ],

        # IMPORTANT:
        # No dynamic_axes.
        #
        # Every tensor dimension is fixed.
        dynamic_axes=None,

        opset_version=args.opset,

        do_constant_folding=True,

        dynamo=False,
    )

    print()
    print("Export complete.")
    print(output)

    # ------------------------------------------------------------
    # ONNX verification
    # ------------------------------------------------------------

    print()
    print("Checking ONNX...")

    import onnx

    onnx_model = onnx.load(str(output))

    onnx.checker.check_model(onnx_model)

    print("ONNX checker: OK")

    # ------------------------------------------------------------
    # ONNX Runtime test
    # ------------------------------------------------------------

    import onnxruntime as ort
    import numpy as np

    print()
    print("Testing ONNX Runtime...")

    session = ort.InferenceSession(
        str(output),
        providers=["CPUExecutionProvider"],
    )

    print("Inputs:")

    for x in session.get_inputs():
        print(
            f"  {x.name}: "
            f"{x.shape} "
            f"{x.type}"
        )

    print("Outputs:")

    for x in session.get_outputs():
        print(
            f"  {x.name}: "
            f"{x.shape} "
            f"{x.type}"
        )

    ort_inputs = {
        "keypoints0": kpts0.numpy(),
        "keypoints1": kpts1.numpy(),
        "descriptors0": desc0.numpy(),
        "descriptors1": desc1.numpy(),
        "scores0": scores0.numpy(),
        "scores1": scores1.numpy(),
    }

    ort_outputs = session.run(
        None,
        ort_inputs,
    )

    print()
    print("ORT outputs:")

    for name, x in zip(
        [
            "matches0",
            "matches1",
            "matching_scores0",
            "matching_scores1",
        ],
        ort_outputs,
    ):
        print(
            f"  {name}: "
            f"shape={x.shape}, "
            f"dtype={x.dtype}"
        )

    print()
    print("=" * 70)
    print("SUCCESS")
    print("=" * 70)


if __name__ == "__main__":
    main()