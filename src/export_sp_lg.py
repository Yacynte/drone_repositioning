import argparse
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
import torch
import torch.nn as nn
import torch.nn.functional as F

from lightglue import LightGlue, SuperPoint


class ExportableSuperPoint(nn.Module):
    """
    Export-friendly SuperPoint.

    Fixed:
        input:       [B, 1, H, W]
        keypoints:   [B, K, 2]
        scores:      [B, K]
        descriptors: [B, K, 256]

    Unlike the original LightGlue implementation, this does not first
    construct a variable-length list of detected keypoints.

    Instead it performs top-k directly on the dense score image.
    """

    def __init__(self, max_num_keypoints=1024):
        super().__init__()

        self.max_num_keypoints = max_num_keypoints

        # Load the normal SuperPoint implementation so that we use exactly
        # the same architecture and pretrained weights.
        sp = SuperPoint(
            max_num_keypoints=None,
            detection_threshold=0.0005,
            nms_radius=4,
            remove_borders=4,
        ).eval()

        # Copy the actual network into this module.
        self.conv1a = sp.conv1a
        self.conv1b = sp.conv1b
        self.conv2a = sp.conv2a
        self.conv2b = sp.conv2b
        self.conv3a = sp.conv3a
        self.conv3b = sp.conv3b
        self.conv4a = sp.conv4a
        self.conv4b = sp.conv4b

        self.convPa = sp.convPa
        self.convPb = sp.convPb

        self.convDa = sp.convDa
        self.convDb = sp.convDb

        self.relu = nn.ReLU(inplace=False)

    def forward(self, image):
        # ---------------------------------------------------------------
        # Shared encoder
        # ---------------------------------------------------------------

        x = self.relu(self.conv1a(image))
        x = self.relu(self.conv1b(x))
        x = F.max_pool2d(x, 2, 2)

        x = self.relu(self.conv2a(x))
        x = self.relu(self.conv2b(x))
        x = F.max_pool2d(x, 2, 2)

        x = self.relu(self.conv3a(x))
        x = self.relu(self.conv3b(x))
        x = F.max_pool2d(x, 2, 2)

        x = self.relu(self.conv4a(x))
        x = self.relu(self.conv4b(x))

        # ---------------------------------------------------------------
        # Dense keypoint scores
        # ---------------------------------------------------------------

        cPa = self.relu(self.convPa(x))
        scores = self.convPb(cPa)

        scores = F.softmax(scores, dim=1)[:, :-1]

        b, _, h, w = scores.shape

        scores = scores.permute(0, 2, 3, 1)
        scores = scores.reshape(b, h, w, 8, 8)
        scores = scores.permute(0, 1, 3, 2, 4)
        scores = scores.reshape(b, h * 8, w * 8)

        # Same NMS as the original implementation.
        scores = self.simple_nms(scores, 4)

        # Remove borders.
        scores[:, :4] = -1
        scores[:, -4:] = -1
        scores[:, :, :4] = -1
        scores[:, :, -4:] = -1

        # ---------------------------------------------------------------
        # FIXED top-K
        # ---------------------------------------------------------------

        Hs, Ws = scores.shape[-2:]

        flat_scores = scores.reshape(b, -1)

        k = min(self.max_num_keypoints, Hs * Ws)

        top_scores, top_indices = torch.topk(
            flat_scores,
            k=k,
            dim=1,
            sorted=True,
        )

        ys = torch.div(
            top_indices,
            Ws,
            rounding_mode="floor",
        )

        xs = top_indices - ys * Ws

        keypoints = torch.stack(
            [xs, ys],
            dim=-1,
        ).float()

        # ---------------------------------------------------------------
        # Dense descriptors
        # ---------------------------------------------------------------

        cDa = self.relu(self.convDa(x))
        descriptors = self.convDb(cDa)

        descriptors = F.normalize(
            descriptors,
            p=2,
            dim=1,
        )

        # Sample descriptors at the selected keypoints.
        keypoints_desc = keypoints.clone()

        keypoints_desc = keypoints_desc - 8 / 2 + 0.5

        keypoints_desc[..., 0] /= (
            Ws - 8 / 2 - 0.5
        )

        keypoints_desc[..., 1] /= (
            Hs - 8 / 2 - 0.5
        )

        keypoints_desc = keypoints_desc * 2 - 1

        grid = keypoints_desc.view(
            b,
            1,
            k,
            2,
        )

        descriptors = F.grid_sample(
            descriptors,
            grid,
            mode="bilinear",
            align_corners=True,
        )

        descriptors = descriptors.reshape(
            b,
            256,
            k,
        )

        descriptors = F.normalize(
            descriptors,
            p=2,
            dim=1,
        )

        # LightGlue expects [B, K, C].
        descriptors = descriptors.transpose(1, 2).contiguous()

        return keypoints, top_scores, descriptors

    @staticmethod
    def simple_nms(scores, nms_radius):
        """
        Same NMS algorithm as LightGlue's SuperPoint implementation.
        """

        def max_pool(x):
            return F.max_pool2d(
                x,
                kernel_size=nms_radius * 2 + 1,
                stride=1,
                padding=nms_radius,
            )

        zeros = torch.zeros_like(scores)

        max_mask = scores == max_pool(scores)

        for _ in range(2):
            supp_mask = max_pool(max_mask.float()) > 0

            supp_scores = torch.where(
                supp_mask,
                zeros,
                scores,
            )

            new_max_mask = supp_scores == max_pool(
                supp_scores
            )

            max_mask = max_mask | (
                new_max_mask & (~supp_mask)
            )

        return torch.where(
            max_mask,
            scores,
            zeros,
        )


class SuperPointLightGluePipeline(nn.Module):

    def __init__(self, max_num_keypoints=1024):
        super().__init__()

        self.max_num_keypoints = max_num_keypoints

        self.extractor = ExportableSuperPoint(
            max_num_keypoints=max_num_keypoints
        ).eval()

        self.matcher = LightGlue(
            features="superpoint"
        ).eval()

    def forward(self, image0, image1):

        kpts0, scores0, desc0 = self.extractor(image0)
        kpts1, scores1, desc1 = self.extractor(image1)

        feats0 = {
            "keypoints": kpts0,
            "keypoint_scores": scores0,
            "descriptors": desc0,
        }

        feats1 = {
            "keypoints": kpts1,
            "keypoint_scores": scores1,
            "descriptors": desc1,
        }

        matches_dict = self.matcher(
            {
                "image0": feats0,
                "image1": feats1,
            }
        )

        matches = matches_dict["matches"][0]
        match_scores = matches_dict["scores"][0]

        # LightGlue may return a variable number of matches.
        #
        # We convert the match indices into coordinates.
        mkpts0 = kpts0[0][matches[:, 0]]
        mkpts1 = kpts1[0][matches[:, 1]]

        return mkpts0, mkpts1, match_scores


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--height",
        type=int,
        default=1080,
    )

    parser.add_argument(
        "--width",
        type=int,
        default=1920,
    )

    parser.add_argument(
        "--keypoints",
        type=int,
        default=1024,
    )

    parser.add_argument(
        "--output",
        default="weights/superpoint_lightglue.onnx",
    )

    args = parser.parse_args()

    output = Path(args.output)
    output.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    print()
    print("========================================")
    print("SuperPoint + LightGlue ONNX export")
    print("========================================")
    print(f"Input:      {args.width} x {args.height}")
    print(f"Keypoints:  {args.keypoints}")
    print(f"Output:     {output}")
    print()

    pipeline = SuperPointLightGluePipeline(
        max_num_keypoints=args.keypoints
    ).eval()

    pipeline.requires_grad_(False)

    dummy0 = torch.zeros(
        1,
        1,
        args.height,
        args.width,
        dtype=torch.float32,
    )

    dummy1 = torch.zeros_like(dummy0)

    print("Testing PyTorch forward...")

    with torch.no_grad():

        pt_outputs = pipeline(
            dummy0,
            dummy1,
        )

    for name, tensor in zip(
        ["mkpts0", "mkpts1", "scores"],
        pt_outputs,
    ):
        print(
            f"  {name}: "
            f"shape={tuple(tensor.shape)} "
            f"dtype={tensor.dtype}"
        )

    print()
    print("Exporting ONNX...")

    torch.onnx.export(
        pipeline,
        (dummy0, dummy1),
        str(output),
        input_names=[
            "image0",
            "image1",
        ],
        output_names=[
            "mkpts0",
            "mkpts1",
            "scores",
        ],
        opset_version=17,
        dynamo=False,
        do_constant_folding=True,
    )

    print()
    print("Checking ONNX...")

    model = onnx.load(str(output))
    onnx.checker.check_model(model)

    print("ONNX CHECK: OK")

    print()
    print("Loading with ONNX Runtime...")

    session = ort.InferenceSession(
        str(output),
        providers=[
            "CPUExecutionProvider"
        ],
    )

    print(
        "Providers:",
        session.get_providers(),
    )

    print(
        "Inputs:",
        [
            (x.name, x.shape, x.type)
            for x in session.get_inputs()
        ],
    )

    print(
        "Outputs:",
        [
            (x.name, x.shape, x.type)
            for x in session.get_outputs()
        ],
    )

    print()
    print("========================================")
    print("EXPORT COMPLETE")
    print("========================================")


if __name__ == "__main__":
    main()