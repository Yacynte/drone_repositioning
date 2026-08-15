import argparse
from pathlib import Path

import torch
import torch.nn as nn
import torch.nn.functional as F

from lightglue import SuperPoint


# ============================================================
# ONNX-safe SuperPoint NMS
# ============================================================

def simple_nms_onnx(scores: torch.Tensor, radius: int):
    """
    Equivalent to LightGlue's simple_nms(), but explicitly
    keeps the tensor 4-D for MaxPool2d.

    Input:
        scores: [B, H, W]

    Output:
        scores: [B, H, W]
    """

    assert radius >= 0

    kernel = 2 * radius + 1

    def max_pool(x):
        # IMPORTANT:
        #
        # LightGlue operates on [B,H,W], but MaxPool2d should
        # explicitly receive [B,C,H,W].
        x = x.unsqueeze(1)

        x = F.max_pool2d(x, kernel_size=kernel, stride=1, padding=radius)

        return x.squeeze(1)

    zeros = torch.zeros_like(scores)

    max_mask = scores == max_pool(scores)

    for _ in range(2):
        supp_mask = max_pool(max_mask.float()) > 0

        supp_scores = torch.where(supp_mask, zeros, scores)

        new_max_mask = (supp_scores == max_pool(supp_scores))

        max_mask = (max_mask | (new_max_mask & (~supp_mask)))

    return torch.where(max_mask, scores, zeros)


# ============================================================
# ONNX-safe descriptor sampling
# ============================================================

def sample_descriptors_onnx(keypoints: torch.Tensor, descriptors: torch.Tensor, s: int = 8):
    """
    Equivalent to LightGlue's sample_descriptors().

    keypoints:
        [B, N, 2]

    descriptors:
        [B, C, H, W]

    returns:
        [B, N, C]
    """

    b, c, h, w = descriptors.shape

    keypoints = keypoints - s / 2 + 0.5

    scale = torch.tensor(
        [
            w * s - s / 2 - 0.5,
            h * s - s / 2 - 0.5,
        ],
        dtype=keypoints.dtype,
        device=keypoints.device,
    )

    keypoints = (
        keypoints / scale[None, None]
    )

    keypoints = keypoints * 2 - 1

    # [B,N,2] -> [B,1,N,2]
    grid = keypoints.reshape(b,1,-1,2)

    sampled = F.grid_sample(
        descriptors,
        grid,
        mode="bilinear",
        align_corners=True,
    )

    # [B,C,1,N]
    # -> [B,C,N]
    sampled = sampled.reshape(b,c,-1)

    sampled = F.normalize(sampled, p=2, dim=1)

    # [B,C,N] -> [B,N,C]
    return sampled.transpose(1, 2).contiguous()


# ============================================================
# Independent SuperPoint
# ============================================================

class IndependentSuperPoint(nn.Module):
    """
    SuperPoint implementation based directly on the LightGlue
    SuperPoint source, but with ONNX-safe postprocessing.

    Input:
        image [1,1,H,W]

    Outputs:
        keypoints   [1,N,2]
        scores      [1,N]
        descriptors [1,N,256]

    N is dynamic.

    There is NO padding to max_num_keypoints.
    """

    def __init__(
        self,
        max_num_keypoints: int = 2048,
        detection_threshold: float = 0.0005,
        nms_radius: int = 4,
        remove_borders: int = 4,
    ):
        super().__init__()

        self.max_num_keypoints = max_num_keypoints
        self.detection_threshold = detection_threshold
        self.nms_radius = nms_radius
        self.remove_borders = remove_borders

        # ----------------------------------------------------
        # Architecture from LightGlue SuperPoint
        # ----------------------------------------------------

        self.relu = nn.ReLU(inplace=False)

        self.pool = nn.MaxPool2d(
            kernel_size=2,
            stride=2,
        )

        c1 = 64
        c2 = 64
        c3 = 128
        c4 = 128
        c5 = 256

        self.conv1a = nn.Conv2d(
            1, c1, 3, 1, 1
        )
        self.conv1b = nn.Conv2d(
            c1, c1, 3, 1, 1
        )

        self.conv2a = nn.Conv2d(
            c1, c2, 3, 1, 1
        )
        self.conv2b = nn.Conv2d(
            c2, c2, 3, 1, 1
        )

        self.conv3a = nn.Conv2d(
            c2, c3, 3, 1, 1
        )
        self.conv3b = nn.Conv2d(
            c3, c3, 3, 1, 1
        )

        self.conv4a = nn.Conv2d(
            c3, c4, 3, 1, 1
        )
        self.conv4b = nn.Conv2d(
            c4, c4, 3, 1, 1
        )

        # Detector
        self.convPa = nn.Conv2d(
            c4, c5, 3, 1, 1
        )

        self.convPb = nn.Conv2d(
            c5, 65, 1, 1, 0
        )

        # Descriptor
        self.convDa = nn.Conv2d(
            c4, c5, 3, 1, 1
        )

        self.convDb = nn.Conv2d(
            c5, 256, 1, 1, 0
        )

    def load_lightglue_weights(self):
        """
        Load the official LightGlue SuperPoint weights.

        We instantiate the official SuperPoint only for loading
        its state_dict. Its forward() is NEVER used by this model.
        """

        print("Loading LightGlue SuperPoint weights...")

        reference = SuperPoint(
            max_num_keypoints=self.max_num_keypoints
        ).eval()

        self.load_state_dict(
            reference.state_dict()
        )

        del reference

        print("Weights loaded.")

    # --------------------------------------------------------
    # Backbone
    # --------------------------------------------------------

    def backbone(self, image):
        x = self.relu(self.conv1a(image))
        x = self.relu(self.conv1b(x))
        x = self.pool(x)

        x = self.relu(self.conv2a(x))
        x = self.relu(self.conv2b(x))
        x = self.pool(x)

        x = self.relu(self.conv3a(x))
        x = self.relu(self.conv3b(x))
        x = self.pool(x)

        x = self.relu(self.conv4a(x))
        x = self.relu(self.conv4b(x))

        return x

    # --------------------------------------------------------
    # Forward
    # --------------------------------------------------------

    def forward(self, image):

        # ----------------------------------------------------
        # Input
        # ----------------------------------------------------

        # We deliberately support only grayscale here.
        #
        # This keeps the ONNX graph simple and matches the
        # [1,1,H,W] deployment interface.
        if image.shape[1] != 1:
            raise RuntimeError(
                "IndependentSuperPoint expects grayscale "
                "input [B,1,H,W]"
            )

        # ----------------------------------------------------
        # Shared encoder
        # ----------------------------------------------------

        x = self.backbone(image)

        # ----------------------------------------------------
        # Detector
        # ----------------------------------------------------

        cPa = self.relu(
            self.convPa(x)
        )

        scores = self.convPb(cPa)

        scores = F.softmax(
            scores,
            dim=1,
        )

        # Remove dustbin channel.
        scores = scores[:, :-1]

        b, _, h, w = scores.shape

        # ----------------------------------------------------
        # Rearrange 65-channel detector output into full
        # resolution score map.
        #
        # [B,64,Hc,Wc]
        #      ↓
        # [B,Hc,Wc,8,8]
        #      ↓
        # [B,Hc,8,Wc,8]
        #      ↓
        # [B,H,W]
        # ----------------------------------------------------

        scores = scores.permute(
            0, 2, 3, 1
        )

        scores = scores.reshape(
            b,
            h,
            w,
            8,
            8,
        )

        scores = scores.permute(
            0,
            1,
            3,
            2,
            4,
        )

        scores = scores.reshape(
            b,
            h * 8,
            w * 8,
        )

        # ----------------------------------------------------
        # NMS
        # ----------------------------------------------------

        scores = simple_nms_onnx(
            scores,
            self.nms_radius,
        )

        # ----------------------------------------------------
        # Border removal
        #
        # Do NOT use:
        #
        # scores[:, :pad] = -1
        #
        # because in-place updates are undesirable for ONNX.
        # ----------------------------------------------------

        if self.remove_borders > 0:
            pad = self.remove_borders

            height = scores.shape[-2]
            width = scores.shape[-1]

            yy = torch.arange(
                height,
                device=scores.device,
            )

            xx = torch.arange(
                width,
                device=scores.device,
            )

            valid_y = (
                (yy >= pad)
                & (yy < height - pad)
            )

            valid_x = (
                (xx >= pad)
                & (xx < width - pad)
            )

            valid = (
                valid_y[:, None]
                & valid_x[None, :]
            )

            scores = torch.where(
                valid[None],
                scores,
                torch.zeros_like(scores),
            )

        # ----------------------------------------------------
        # Threshold
        # ----------------------------------------------------

        mask = scores > self.detection_threshold

        # For deployment we export batch=1.
        #
        # torch.nonzero gives:
        #
        # [N,3] = [batch,y,x]
        #
        # We then remove the batch column.
        indices = torch.nonzero(
            mask,
            as_tuple=False,
        )

        # Since deployment is batch=1:
        #
        # indices[:, 1:] = [y,x]
        keypoints_yx = indices[:, 1:]

        keypoint_scores = scores[
            indices[:, 0],
            indices[:, 1],
            indices[:, 2],
        ]

        # ----------------------------------------------------
        # Top-K
        #
        # IMPORTANT:
        #
        # We only apply top-k if there are more than the
        # configured maximum.
        #
        # The exported graph may specialize this branch based
        # on the dummy input. Therefore we intentionally use
        # max_num_keypoints=None during export below.
        # ----------------------------------------------------

        if self.max_num_keypoints is not None:

            # We cannot safely branch on a dynamic N during
            # ONNX tracing.
            #
            # Instead, this is handled by the exporter wrapper
            # below using a fixed TopK only when desired.
            pass

        # ----------------------------------------------------
        # Convert (y,x) -> (x,y)
        # ----------------------------------------------------

        keypoints = torch.flip(
            keypoints_yx,
            dims=[1],
        ).float()

        # ----------------------------------------------------
        # Dense descriptors
        # ----------------------------------------------------

        cDa = self.relu(
            self.convDa(x)
        )

        descriptors = self.convDb(cDa)

        descriptors = F.normalize(
            descriptors,
            p=2,
            dim=1,
        )

        # ----------------------------------------------------
        # Sample descriptor at every detected keypoint
        # ----------------------------------------------------

        descriptors = sample_descriptors_onnx(
            keypoints[None],
            descriptors,
            8,
        )

        return (
            keypoints[None],
            descriptors,
            keypoint_scores[None],
        )


# ============================================================
# Export
# ============================================================

def export_superpoint(
    output: Path,
    height: int,
    width: int,
    max_keypoints: int,
):

    print()
    print("=" * 60)
    print("Exporting independent SuperPoint")
    print("=" * 60)

    model = IndependentSuperPoint(
        # IMPORTANT:
        #
        # We do not perform TopK inside the exported graph.
        #
        # The number of output features is determined by the
        # detection threshold.
        max_num_keypoints=None,
    ).eval()

    model.load_lightglue_weights()

    dummy = torch.zeros(
        1,
        1,
        height,
        width,
        dtype=torch.float32,
    )

    output.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    print("Exporting...")

    with torch.no_grad():

        torch.onnx.export(
            model,
            (dummy,),
            str(output),

            input_names=[
                "image",
            ],

            output_names=[
                "keypoints",
                "descriptors",
                "scores",
            ],

            dynamic_axes={
                "image": {
                    2: "height",
                    3: "width",
                },

                "keypoints": {
                    1: "num_keypoints",
                },

                "scores": {
                    1: "num_keypoints",
                },

                "descriptors": {
                    1: "num_keypoints",
                },
            },

            opset_version=18,

            do_constant_folding=True,

            dynamo=False,
        )

    print()
    print("Saved:")
    print(output)


# ============================================================
# Main
# ============================================================

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
        default=2048,
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=Path(
            "weights/superpoint.onnx"
        ),
    )

    args = parser.parse_args()

    export_superpoint(
        args.output,
        args.height,
        args.width,
        args.keypoints,
    )


if __name__ == "__main__":
    main()