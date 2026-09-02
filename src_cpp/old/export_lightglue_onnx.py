import argparse
from pathlib import Path

import torch
import torch.nn as nn
import torch.nn.functional as F

from lightglue import LightGlue


# ============================================================
# ONNX-safe attention
# ============================================================

class ONNXAttention(nn.Module):
    """
    Explicit scaled dot-product attention.

    q: [B, H, M, D]
    k: [B, H, N, D]
    v: [B, H, N, D]

    output:
        [B, H, M, D]
    """

    def forward(self, q, k, v):
        d = q.shape[-1]

        scores = torch.matmul(
            q,
            k.transpose(-2, -1),
        )

        scores = scores / torch.sqrt(
            torch.tensor(
                float(d),
                dtype=q.dtype,
                device=q.device,
            )
        )

        attention = F.softmax(scores, dim=-1)

        return torch.matmul(attention, v)


# ============================================================
# Rotary positional encoding
# ============================================================

def rotate_half(x):
    """
    x:
        [..., D]

    D must be even.
    """

    shape = x.shape

    x = x.reshape(
        *shape[:-1],
        shape[-1] // 2,
        2,
    )

    x1 = x[..., 0]
    x2 = x[..., 1]

    x = torch.stack(
        (-x2, x1),
        dim=-1,
    )

    return x.reshape(*shape)


def apply_rotary(freqs, x):
    """
    freqs:
        [2, B, H, N, D]

    x:
        [B, H, N, D]
    """

    return (
        x * freqs[0]
        + rotate_half(x) * freqs[1]
    )


# ============================================================
# ONNX Self Attention
# ============================================================

class ONNXSelfBlock(nn.Module):

    def __init__(self, original):
        super().__init__()

        self.embed_dim = original.embed_dim
        self.num_heads = original.num_heads
        self.head_dim = original.head_dim

        self.Wqkv = original.Wqkv
        self.out_proj = original.out_proj
        self.ffn = original.ffn

        self.attention = ONNXAttention()

    def forward(self, x, encoding):

        B = x.shape[0]
        N = x.shape[1]

        qkv = self.Wqkv(x)

        # [B, N, 3*D]
        #
        # -> [B, N, 3, H, Dh]
        qkv = qkv.reshape(
            B,
            N,
            3,
            self.num_heads,
            self.head_dim,
        )

        # -> [B, H, N, 3, Dh]
        qkv = qkv.permute(
            0,
            3,
            1,
            2,
            4,
        )

        q = qkv[:, :, :, 0, :]
        k = qkv[:, :, :, 1, :]
        v = qkv[:, :, :, 2, :]

        q = apply_rotary(
            encoding,
            q,
        )

        k = apply_rotary(
            encoding,
            k,
        )

        context = self.attention(
            q,
            k,
            v,
        )

        # [B, H, N, Dh]
        #
        # -> [B, N, H, Dh]
        context = context.permute(
            0,
            2,
            1,
            3,
        )

        # -> [B, N, D]
        context = context.reshape(
            B,
            N,
            self.embed_dim,
        )

        message = self.out_proj(context)

        return x + self.ffn(
            torch.cat(
                [x, message],
                dim=-1,
            )
        )


# ============================================================
# ONNX Cross Attention
# ============================================================

class ONNXCrossBlock(nn.Module):

    def __init__(self, original):
        super().__init__()

        self.heads = original.heads

        self.embed_dim = (
            original.to_out.out_features
        )

        self.head_dim = (
            original.to_qk.out_features
            // self.heads
        )

        self.scale = self.head_dim ** -0.5

        self.to_qk = original.to_qk
        self.to_v = original.to_v
        self.to_out = original.to_out
        self.ffn = original.ffn

        self.attention = ONNXAttention()

    def _reshape_heads(self, x):

        B = x.shape[0]
        N = x.shape[1]

        x = x.reshape(
            B,
            N,
            self.heads,
            self.head_dim,
        )

        return x.permute(
            0,
            2,
            1,
            3,
        )

    def _merge_heads(self, x):

        B = x.shape[0]
        N = x.shape[2]

        x = x.permute(
            0,
            2,
            1,
            3,
        )

        return x.reshape(
            B,
            N,
            self.heads * self.head_dim,
        )

    def forward(self, x0, x1):

        qk0 = self._reshape_heads(
            self.to_qk(x0)
        )

        qk1 = self._reshape_heads(
            self.to_qk(x1)
        )

        v0 = self._reshape_heads(
            self.to_v(x0)
        )

        v1 = self._reshape_heads(
            self.to_v(x1)
        )

        # LightGlue scales both q and k by sqrt(scale)
        qk0 = qk0 * self.scale ** 0.5
        qk1 = qk1 * self.scale ** 0.5

        # ----------------------------------------------------
        # image 0 -> image 1
        # ----------------------------------------------------

        sim01 = torch.matmul(
            qk0,
            qk1.transpose(-2, -1),
        )

        attn01 = F.softmax(
            sim01,
            dim=-1,
        )

        m0 = torch.matmul(
            attn01,
            v1,
        )

        # ----------------------------------------------------
        # image 1 -> image 0
        # ----------------------------------------------------

        sim10 = torch.matmul(
            qk1,
            qk0.transpose(-2, -1),
        )

        attn10 = F.softmax(
            sim10,
            dim=-1,
        )

        m1 = torch.matmul(
            attn10,
            v0,
        )

        # Merge heads

        m0 = self._merge_heads(m0)
        m1 = self._merge_heads(m1)

        m0 = self.to_out(m0)
        m1 = self.to_out(m1)

        x0 = x0 + self.ffn(
            torch.cat(
                [x0, m0],
                dim=-1,
            )
        )

        x1 = x1 + self.ffn(
            torch.cat(
                [x1, m1],
                dim=-1,
            )
        )

        return x0, x1


# ============================================================
# ONNX Transformer Layer
# ============================================================

class ONNXTransformerLayer(nn.Module):

    def __init__(self, original):
        super().__init__()

        self.self_attn = ONNXSelfBlock(
            original.self_attn
        )

        self.cross_attn = ONNXCrossBlock(
            original.cross_attn
        )

    def forward(
        self,
        desc0,
        desc1,
        encoding0,
        encoding1,
    ):

        desc0 = self.self_attn(
            desc0,
            encoding0,
        )

        desc1 = self.self_attn(
            desc1,
            encoding1,
        )

        desc0, desc1 = self.cross_attn(
            desc0,
            desc1,
        )

        return desc0, desc1


# ============================================================
# ONNX Match Assignment
# ============================================================

class ONNXMatchAssignment(nn.Module):

    def __init__(self, original):
        super().__init__()

        self.dim = original.dim

        self.matchability = original.matchability
        self.final_proj = original.final_proj

    def forward(self, desc0, desc1):

        mdesc0 = self.final_proj(desc0)
        mdesc1 = self.final_proj(desc1)

        scale = self.dim ** 0.25

        mdesc0 = mdesc0 / scale
        mdesc1 = mdesc1 / scale

        sim = torch.matmul(
            mdesc0,
            mdesc1.transpose(-2, -1),
        )

        z0 = self.matchability(desc0)
        z1 = self.matchability(desc1)

        # Log assignment

        certainties = (
            F.logsigmoid(z0)
            + F.logsigmoid(
                z1.transpose(1, 2)
            )
        )

        scores0 = F.log_softmax(
            sim,
            dim=2,
        )

        scores1 = F.log_softmax(
            sim.transpose(-2, -1),
            dim=2,
        ).transpose(-2, -1)

        scores = (
            scores0
            + scores1
            + certainties
        )

        return scores


# ============================================================
# Full ONNX LightGlue
# ============================================================

class ONNXLightGlue(nn.Module):

    def __init__(self):

        super().__init__()

        # Load official pretrained LightGlue.
        #
        # FlashAttention is disabled because we want an
        # explicit ONNX-compatible graph.
        original = LightGlue(
            features="superpoint",
            depth_confidence=-1,
            width_confidence=-1,
            flash=False,
            mp=False,
        ).eval()

        self.num_heads = original.conf.num_heads
        self.descriptor_dim = original.conf.descriptor_dim

        self.posenc = original.posenc

        self.transformers = nn.ModuleList(
            [
                ONNXTransformerLayer(layer)
                for layer in original.transformers
            ]
        )

        self.log_assignment = nn.ModuleList(
            [
                ONNXMatchAssignment(layer)
                for layer in original.log_assignment
            ]
        )

    def forward(
        self,
        kpts0,
        scores0,
        desc0,
        kpts1,
        scores1,
        desc1,
    ):

        # ----------------------------------------------------
        # Normalize keypoints
        #
        # We don't use image_size because the exported model
        # receives already extracted keypoints.
        #
        # The normalization here reproduces LightGlue's
        # normalize_keypoints() behavior when image_size is
        # absent.
        # ----------------------------------------------------

        size0 = (
            1
            + kpts0.max(dim=-2).values
            - kpts0.min(dim=-2).values
        )

        size1 = (
            1
            + kpts1.max(dim=-2).values
            - kpts1.min(dim=-2).values
        )

        shift0 = size0 / 2
        scale0 = size0.max(dim=-1).values / 2

        shift1 = size1 / 2
        scale1 = size1.max(dim=-1).values / 2

        kpts0 = (
            kpts0
            - shift0.unsqueeze(-2)
        ) / scale0.unsqueeze(-1).unsqueeze(-1)

        kpts1 = (
            kpts1
            - shift1.unsqueeze(-2)
        ) / scale1.unsqueeze(-1).unsqueeze(-1)

        # ----------------------------------------------------
        # Positional encoding
        # ----------------------------------------------------

        encoding0 = self.posenc(kpts0)
        encoding1 = self.posenc(kpts1)

        # ----------------------------------------------------
        # LightGlue transformer stack
        # ----------------------------------------------------

        for i in range(
            len(self.transformers)
        ):

            desc0, desc1 = self.transformers[i](
                desc0,
                desc1,
                encoding0,
                encoding1,
            )

        # ----------------------------------------------------
        # Final assignment
        # ----------------------------------------------------

        scores = self.log_assignment[-1](
            desc0,
            desc1,
        )

        # Remove dustbin row/column

        scores = scores[
            :, :-1, :-1
        ]

        # ----------------------------------------------------
        # Mutual nearest-neighbor matching
        # ----------------------------------------------------

        max0 = scores.max(
            dim=2
        )

        max1 = scores.max(
            dim=1
        )

        m0 = max0.indices
        m1 = max1.indices

        # Indices

        N = kpts0.shape[1]
        M = kpts1.shape[1]

        indices0 = torch.arange(
            N,
            device=kpts0.device,
        ).unsqueeze(0)

        indices1 = torch.arange(
            M,
            device=kpts1.device,
        ).unsqueeze(0)

        # Mutual matches

        mutual0 = ( indices0 == m1.gather(1, m0))

        mutual1 = (indices1 == m0.gather(1, m1))

        match_scores0 = torch.exp(
            max0.values
        )

        match_scores1 = torch.where(
            mutual1,
            match_scores0.gather(
                1,
                m1,
            ),
            torch.zeros_like(
                match_scores0
            ),
        )

        # LightGlue threshold

        threshold = 0.1

        valid0 = (
            mutual0
            & (match_scores0 > threshold)
        )

        valid1 = (
            mutual1
            & valid0.gather(
                1,
                m1,
            )
        )

        matches0 = torch.where(
            valid0,
            m0,
            torch.full_like(
                m0,
                -1,
            ),
        )

        matches1 = torch.where(
            valid1,
            m1,
            torch.full_like(
                m1,
                -1,
            ),
        )

        matching_scores0 = torch.where(
            valid0,
            match_scores0,
            torch.zeros_like(
                match_scores0
            ),
        )

        matching_scores1 = torch.where(
            valid1,
            match_scores1,
            torch.zeros_like(
                match_scores1
            ),
        )

        return (
            matches0,
            matches1,
            matching_scores0,
            matching_scores1,
        )


# ============================================================
# Export
# ============================================================

def export_model(
    output: Path,
    n0: int = 512,
    n1: int = 511,
):

    print("Creating LightGlue...")

    model = ONNXLightGlue().eval()

    print("Creating dummy inputs...")

    kpts0 = torch.randn(
        1,
        n0,
        2,
    )

    scores0 = torch.rand(
        1,
        n0,
    )

    desc0 = F.normalize(
        torch.randn(
            1,
            n0,
            256,
        ),
        dim=-1,
    )

    kpts1 = torch.randn(
        1,
        n1,
        2,
    )

    scores1 = torch.rand(
        1,
        n1,
    )

    desc1 = F.normalize(
        torch.randn(
            1,
            n1,
            256,
        ),
        dim=-1,
    )

    inputs = (
        kpts0,
        scores0,
        desc0,
        kpts1,
        scores1,
        desc1,
    )

    print("Running PyTorch test...")

    with torch.no_grad():

        outputs = model(*inputs)

    for name, value in zip(
        [
            "matches0",
            "matches1",
            "matching_scores0",
            "matching_scores1",
        ],
        outputs,
    ):
        print(
            f"  {name}: "
            f"{tuple(value.shape)} "
            f"{value.dtype}"
        )

    output.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    print("\nExporting ONNX...")

    torch.onnx.export(
        model,
        inputs,
        str(output),

        input_names=[
            "kpts0",
            "scores0",
            "desc0",
            "kpts1",
            "scores1",
            "desc1",
        ],

        output_names=[
            "matches0",
            "matches1",
            "matching_scores0",
            "matching_scores1",
        ],

        dynamic_axes={
            "kpts0": {
                1: "num_keypoints0",
            },
            "scores0": {
                1: "num_keypoints0",
            },
            "desc0": {
                1: "num_keypoints0",
            },

            "kpts1": {
                1: "num_keypoints1",
            },
            "scores1": {
                1: "num_keypoints1",
            },
            "desc1": {
                1: "num_keypoints1",
            },

            "matches0": {
                1: "num_keypoints0",
            },
            "matches1": {
                1: "num_keypoints1",
            },
            "matching_scores0": {
                1: "num_keypoints0",
            },
            "matching_scores1": {
                1: "num_keypoints1",
            },
        },

        opset_version=18,

        do_constant_folding=True,

        dynamo=False,
    )

    print(
        f"\nSaved LightGlue ONNX:\n"
        f"  {output}"
    )


# ============================================================
# Main
# ============================================================

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--output",
        type=Path,
        default=Path(
            "weights/lightglue.onnx"
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