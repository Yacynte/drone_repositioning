"""
Runs continuously alongside the C++ ImageMatcher app (launched by main.cpp via
launch_python_posix), matching each incoming camera frame against a fixed target
image using ONNX-exported SuperPoint (keypoint detection) + LightGlue (matching).

Two POSIX shared-memory segments tie this process to the C++ side:
  - shm_name ("single_frame_shm", written by RtspReader in RstpReader.cpp): the
    latest grayscale camera frame. See _get_latest_frame().
  - SHM_NAME_MATCHES ("sp_sg_matches", read by SPSGReader in Matches.hpp): this
    process's match results (keypoints + scores + per-match covariance) for the
    latest frame. See _write_matches() for the exact byte layout (must match
    SPSGReader::parse() in Matches.hpp).

Entry point: run() loops forever, pulling frames and writing matches, until either
the C++ side sets is_alive=0 on the frame segment or this process is killed.
"""
import numpy as np
import cv2
import struct
import time
import argparse
import signal
import onnxruntime as ort
from multiprocessing import shared_memory

class FeatureMatcherONNX:
    # Loads the SuperPoint/LightGlue ONNX models, opens/creates both shared-memory
    # segments, and extracts+caches SuperPoint features for target_image_path (see
    # set_target()). shm_name must match the segment name RtspReader was constructed
    # with on the C++ side.
    def __init__(self, target_image_path, shm_name="single_frame_shm"):
        self.running = True
        self.WIDTH = 1920
        self.HEIGHT = 1080

        self.superpointWidth = 640
        self.superpointHeight = 480

        self.ratio_height = self.HEIGHT / self.superpointHeight
        self.ratio_width = self.WIDTH / self.superpointWidth

        # ── Matches shared memory setup (Kept identical to yours) ────────────────
        self.SHM_NAME_MATCHES = "sp_sg_matches"
        self.MAX_KP_MATCHES = 512
        self.SHM_SIZE_MATCHES = 1 + 1 + 1 + 4 + 4 + self.MAX_KP_MATCHES * (2 + 2 + 1 + 3) * 4
        
        try:
            self.shm = shared_memory.SharedMemory(name=self.SHM_NAME_MATCHES)
        except FileNotFoundError:
            self.shm = shared_memory.SharedMemory(name=self.SHM_NAME_MATCHES, create=True, size=self.SHM_SIZE_MATCHES)

        self.buf = self.shm.buf
        self.buf[0] = 1 

        # ── Frame shared memory setup ───────────────────────────────────────────
        self.HEADER_SIZE = 1 + 1 + 1 + 1 + 4 + 4 + 4
        self.shm_frames = shared_memory.SharedMemory(name=shm_name)
        self.buf_frames = self.shm_frames.buf
        self.last_processed_frame_id = -1

        # ── ONNX Runtime Session with TensorRT Provider ─────────────────────────
        # NOTE: hardcoded absolute paths (including the "user" account name) rather
        # than a path relative to this script or an expanded "~" like the C++ side
        # uses (see expandUser() in Utils.cpp) — will break if deployed under a
        # different username or install location.
        self.sp_session = self.get_session("/home/user/drone_repositioning/weights/superpoint.onnx", provider="auto")
        self.lg_session = self.get_session("/home/user/drone_repositioning/weights/lightglue_patched.onnx", provider="auto")

        # Extract and cache target features — call again to switch target
        self.set_target(target_image_path)

    # Builds an onnxruntime InferenceSession for model_path with the requested
    # execution provider. "auto" prefers CUDA when available, else falls back to
    # CPU. A commented-out TensorRT branch is left in place (both here and under
    # "auto") as a starting point for enabling TRT once its engine-cache setup is
    # verified on target hardware.
    def get_session(self, model_path, provider="auto"):
        available = ort.get_available_providers()
        
        # Configure session options to bypass strict shape inference failures on legacy attributes
        sess_options = ort.SessionOptions()
        sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        # This prevents ONNX Runtime from throwing shape inference errors on custom/imported nodes like MaxPool
        sess_options.add_session_config_entry("session.load_model_fmt", "Protobuf")

        if provider == "cpu":
            providers = ["CPUExecutionProvider"]
        elif provider == "cuda":
            if "CUDAExecutionProvider" not in available:
                raise RuntimeError("CUDAExecutionProvider is not available")
            providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        # elif provider == "trt":                                          # ← add this
        #     if "TensorrtExecutionProvider" not in available:
        #         raise RuntimeError("TensorrtExecutionProvider not available")
        #     providers = [
        #         ('TensorrtExecutionProvider', {
        #             'trt_engine_cache_enable': True,
        #             'trt_engine_cache_path':   'weights/trt_cache',
        #             'trt_fp16_enable':         True,
        #             "trt_profile_min_shapes": "image:1x1x480x640",
        #             "trt_profile_opt_shapes": "image:1x1x480x640",
        #             "trt_profile_max_shapes": "image:1x1x480x640",
        #         }),
        #         'CUDAExecutionProvider',
        #         'CPUExecutionProvider',
        #     ]
        elif provider == "auto":
            # if "TensorrtExecutionProvider" in available:                 # ← prefer TRT
            #     providers = [
            #         ('TensorrtExecutionProvider', {
            #             'trt_engine_cache_enable': True,
            #             'trt_engine_cache_path':   'weights/trt_cache',
            #             'trt_fp16_enable':         True,
            #         }),
            #         'CUDAExecutionProvider',
            #         'CPUExecutionProvider',
            #     ]
            if "CUDAExecutionProvider" in available:
                providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
            else:
                providers = ["CPUExecutionProvider"]
        else:
            raise ValueError(f"Unknown provider: {provider}")

        session = ort.InferenceSession(
            model_path,
            sess_options=sess_options,  # Pass the customized options here
            providers=providers,
        )

        return session


    def set_target(self, target_image_path: str):
        """Call this whenever the target image changes — no re-export needed."""
        img = cv2.imread(target_image_path, cv2.IMREAD_GRAYSCALE)
        img = cv2.resize(img, (self.superpointWidth, self.superpointHeight))
        tensor = img.astype(np.float32) / 255.0
        tensor = tensor[None, None]  # (1,1,H,W)

        (self.target_kpts, self.target_desc, self.target_scores, self.target_mask, 
                            self.target_num_keypoints) = self.pad_superpoint(*self.sp_session.run(None, {'image': tensor}))

    def pad_superpoint(self, kpts, desc, scores):
        """
        Convert variable-length SuperPoint output into
        fixed-size tensors.

        Returns:
            kpts    [1, MAX, 2]
            desc    [1, MAX, 256]
            scores  [1, MAX]
            mask    [1, MAX]
            n       int
        """

        n = kpts.shape[1]
        max_keypoints = self.MAX_KP_MATCHES
        if n > max_keypoints:
            kpts = kpts[:, :max_keypoints, :]
            desc = desc[:, :max_keypoints, :]
            scores = scores[:, :max_keypoints]
            n = max_keypoints

        pad = max_keypoints - n

        if pad > 0:
            kpts = np.pad(
                kpts,
                ((0, 0), (0, pad), (0, 0)),
                constant_values=0,
            )

            desc = np.pad(
                desc,
                ((0, 0), (0, pad), (0, 0)),
                constant_values=0,
            )

            scores = np.pad(
                scores,
                ((0, 0), (0, pad)),
                constant_values=0,
            )

        mask = np.zeros(
            (1, max_keypoints),
            dtype=np.bool_,
        )

        mask[:, :n] = True

        return kpts, desc, scores, mask, n

    # Intended as a signal handler to stop run()'s loop gracefully, but it's never
    # registered (no signal.signal(...) call anywhere) — currently dead code. In
    # practice Ctrl+C still works because Python's default SIGINT handling raises
    # KeyboardInterrupt, which propagates out of run()'s while loop into its `finally:
    # self._cleanup()`.
    def _shutdown(self, sig=None, frame=None):
        self.running = False

    # Blocks (busy-polling every 1ms) until RtspReader (C++ side) publishes a frame
    # with a new frame_id, then copies it out of shared memory under the is_reading
    # flag so a concurrent write can't tear it. Returns (None, -1) if the C++ side
    # has set is_alive=0 (shutting down) — this is what ends run()'s main loop.
    def _get_latest_frame(self):
        is_alive, is_writing, is_reading = struct.unpack_from("BBB", self.buf_frames, 0)
        frame_id, _, _ = struct.unpack_from("<III", self.buf_frames, 4)

        if is_alive == 0:
            return None, -1

        while frame_id == self.last_processed_frame_id or is_writing == 1:
            time.sleep(0.001)
            is_writing = self.buf_frames[1]
            frame_id = struct.unpack_from("<I", self.buf_frames, 4)[0]

        try:
            self.buf_frames[2] = 1 # is_reading
            frame_shm = np.ndarray((self.HEIGHT, self.WIDTH), dtype=np.uint8, buffer=self.buf_frames, offset=self.HEADER_SIZE)
            frame_np = frame_shm.copy()
        finally:
            self.buf_frames[2] = 0
            self.last_processed_frame_id = frame_id

        return frame_np, frame_id

    # Thresholds matches by min_score, then buckets the survivors into a
    # rows x cols grid over the target image (by their mkpts1 position) and keeps
    # only the top per_cell (by score) in each cell — the same "spread matches across
    # the image instead of clustering" idea as ImageMatcher::gridFilterMatches on the
    # C++ side, just applied to LightGlue's output instead of SIFT's.
    def _filter_by_grid(self, mkpts0, mkpts1, mscores, rows=2, cols=3, per_cell=25, min_score=0.75):
        if len(mscores) == 0:
            return mkpts0, mkpts1, mscores
        
        # 1. Soft score thresholding
        valid_mask = mscores >= min_score
        if not np.any(valid_mask):
            return mkpts0[:0], mkpts1[:0], mscores[:0]
        
        mkpts0_v = mkpts0[valid_mask]
        mkpts1_v = mkpts1[valid_mask]
        mscores_v = mscores[valid_mask]
        
        # 2. Vectorized 2D Grid Cell Assignment
        cell_h, cell_w = self.superpointHeight / rows, self.superpointWidth / cols
        
        col_idx = np.clip((mkpts1_v[:, 0] / cell_w).astype(np.int64), 0, cols - 1)
        row_idx = np.clip((mkpts1_v[:, 1] / cell_h).astype(np.int64), 0, rows - 1)
        
        # Unique 1D index for each grid cell: id = row * cols + col
        cell_ids = row_idx * cols + col_idx
        
        selected_indices = []
        
        # 3. Fast extraction per cell
        for cell_id in range(rows * cols):
            in_cell = np.where(cell_ids == cell_id)[0]
            if len(in_cell) == 0:
                continue
        
            # Sort/select top N in cell
            if len(in_cell) > per_cell:
                # np.argpartition is faster than argsort for top-k selection
                cell_scores = mscores_v[in_cell]
                top_k_local = np.argpartition(cell_scores, -per_cell)[-per_cell:]
                # Sort the final top-k elements descending by score
                top_k_sorted = top_k_local[np.argsort(-cell_scores[top_k_local])]
                best = in_cell[top_k_sorted]
            else:
                best = in_cell
        
            selected_indices.append(best)
        
        if not selected_indices:
            return mkpts0_v[:0], mkpts1_v[:0], mscores_v[:0]
        
        idx = np.concatenate(selected_indices)
        return mkpts0_v[idx], mkpts1_v[idx], mscores_v[idx]

    def _compute_patch_covariances_numpy(self, img, keypoints, patch_size=5, eps=1e-3):
        """
        Estimates a 2D pixel covariance per keypoint from the local image-gradient
        structure tensor (same idea as ImageMatcher::compute_point_covariances in
        ImageMatcher.cpp), vectorized with NumPy instead of a per-pixel C++ loop.
        Returns an (N, 3) array of (xx, xy, yy) — the packed upper triangle of each
        2x2 covariance — which _write_matches() sends as-is; the C++ side
        (ImageMatcher::getAlignment) unpacks it back into a cv::Matx22f. Points too
        close to the image border to have a full patch get an identity-ish fallback
        covariance (1, 0, 1).
        """
        gy, gx = np.gradient(img)
        Ixx, Iyy, Ixy = gx * gx, gy * gy, gx * gy
        half_w = patch_size // 2
        N = keypoints.shape[0]
        covs = np.zeros((N, 3), dtype=np.float32)

        kpts_int = np.round(keypoints).astype(np.int32)
        u, v = kpts_int[:, 0], kpts_int[:, 1]
        H_img, W_img = img.shape

        valid = (u >= half_w) & (u < W_img - half_w) & (v >= half_w) & (v < H_img - half_w)
        valid_indices = np.where(valid)[0]

        for idx in valid_indices:
            x, y = u[idx], v[idx]
            sum_xx = Ixx[y - half_w : y + half_w + 1, x - half_w : x + half_w + 1].sum()
            sum_yy = Iyy[y - half_w : y + half_w + 1, x - half_w : x + half_w + 1].sum()
            sum_xy = Ixy[y - half_w : y + half_w + 1, x - half_w : x + half_w + 1].sum()

            H_00, H_01, H_11 = sum_xx + eps, sum_xy, sum_yy + eps
            det = H_00 * H_11 - H_01 * H_01

            covs[idx, 0] = H_11 / det
            covs[idx, 1] = -H_01 / det
            covs[idx, 2] = H_00 / det

        covs[~valid] = [1.0, 0.0, 1.0]
        return covs

    # Main loop: pull the latest frame, run SuperPoint on it, match against the
    # cached target features with LightGlue, filter/rescale the matches, estimate a
    # per-match covariance, and publish the result. Runs until _get_latest_frame()
    # signals the C++ side has shut down, then always calls _cleanup() (including on
    # an unhandled exception or Ctrl+C).
    def run(self):
        try:
            while self.running:
                cur_image_, frame_id = self._get_latest_frame()
                if cur_image_ is None or frame_id == -1:
                    break
                cur_image = cv2.resize(cur_image_, (self.superpointWidth, self.superpointHeight), interpolation=cv2.INTER_AREA)
                # Preprocess frame to match model expectations [1, 1, H, W]
                cur_tensor = cur_image.astype(np.float32) / 255.0
                cur_tensor = cur_tensor[None, None]
                # Step 1: extract current frame features
                (kpts0, desc0, scores0, cur_mask,
                                            cur_num_keypoints) = self.pad_superpoint(*self.sp_session.run(None, {'image': cur_tensor}))
                # Step 2: match against cached target features
                outputs = self.lg_session.run(None, {
                    'kpts0':   kpts0,   'desc0':   desc0,   'scores0': scores0,
                    'kpts1':   self.target_kpts, 'desc1':   self.target_desc, 'scores1': self.target_scores })

                matches0 = outputs[0][0]   # [N]
                scores0  = outputs[2][0]   # [N]

                valid = (
                    (np.arange(len(matches0)) < cur_num_keypoints) &
                    (matches0 >= 0) &
                    (matches0 < self.target_num_keypoints)
                )

                idx0 = np.where(valid)[0]
                idx1 = matches0[valid]
                scores = scores0[valid]

                mkpts0 = kpts0[0][idx0]
                mkpts1 = self.target_kpts[0][idx1]

                # scale back to original resolution
                mkpts0[:, 0] *= self.ratio_width   # x
                mkpts0[:, 1] *= self.ratio_height   # y
                mkpts1[:, 0] *= self.ratio_width
                mkpts1[:, 1] *= self.ratio_height

                mkpts0_filtered, mkpts1_filtered, scores_filtered = self._filter_by_grid(mkpts0, mkpts1, scores)
                covariances = self._compute_patch_covariances_numpy(cur_image_, mkpts0_filtered)
                self._write_matches(mkpts0_filtered, mkpts1_filtered, scores_filtered, covariances)

        finally:
            self._cleanup()

    # Serializes up to MAX_KP_MATCHES matches into the sp_sg_matches shared-memory
    # segment: [is_alive][py_writing][is_reading][frame_id:u32][N:u32][mkpts0 Nx2f32]
    # [mkpts1 Nx2f32][mscores Nf32][covariances Nx3f32], guarded by the py_writing/
    # is_reading handshake so SPSGReader (Matches.hpp, C++ side) never reads a torn
    # payload. This layout MUST stay in sync with SPSGReader::parse() in Matches.hpp.
    def _write_matches(self, mkpts0, mkpts1, mscores, covariances):
        N = min(len(mscores), self.MAX_KP_MATCHES)
        mkpts0 = mkpts0[:N].astype(np.float32)
        mkpts1 = mkpts1[:N].astype(np.float32)
        mscores = mscores[:N].astype(np.float32)
        covariances = covariances[:N].astype(np.float32)

        while self.buf[2] == 1:
            time.sleep(1e-5)

        self.buf[1] = 1 # py_writing
        offset = 3
        frame_id = struct.unpack_from('I', self.buf, offset)[0] + 1
        struct.pack_into('I', self.buf, offset, frame_id); offset += 4
        struct.pack_into('I', self.buf, offset, N); offset += 4

        self.buf[offset:offset + N*8] = mkpts0.tobytes(); offset += N*8
        self.buf[offset:offset + N*8] = mkpts1.tobytes(); offset += N*8
        self.buf[offset:offset + N*4] = mscores.tobytes(); offset += N*4
        self.buf[offset:offset + N*12] = covariances.tobytes()
        self.buf[1] = 0

    # Signals is_alive=0 on the matches segment (so SPSGReader knows to stop) and
    # releases both shared-memory handles.
    def _cleanup(self):
        self.shm_frames.close()
        self.buf[0] = 0
        time.sleep(0.1)
        self.shm.close()
        self.shm.unlink()
        print("[FeatureMatcher] Cleaned up.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--target', default="/home/user/drone_repositioning/targets/Capture_001.png")
    args = parser.parse_args()

    matcher = FeatureMatcherONNX(target_image_path=args.target)
    matcher.run()