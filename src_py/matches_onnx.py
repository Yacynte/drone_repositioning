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
import os
import struct
import time
import argparse
import signal
import onnxruntime as ort

# The CUDA 13 / cuDNN 9 runtime comes as pip packages (onnxruntime-gpu[cuda,cudnn]) under
# site-packages/nvidia/..., which the dynamic loader does not search. Without this the
# CUDA provider fails to load (libcublasLt.so.13 not found) and ONNX Runtime silently
# falls back to CPU (LightGlue ~320 ms instead of a few ms).
try:
    ort.preload_dlls()
except Exception as e:  # older onnxruntime or no CUDA packages: CPU fallback still works
    print(f"onnxruntime.preload_dlls() failed: {e}")
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
        # Payload, then a fixed-offset timing block (must match SPSGReader in
        # include/Matches.hpp): uint32 cam_frame_id, uint32 reserved, float64 t_frame,
        # float64 t_done (wall-clock seconds).
        self.TIMING_OFFSET = 1 + 1 + 1 + 4 + 4 + self.MAX_KP_MATCHES * (2 + 2 + 1 + 3) * 4
        self.SHM_SIZE_MATCHES = self.TIMING_OFFSET + 4 + 4 + 8 + 8

        try:
            self.shm = shared_memory.SharedMemory(name=self.SHM_NAME_MATCHES)
            if self.shm.size < self.SHM_SIZE_MATCHES:   # leftover segment from an older layout
                self.shm.close()
                self.shm.unlink()
                raise FileNotFoundError
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
        # What ONNX Runtime actually uses: a failed CUDA provider silently drops to CPU,
        # which the requested list would not show.
        print(f"providers for {os.path.basename(model_path)}: requested {providers}, "
              f"in use {session.get_providers()}")

        return session


    # Sub-pixel refinement of the current-frame match positions. SuperPoint runs at
    # 640x480 and returns integer keypoints, so scaled back to full resolution every
    # point sits on a 3 x 2.25 px grid (~1.1 px of quantization noise, most of the noise
    # floor seen in the C++ logs). With the target point held fixed, pyramidal LK on the
    # full-resolution images moves the current point (initial guess: the SuperPoint
    # match) until the 21x21 neighbourhoods agree, to sub-pixel precision. A refinement
    # is kept only if LK converged both ways, the backward track returns within
    # LK_FB_MAX_ERR px of the target point and the point moved less than LK_MAX_SHIFT px;
    # otherwise the original match is kept. Measured on run end images vs GT3/GT4:
    # inlier Sampson residual 1.06 -> 0.87 px and 0.75 -> 0.47 px, ~2 ms per frame.
    LK_WIN = (21, 21)
    LK_MAX_LEVEL = 1          # pyramid levels 0 and 1
    LK_FB_MAX_ERR = 0.5       # px, forward-backward round trip
    LK_MAX_SHIFT = 4.0        # px, SuperPoint guess is within ~1.5 px of the truth

    def _refine_lk(self, cur_full, mkpts0, mkpts1):
        if len(mkpts0) == 0:
            return mkpts0
        p0 = np.ascontiguousarray(mkpts0, dtype=np.float32).reshape(-1, 1, 2)
        p1 = np.ascontiguousarray(mkpts1, dtype=np.float32).reshape(-1, 1, 2)
        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        lk = dict(winSize=self.LK_WIN, maxLevel=self.LK_MAX_LEVEL, criteria=criteria,
                  flags=cv2.OPTFLOW_USE_INITIAL_FLOW)
        fwd, st_f, _ = cv2.calcOpticalFlowPyrLK(self.target_full, cur_full, p1, p0.copy(), **lk)
        bwd, st_b, _ = cv2.calcOpticalFlowPyrLK(cur_full, self.target_full, fwd, p1.copy(), **lk)
        fwd, bwd = fwd.reshape(-1, 2), bwd.reshape(-1, 2)
        ok = ((st_f.ravel() == 1) & (st_b.ravel() == 1)
              & (np.linalg.norm(bwd - mkpts1, axis=1) < self.LK_FB_MAX_ERR)
              & (np.linalg.norm(fwd - mkpts0, axis=1) < self.LK_MAX_SHIFT))
        return np.where(ok[:, None], fwd, mkpts0).astype(np.float32)

    def set_target(self, target_image_path: str):
        """Call this whenever the target image changes — no re-export needed."""
        img = cv2.imread(target_image_path, cv2.IMREAD_GRAYSCALE)
        # Full-resolution copy for the LK refinement of the matches (_refine_lk); the
        # matched points live in WIDTH x HEIGHT coordinates.
        self.target_full = img if img.shape == (self.HEIGHT, self.WIDTH) \
            else cv2.resize(img, (self.WIDTH, self.HEIGHT), interpolation=cv2.INTER_AREA)
        img = cv2.resize(img, (self.superpointWidth, self.superpointHeight))
        tensor = img.astype(np.float32) / 255.0
        tensor = tensor[None, None]  # (1,1,H,W)

        (self.target_kpts, self.target_desc, self.target_scores, self.target_mask, 
                            self.target_num_keypoints) = self.pad_superpoint(*self.sp_session.run(None, {'image': tensor}))

    def _select_keypoints(self, kpts, scores, budget, rows=4, cols=4):
        """
        Indices of at most `budget` keypoints, spread over a rows x cols grid of the
        SuperPoint image: each cell first gets its strongest keypoints up to an equal
        share of the budget, unused slots go to the strongest remaining keypoints
        anywhere. Returned strongest first. kpts [N, 2] in SuperPoint pixels, scores [N].
        """
        n = scores.shape[0]
        if n <= budget:
            return np.argsort(-scores)

        cell_h, cell_w = self.superpointHeight / rows, self.superpointWidth / cols
        col_idx = np.clip((kpts[:, 0] / cell_w).astype(np.int64), 0, cols - 1)
        row_idx = np.clip((kpts[:, 1] / cell_h).astype(np.int64), 0, rows - 1)
        cell_ids = row_idx * cols + col_idx

        quota = budget // (rows * cols)
        chosen = np.zeros(n, dtype=np.bool_)
        for cell_id in range(rows * cols):
            in_cell = np.where(cell_ids == cell_id)[0]
            if len(in_cell) == 0:
                continue
            best = in_cell[np.argsort(-scores[in_cell])[:quota]]
            chosen[best] = True

        # Fill the remaining budget with the strongest keypoints not yet chosen.
        remaining = budget - int(chosen.sum())
        if remaining > 0:
            rest = np.where(~chosen)[0]
            chosen[rest[np.argsort(-scores[rest])[:remaining]]] = True

        keep = np.where(chosen)[0]
        return keep[np.argsort(-scores[keep])]

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

        max_keypoints = self.MAX_KP_MATCHES
        # SuperPoint returns keypoints in raster order (top rows first), so keeping the
        # first MAX would only ever use the top ~40% of the image. Pick the budget by
        # score, spread over the image, strongest first.
        keep = self._select_keypoints(kpts[0], scores[0], max_keypoints)
        kpts, desc, scores = kpts[:, keep, :], desc[:, keep, :], scores[:, keep]
        n = kpts.shape[1]

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
    def _filter_by_grid(self, mkpts0, mkpts1, mscores, rows=2, cols=3, per_cell=50, min_score=0.75):
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
        # mkpts are already scaled back to full resolution (run() rescales before
        # calling this), so the cells must be sized on WIDTH x HEIGHT, not on the
        # SuperPoint resolution (that put ~80% of the image in the last column/row).
        cell_h, cell_w = self.HEIGHT / rows, self.WIDTH / cols
        
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
        2x2 structure tensor H (NOT its inverse) — which _write_matches() sends as-is;
        the C++ side (ImageMatcher::getAlignment) unpacks it and inverts it once into
        the covariance Sigma_2D = H^-1. Points too close to the image border to have a
        full patch get an identity fallback H = (1, 0, 1).
        """
        # Gradients are computed only inside the patches (each fetched with a 1 px
        # margin, all at once via fancy indexing): central differences there equal
        # np.gradient of the whole image, which took ~16 ms per 1920x1080 frame on its
        # own (gradient + full-image products) for ~200 5x5 patches.
        half_w = patch_size // 2
        N = keypoints.shape[0]
        covs = np.empty((N, 3), dtype=np.float32)
        covs[:] = [1.0, 0.0, 1.0]        # identity fallback for points too close to the border

        kpts_int = np.round(keypoints).astype(np.int32)
        u, v = kpts_int[:, 0], kpts_int[:, 1]
        H_img, W_img = img.shape

        valid = (u >= half_w) & (u < W_img - half_w) & (v >= half_w) & (v < H_img - half_w)
        # Patches with a full 1 px margin inside the image: vectorized central differences.
        inner = (valid & (u >= half_w + 1) & (u < W_img - half_w - 1)
                 & (v >= half_w + 1) & (v < H_img - half_w - 1))
        idx = np.where(inner)[0]
        if len(idx):
            off = np.arange(-half_w - 1, half_w + 2)
            win = img[v[idx, None, None] + off[None, :, None],
                      u[idx, None, None] + off[None, None, :]].astype(np.float64)
            gx = (win[:, 1:-1, 2:] - win[:, 1:-1, :-2]) * 0.5
            gy = (win[:, 2:, 1:-1] - win[:, :-2, 1:-1]) * 0.5
            # Send the structure tensor H itself; the C++ side (getAlignment) inverts
            # it once to get Sigma_2D = H^-1. (Sending H^-1 here made C++ invert twice.)
            covs[idx, 0] = (gx * gx).sum(axis=(1, 2)) + eps
            covs[idx, 1] = (gx * gy).sum(axis=(1, 2))
            covs[idx, 2] = (gy * gy).sum(axis=(1, 2)) + eps

        # Patches touching the image edge (rare): np.gradient on a small crop gives the
        # same one-sided differences at the edge as on the full image.
        for i in np.where(valid & ~inner)[0]:
            x, y = u[i], v[i]
            y0, y1 = max(y - half_w - 1, 0), min(y + half_w + 2, H_img)
            x0, x1 = max(x - half_w - 1, 0), min(x + half_w + 2, W_img)
            gy_c, gx_c = np.gradient(img[y0:y1, x0:x1])
            ry = slice(y - half_w - y0, y + half_w + 1 - y0)
            rx = slice(x - half_w - x0, x + half_w + 1 - x0)
            gx_p, gy_p = gx_c[ry, rx], gy_c[ry, rx]
            covs[i] = [(gx_p * gx_p).sum() + eps, (gx_p * gy_p).sum(), (gy_p * gy_p).sum() + eps]

        return covs

    # Main loop: pull the latest frame, run SuperPoint on it, match against the
    # cached target features with LightGlue, filter/rescale the matches, estimate a
    # per-match covariance, and publish the result. Runs until _get_latest_frame()
    # signals the C++ side has shut down, then always calls _cleanup() (including on
    # an unhandled exception or Ctrl+C).
    def run(self):
        try:
            while self.running:
                t_wait = time.time()
                cur_image_, frame_id = self._get_latest_frame()
                if cur_image_ is None or frame_id == -1:
                    break
                t_frame = time.time()
                tic = time.perf_counter()
                stages = {}
                def lap(name):
                    nonlocal tic
                    now = time.perf_counter()
                    stages[name] = (now - tic) * 1000.0
                    tic = now
                cur_image = cv2.resize(cur_image_, (self.superpointWidth, self.superpointHeight), interpolation=cv2.INTER_AREA)
                # Preprocess frame to match model expectations [1, 1, H, W]
                cur_tensor = cur_image.astype(np.float32) / 255.0
                cur_tensor = cur_tensor[None, None]
                lap("resize")
                # Step 1: extract current frame features
                (kpts0, desc0, scores0, cur_mask,
                                            cur_num_keypoints) = self.pad_superpoint(*self.sp_session.run(None, {'image': cur_tensor}))
                lap("superpoint")
                # Step 2: match against cached target features
                outputs = self.lg_session.run(None, {
                    'kpts0':   kpts0,   'desc0':   desc0,   'scores0': scores0,
                    'kpts1':   self.target_kpts, 'desc1':   self.target_desc, 'scores1': self.target_scores })

                lap("lightglue")
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
                lap("filter")
                mkpts0_filtered = self._refine_lk(cur_image_, mkpts0_filtered, mkpts1_filtered)
                lap("lk")
                covariances = self._compute_patch_covariances_numpy(cur_image_, mkpts0_filtered)
                lap("cov")
                match_id = self._write_matches(mkpts0_filtered, mkpts1_filtered, scores_filtered,
                                               covariances, frame_id, t_frame)
                lap("write")
                # One line per match set; the C++ launcher logs Python's stdout as [python].
                print(f"timing match_id={match_id} cam_frame={frame_id} n={len(scores_filtered)} "
                      f"wait_ms={(t_frame - t_wait) * 1000.0:.1f} "
                      + " ".join(f"{k}_ms={v:.1f}" for k, v in stages.items())
                      + f" total_ms={sum(stages.values()):.1f}", flush=True)

        finally:
            self._cleanup()

    # Serializes up to MAX_KP_MATCHES matches into the sp_sg_matches shared-memory
    # segment: [is_alive][py_writing][is_reading][frame_id:u32][N:u32][mkpts0 Nx2f32]
    # [mkpts1 Nx2f32][mscores Nf32][covariances Nx3f32], guarded by the py_writing/
    # is_reading handshake so SPSGReader (Matches.hpp, C++ side) never reads a torn
    # payload. This layout MUST stay in sync with SPSGReader::parse() in Matches.hpp.
    def _write_matches(self, mkpts0, mkpts1, mscores, covariances, cam_frame_id=0, t_frame=float("nan")):
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
        struct.pack_into('<IIdd', self.buf, self.TIMING_OFFSET, int(cam_frame_id), 0,
                         float(t_frame), time.time())
        self.buf[1] = 0
        return frame_id

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