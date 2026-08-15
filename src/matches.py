import torch
from lightglue import LightGlue, SuperPoint
from lightglue.utils import rbd, load_image
import cv2
import struct
import numpy as np
from multiprocessing import shared_memory
import time
import argparse
import signal
import sys

class FeatureMatcher:
    # MAX_KP_MATCHES   = 1024
    # SHM_NAME_MATCHES = "sp_sg_matches"
    # SHM_SIZE_MATCHES = 1 + 1 + 1 + 4 + 4  + MAX_KP_MATCHES * (2 + 2 + 1 + 3) * 4 # alive + python + c++ writing flag + frame id + header + covariances + kpts0 + kpts1 + scores

    def __init__(self, target_image_path, shm_name="single_frame_shm"):
        self.running = True

        # ── Matches shared memory ────────────────────────────────────────────────
        self.SHM_NAME_MATCHES = "sp_sg_matches"
        self.MAX_KP_MATCHES = 1024
        self.SHM_SIZE_MATCHES = 1 + 1 + 1 + 4 + 4 + self.MAX_KP_MATCHES * (2 + 2 + 1 + 3) * 4
        
        # Try to attach to existing, else create
        try:
            self.shm = shared_memory.SharedMemory(name=self.SHM_NAME_MATCHES)
            print(f"[Attached] {self.SHM_NAME_MATCHES}")
        except FileNotFoundError:
            self.shm = shared_memory.SharedMemory(name=self.SHM_NAME_MATCHES, create=True, size=self.SHM_SIZE_MATCHES)
            print(f"[Created] {self.SHM_NAME_MATCHES} ({self.SHM_SIZE_MATCHES} bytes)")
        
        self.buf = self.shm.buf
        self.buf[0] = 1  # alive flag
    

        # # ── Shared memory ────────────────────────────────────────────────
        # self.shm = shared_memory.SharedMemory(name=self.SHM_NAME_MATCHES, create=True, size=self.SHM_SIZE_MATCHES)
        # self.buf = self.shm.buf
        # self.buf[:self.SHM_SIZE_MATCHES] = bytes(self.SHM_SIZE_MATCHES)
        # self.buf[0] = 1  # alive

        # ── Shared memory frame ────────────────────────────────────────────────
        self.shm_name = shm_name
        self.WIDTH = 1920
        self.HEIGHT = 1080
        self.FRAME_SIZE = self.WIDTH * self.HEIGHT
        
        # Header: is_alive(1), is_writing(1), is_reading(1), padding(1), frame_id(4), width(4), height(4)
        self.HEADER_SIZE = 1 + 1 + 1 + 1 + 4 + 4 + 4
        self.SHM_SIZE_FRAME = self.HEADER_SIZE + self.FRAME_SIZE
        # Connect to existing C++ Shared Memory
        self.shm_frames = shared_memory.SharedMemory(name=shm_name)
        self.last_processed_frame_id = -1
        self.buf_frames = self.shm_frames.buf
        # self.buf_frames[:self.SHM_SIZE_FRAME] = bytes(self.SHM_SIZE_FRAME)


        # ── Models ───────────────────────────────────────────────────────
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.extractor = SuperPoint(max_num_keypoints=2048).eval().to(self.device)
        self.matcher   = LightGlue(features='superpoint').eval().to(self.device)

        # ── Target image ─────────────────────────────────────────────────
        target = load_image(target_image_path).to(self.device)
        with torch.no_grad():
            self.target_features = self.extractor.extract(target)

        # # ── Camera ───────────────────────────────────────────────────────
        # src = int(camera) if str(camera).isdigit() else camera
        # self.cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
        # self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        # self.cap.set(cv2.CAP_PROP_FPS, 30)
        # if not self.cap.isOpened():
        #     raise RuntimeError(f"Cannot open camera: {camera}")

        # ── Signals ──────────────────────────────────────────────────────
        signal.signal(signal.SIGINT,  self._shutdown)
        signal.signal(signal.SIGTERM, self._shutdown)

        print(f"[FeatureMatcher] device={self.device} SHM_SIZE={self.SHM_SIZE_MATCHES} target={target_image_path}")

    def _shutdown(self, sig=None, frame=None):
        self.running = False

    def _frame_to_tensor(self):
        gray, frame_id = self._get_latest_frame()
        if gray is None:
            return None, frame_id
        tensor = torch.from_numpy(gray).float() / 255.0
        return tensor.unsqueeze(0).unsqueeze(0).to(self.device), frame_id

    def _filter_by_grid(self, mkpts0, mkpts1, mscores, rows=2, cols=3, per_cell=50, min_score=0.75):
        if len(mscores) == 0:
            return mkpts0, mkpts1, mscores

        # 1. Soft score thresholding (use a lower threshold so low-texture cells still get features)
        valid_mask = mscores >= min_score
        if not torch.any(valid_mask):
            return mkpts0[:0], mkpts1[:0], mscores[:0]

        mkpts0_v = mkpts0[valid_mask]
        mkpts1_v = mkpts1[valid_mask]
        mscores_v = mscores[valid_mask]

        # 2. Vectorized 2D Grid Cell Assignment
        cell_h, cell_w = self.HEIGHT / rows, self.WIDTH / cols

        col_idx = torch.clamp((mkpts1_v[:, 0] / cell_w).long(), min=0, max=cols - 1)
        row_idx = torch.clamp((mkpts1_v[:, 1] / cell_h).long(), min=0, max=rows - 1)

        # Unique 1D index for each grid cell: id = row * cols + col
        cell_ids = row_idx * cols + col_idx

        selected_indices = []

        # 3. Fast extraction per cell
        for cell_id in range(rows * cols):
            in_cell = torch.where(cell_ids == cell_id)[0]
            if len(in_cell) == 0:
                continue

            # Sort top N in cell
            if len(in_cell) > per_cell:
                top_k = torch.topk(mscores_v[in_cell], k=per_cell).indices
                best = in_cell[top_k]
            else:
                best = in_cell

            selected_indices.append(best)

        if not selected_indices:
            return mkpts0_v[:0], mkpts1_v[:0], mscores_v[:0]

        idx = torch.cat(selected_indices)
        return mkpts0_v[idx], mkpts1_v[idx], mscores_v[idx]


    def _filter_by_grid_old(self, mkpts0, mkpts1, mscores, rows=2, cols=3, per_cell=200):
        selected = []
        cell_h, cell_w = self.HEIGHT / rows, self.WIDTH / cols

        for r in range(rows):
            for c in range(cols):
                x_min, x_max = c * cell_w, (c + 1) * cell_w
                y_min, y_max = r * cell_h, (r + 1) * cell_h
                mask    = (mkpts1[:, 0] >= x_min) & (mkpts1[:, 0] < x_max) & \
                          (mkpts1[:, 1] >= y_min) & (mkpts1[:, 1] < y_max) & \
                          (mscores > 0.85)
                in_cell = torch.where(mask)[0]
                if len(in_cell) == 0:
                    continue
                best = in_cell[torch.argsort(mscores[in_cell], descending=True)[:per_cell]]
                selected.append(best)

        if not selected:
            return mkpts0, mkpts1, mscores
        idx = torch.cat(selected)
        return mkpts0[idx], mkpts1[idx], mscores[idx]

    def _get_latest_frame(self):
        # Unpack flags from header: (is_alive, is_writing, is_reading, frame_id)
        # is_alive, is_writing, is_reading, frame_id = struct.unpack("=?BBI", self.shm_frames.buf[0:7])
        is_alive, is_writing, is_reading = struct.unpack_from("BBB", self.buf_frames, 0)
        frame_id, width, height = struct.unpack_from("<III", self.buf_frames, 4)
        # is_alive   = self.buf_frames[0]  # Read byte 0
        # is_writing = self.buf_frames[1]  # Read byte 1
        # is_reading = self.buf_frames[2]  # Read byte 2
        # frame_id   = struct.unpack_from('I', self.buf_frames, 4)[0]  # Read frame ID

        # 1. Check if memory is alive
        if is_alive == 0:
            return None, -1

        # 2. Check if frame is new or if C++ is currently writing
        while frame_id == self.last_processed_frame_id or is_writing == 1:
            # return None, frame_id
            time.sleep(0.001)  # Sleep briefly to avoid busy waiting
            is_writing = self.buf_frames[1]  # Update is_writing flag
            frame_id = struct.unpack_from("<I", self.buf_frames, 4)

        try:
            # 3. Set is_reading flag to 1 so C++ doesn't overwrite while copying to GPU
            # struct.pack_into("I", self.buf_frames, 8, 1)
            self.buf_frames[2] = 1

            # Map raw buffer directly into zero-copy NumPy array
            frame_shm = np.ndarray((self.HEIGHT, self.WIDTH), dtype=np.uint8, 
                                  buffer=self.buf_frames, offset=self.HEADER_SIZE)
            frame_np = frame_shm.copy()  # Copy to avoid issues with shared memory
              # Return a copy to avoid issues with shared memory
            # Send tensor to GPU for SuperPoint/LightGlue
            # gpu_tensor = torch.from_numpy(frame_shm).cuda().float() / 255.0
            
            # self.last_processed_frame_id = frame_id
            # return gpu_tensor.unsqueeze(0).unsqueeze(0), frame_id

        finally:
            # 4. Always reset is_reading flag back to 0
            # struct.pack_into("I", self.buf_frames, 8, 0)
            self.buf_frames[2] = 0
            self.last_processed_frame_id = frame_id

        return frame_np, frame_id


    def _write_matches(self, mkpts0, mkpts1, mscores, covariances):
        N       = min(len(mscores), self.MAX_KP_MATCHES)
        mkpts0  = mkpts0.cpu().numpy().astype(np.float32)[:N]
        mkpts1  = mkpts1.cpu().numpy().astype(np.float32)[:N]
        mscores = mscores.cpu().numpy().astype(np.float32)[:N]
        covariances = covariances.cpu().numpy().astype(np.float32)[:N]  

        # Interleave data into a contiguous structured array for fast SHM copy
        # Memory layout per match: [u0, v0, u1, v1, score, cov_xx, cov_xy, cov_yy]
        # packed_data = np.hstack([mkpts0, mkpts1, mscores[:, None], mcovariances])

        while self.buf[2] == 1:      # wait for C++ to finish reading
            time.sleep(1e-5)

        self.buf[1] = 1              # py_writing
        offset   = 3
        frame_id = struct.unpack_from('I', self.buf, offset)[0] + 1
        struct.pack_into('I', self.buf, offset, frame_id);   offset += 4
        struct.pack_into('I', self.buf, offset, N);          offset += 4
        # raw_bytes = packed_data.tobytes()
        # self.buf[offset : offset + N*32] = raw_bytes
        self.buf[offset:offset + N*8] = mkpts0.tobytes();   offset += N*8
        self.buf[offset:offset + N*8] = mkpts1.tobytes();   offset += N*8
        self.buf[offset:offset + N*4] = mscores.tobytes();  offset += N*4
        self.buf[offset:offset + N*12] = covariances.tobytes()  
        self.buf[1] = 0              # done

    def _compute_patch_covariances(
        self, image_tensor: torch.Tensor, keypoints: torch.Tensor, patch_size=5, eps=1e-3
    ):
        """Calculates PNEC anisotropic covariances directly on CPU/GPU.

        image_tensor: (1, 1, H, W) normalized image [0, 1]
        keypoints: (N, 2) [u, v] coordinates in target image (mkpts1)
        Returns: (N, 3) float tensor [cov_xx, cov_xy, cov_yy]
        """
        img = image_tensor.squeeze()  # (H, W)

        # 1. Image Spatial Gradients
        gy, gx = torch.gradient(img)

        Ixx = gx * gx
        Iyy = gy * gy
        Ixy = gx * gy

        half_w = patch_size // 2
        N = keypoints.shape[0]
        covs = torch.zeros((N, 3), device=self.device)

        kpts_int = torch.round(keypoints).long()
        u, v = kpts_int[:, 0], kpts_int[:, 1]

        H_img, W_img = img.shape
        valid = (
            (u >= half_w) & (u < W_img - half_w) & (v >= half_w) & (v < H_img - half_w)
        )

        valid_indices = torch.where(valid)[0]

        for idx in valid_indices:
            x, y = u[idx], v[idx]

            # Sum inner products across 5x5 window
            sum_xx = Ixx[
                y - half_w : y + half_w + 1, x - half_w : x + half_w + 1
            ].sum()
            sum_yy = Iyy[
                y - half_w : y + half_w + 1, x - half_w : x + half_w + 1
            ].sum()
            sum_xy = Ixy[
                y - half_w : y + half_w + 1, x - half_w : x + half_w + 1
            ].sum()

            # Structure Tensor H with regularizer
            H_00 = sum_xx + eps
            H_01 = sum_xy
            H_11 = sum_yy + eps

            # 2x2 Matrix Inversion: inv(H) = (1 / det) * [H11, -H01; -H01, H00]
            det = H_00 * H_11 - H_01 * H_01

            covs[idx, 0] = H_11 / det  # cov_xx
            covs[idx, 1] = -H_01 / det  # cov_xy
            covs[idx, 2] = H_00 / det  # cov_yy

        # Isotropic identity prior fallback for boundary keypoints
        covs[~valid, 0] = 1.0
        covs[~valid, 1] = 0.0
        covs[~valid, 2] = 1.0

        return covs
    
    def run(self):
        try:
            while self.running:
                cur_image, frame_id = self._frame_to_tensor()
                if cur_image is None or frame_id == -1:
                    break  # Exit if SHM is dead
                with torch.no_grad():
                    cur_features = self.extractor.extract(cur_image)
                    result       = self.matcher({'image0': cur_features,
                                                 'image1': self.target_features})

                cur_f, tgt_f, res = rbd(cur_features), rbd(self.target_features), rbd(result)

                mkpts0  = cur_f['keypoints'][res['matches'][:, 0]]
                mkpts1  = tgt_f['keypoints'][res['matches'][:, 1]]
                scores  = res['scores']

                mkpts0, mkpts1, scores = self._filter_by_grid(mkpts0, mkpts1, scores)
                # print(f"[FeatureMatcher] matches: {len(scores)}")

                covariances = self._compute_patch_covariances(cur_image, mkpts0)

                self._write_matches(mkpts0, mkpts1, scores, covariances)

                if self.buf[0] == 0:  # C++ signaled stop
                    print("[FeatureMatcher] C++ signaled stop.")
                    break

        finally:
            self._cleanup()

        return

    def _cleanup(self):
        self.shm_frames.close()
        self.shm_frames.unlink()
        self.buf[0] = 0   # signal shutdown to C++
        time.sleep(0.1)
        self.shm.close()
        self.shm.unlink()
        
        print("[FeatureMatcher] Cleaned up.")
        return


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--target', default="/home/user/drone_repositioning/targets/target_out1.jpg", #required=True,
                        help='Path to target image')
    args = parser.parse_args()

    matcher = FeatureMatcher( target_image_path=args.target)
    matcher.run()