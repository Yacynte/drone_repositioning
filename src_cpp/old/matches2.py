import torch
from lightglue import LightGlue, SuperPoint
from lightglue.utils import rbd
import cv2
import struct
import numpy as np
from multiprocessing import shared_memory
import time

START = True
MAX_KP = 1024
SHM_NAME = "sp_sg_matches"
SHM_SIZE = 1 + 1 + 1 + 4 + 4 + MAX_KP * (2 + 2 + 1) * 4  # alive + python + c++ writing flag + frame id + header + kpts0 + kpts1 + scores

shm = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=SHM_SIZE)
buf = shm.buf
buf[:SHM_SIZE] = bytes(SHM_SIZE)
buf[0] = 1
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

extractor = SuperPoint(max_num_keypoints=2048).eval().to(DEVICE)
matcher   = LightGlue(features='superpoint').eval().to(DEVICE)

def write_matches(mkpts0, mkpts1, mscores):
    N = min(len(mscores), MAX_KP)
    
    # Move tensors to CPU and convert to numpy ONLY right before writing memory
    mkpts0  = mkpts0.cpu().numpy().astype(np.float32)[:N]
    mkpts1  = mkpts1.cpu().numpy().astype(np.float32)[:N]
    mscores = mscores.cpu().numpy().astype(np.float32)[:N]

    while buf[2] == 1:
        time.sleep(1e-5)
    buf[1] = 1
    offset = 3
    frame_id = struct.unpack_from('I', buf, offset)[0] + 1 
    struct.pack_into('I', buf, offset, frame_id);   offset += 4
    struct.pack_into('I', buf, offset, N);         offset += 4
    buf[offset:offset + N*8]  = mkpts0.tobytes();  offset += N*8
    buf[offset:offset + N*8]  = mkpts1.tobytes();  offset += N*8
    buf[offset:offset + N*4]  = mscores.tobytes()
    buf[1] = 0

def frame_to_tensor(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tensor = torch.from_numpy(gray).float() / 255.0             # normalize to [0,1]
    tensor = tensor.unsqueeze(0).unsqueeze(0)                   # (H,W) → (1,1,H,W)
    return tensor.to(DEVICE)

def filter_matches_by_grid(mkpts0, mkpts1, mscores, H, W, rows=2, cols=3, per_cell=200):
    """
    Rewritten using pure PyTorch to avoid CUDA->NumPy errors.
    """
    selected = []
    cell_h = H / rows
    cell_w = W / cols

    for r in range(rows):
        for c in range(cols):
            x_min, x_max = c * cell_w,  (c + 1) * cell_w
            y_min, y_max = r * cell_h,  (r + 1) * cell_h

            # Create a boolean mask on the GPU
            mask = (mkpts1[:, 0] >= x_min) & (mkpts1[:, 0] < x_max) & \
                   (mkpts1[:, 1] >= y_min) & (mkpts1[:, 1] < y_max)
            
            in_cell = torch.where(mask)[0]

            if len(in_cell) == 0:
                continue

            # Sort by score in descending order and grab top 'per_cell'
            cell_scores = mscores[in_cell]
            best_idx = torch.argsort(cell_scores, descending=True)[:per_cell]
            best = in_cell[best_idx]
            
            selected.append(best)

    if not selected:
        return mkpts0, mkpts1, mscores   # fallback: send everything

    # Concatenate using torch instead of numpy
    idx = torch.cat(selected)
    return mkpts0[idx], mkpts1[idx], mscores[idx]


prev_image = None
prev_features = None

idx = 0
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

print(f"SHM_SIZE={SHM_SIZE} MAX_KP={MAX_KP}")
try:
    while START:
        ret, img = cap.read()
        if not ret or img is None:
            print("Camera failed, trying video1...")
            cap.release()  # Release the broken camera first
            cap = cv2.VideoCapture(1, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            ret, img = cap.read()

        if not ret or img is None:
            print("Both cameras failed")
            break
        
        if prev_image is None or prev_features is None:
            prev_image = frame_to_tensor(img)
            with torch.no_grad():
                prev_features = extractor.extract(prev_image)
            idx += 1
            continue

        cur_image = frame_to_tensor(img)

        with torch.no_grad():
            cur_features = extractor.extract(cur_image)
            result = matcher({'image0': prev_features, 'image1': cur_features})

        num_kp0 = prev_features['keypoints'].shape[1]
        num_kp1 = cur_features['keypoints'].shape[1]
        print(f"Keypoints found -> prev: {num_kp0}, cur: {num_kp1}")
        cv2.imwrite(f"debug_img_{idx}.jpg", img)
        save_features = cur_features.copy()
        
        prev_features, cur_features, result = rbd(prev_features), rbd(cur_features), rbd(result)

        prev_matches_  = prev_features['keypoints'][result['matches'][:, 0]]   # (N, 2)
        cur_matches_  = cur_features['keypoints'][result['matches'][:, 1]]   # (N, 2)
        scores_ = result['scores']                                # (N,)
        h, w, _ = img.shape
        
        prev_matches, cur_matches, scores = filter_matches_by_grid(prev_matches_, cur_matches_, scores_, h, w)
        print(f"matches unfiltered: {len(scores_)}, filtered: {len(scores)}")
        
        write_matches(mkpts0=prev_matches, mkpts1=cur_matches, mscores=scores)

        prev_features = save_features
        if idx > 10:
            START = False
        print(f"Wrote image index: {idx}")
        idx += 1

finally:
    buf[0] = 0  # signal C++ we're shutting down
    time.sleep(1)  # give C++ time to see it
    shm.close()
    shm.unlink()
    cap.release()