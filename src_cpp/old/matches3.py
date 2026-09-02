import torch
from lightglue import LightGlue, SuperPoint
from lightglue.utils import rbd, load_image
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

try:
    target_image = load_image("/home/user/drone_repositioning/targets/target_out1.jpg").to(DEVICE)
    with torch.no_grad():
        target_features_ = extractor.extract(target_image)
    # target_features = rbd(target_features)
except Exception as e:
    print(f"Exception during initialization: {e}")
    shm.close()
    shm.unlink()

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
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

        cur_image = frame_to_tensor(img)

        with torch.no_grad():
            cur_features = extractor.extract(cur_image)
            result = matcher({'image0': cur_features, 'image1': target_features_})

        # num_kp0 = prev_features['keypoints'].shape[1]
        # num_kp1 = target_features['keypoints'].shape[1]
        # print(f"Keypoints found -> prev: {num_kp0}, cur: {num_kp1}")
        # cv2.imwrite(f"debug_img_{idx}.jpg", img)        
        cur_features, result, target_features = rbd(cur_features), rbd(result), rbd(target_features_)

        cur_matches_  = cur_features['keypoints'][result['matches'][:, 0]]   # (N, 2)
        target_matches_  = target_features['keypoints'][result['matches'][:, 1]]   # (N, 2)
        scores_ = result['scores']                                # (N,)
        h, w, _ = img.shape
        
        cur_matches, target_matches, scores = filter_matches_by_grid(cur_matches_, target_matches_, scores_, h, w)
        print(f"matches unfiltered: {len(scores_)}, filtered: {len(scores)}")
        
        write_matches(mkpts0=cur_matches, mkpts1=target_matches, mscores=scores)

        if buf[0] == 0:  # Check if C++ has signaled to stop
            print("C++ signaled to stop, exiting.")
            START = False
            break


finally:
    buf[0] = 0  # signal C++ we're shutting down
    time.sleep(1)  # give C++ time to see it
    shm.close()
    shm.unlink()
    cap.release()