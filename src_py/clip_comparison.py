import torch
import clip
from PIL import Image
import torch.nn.functional as F
from pathlib import Path

# Explicitly set device
# device = "cuda" if torch.cuda.is_available() else "cpu"
device = "cpu"
model, preprocess = clip.load("ViT-B/32", device=device)  # load model onto correct device
model.eval()

def scene_similarity(path1, path2):
    img1 = preprocess(Image.open(path1)).unsqueeze(0).to(device)  # move to device
    img2 = preprocess(Image.open(path2)).unsqueeze(0).to(device)  # move to device

    with torch.no_grad():
        f1 = model.encode_image(img1)
        f2 = model.encode_image(img2)

    f1 = F.normalize(f1, dim=-1)
    f2 = F.normalize(f2, dim=-1)

    return (f1 @ f2.T).item()

def main():
    repo_root = Path.cwd()
    print(f"🔍 Searching for log files in {repo_root}...")
    
    # Get MotionLog files (ground truth)
    motion_int = -1  # Specify the lab number for GT file
    # Look for GT (ground truth) file
    gt_int = 2
    motion_logs = sorted(list(repo_root.glob(f'results_pnec/clear/Results_Capture_*.png')))
    if not motion_logs:
        print("❌ No MotionLog files found in groundTruths/")
        return

    gt_logs = sorted(list(repo_root.glob(f'groundTruths_pnec/clear/Capture_*.png')))
    # gt_imgs = sorted(list(repo_root.glob(f'groundTruths_pnec/clear/Capture_*.png')))
    # results_imgs = sorted(list(repo_root.glob(f'results_pnec/clear/Results_Capture_*.png')))
    if gt_logs:
        motion_log_1_path = gt_logs[gt_int]  # Use the GT file
        # Get other motion logs for comparison
        # other_logs = sorted([f for f in motion_logs if 'GT' not in f.name])
        motion_log_2_path = motion_logs[motion_int] #if other_logs else None
        print(f"Loading ground truth (poses): {motion_log_1_path.name}")
        if motion_log_2_path:
            print(f"Loading Unreal Engine path (rates): {motion_log_2_path.name}")

    similarity = scene_similarity(motion_log_1_path, motion_log_2_path)

    print(f"Image similarity: {similarity}")


if __name__ == '__main__':
    main()