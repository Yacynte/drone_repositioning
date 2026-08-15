import torch
import clip
from pathlib import Path
from PIL import Image
import torch.nn.functional as F

device = "cuda" if torch.cuda.is_available() else "cpu"

model, preprocess = clip.load("ViT-B/32", device=device)
model.eval()

motion_int = 0
repo_root = Path.cwd()
gt_imgs = sorted(list(repo_root.glob(f'groundTruths_pnec/clear/Capture_*.png')))
results_imgs = sorted(list(repo_root.glob(f'results_pnec/clear/Results_Capture_*.png')))

def scene_similarity(path1, path2):
    img1 = preprocess(Image.open(path1)).unsqueeze(0).to(device)
    img2 = preprocess(Image.open(path2)).unsqueeze(0).to(device)

    with torch.no_grad():
        f1 = model.encode_image(img1)
        f2 = model.encode_image(img2)

    # Normalize then cosine similarity
    f1 = F.normalize(f1, dim=-1)
    f2 = F.normalize(f2, dim=-1)

    return (f1 @ f2.T).item()

score = scene_similarity(gt_imgs[motion_int], results_imgs[motion_int])
print(f"Scene similarity: {score:.4f}")