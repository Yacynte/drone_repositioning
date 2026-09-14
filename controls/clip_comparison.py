import re
from pathlib import Path

import torch
import clip
import yaml
from PIL import Image
import torch.nn.functional as F


# ============================================================
# Configuration
# ============================================================

GT_DIR = Path("data/imagesGT3")
RESULTS_DIR = Path("data/results3_0")
OUTPUT_FILE = Path("data/results3_0/similarity_results.yaml")

WEATHER_FOLDERS = ["rain", "sunny", "snow"]

IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg", ".bmp", ".webp"}


# ============================================================
# Device and CLIP model
# ============================================================

device = "cuda" if torch.cuda.is_available() else "cpu"

print(f"Using device: {device}")

model, preprocess = clip.load("ViT-B/32", device=device)
model.eval()


# ============================================================
# Helper functions
# ============================================================

def get_gt_id(filename):
    """
    Extract the integer ID from:
        GT1_Capture_20260827_132610.png

    Returns:
        1
    """

    match = re.match(r"^GT(\d+)_Capture_", filename)

    if match is None:
        return None

    return int(match.group(1))


def get_final_id(filename):
    """
    Extract the integer ID from:
        Final1_Capture_20260827_141931.png

    Returns:
        1
    """

    match = re.match(r"^Final(\d+)_Capture_", filename)

    if match is None:
        return None

    return int(match.group(1))


def load_image(path):
    """
    Load an image and convert it to RGB.
    """

    return preprocess(
        Image.open(path).convert("RGB")
    ).unsqueeze(0).to(device)


def scene_similarity(path1, path2):
    """
    Calculate cosine similarity between two images using CLIP.
    """

    img1 = load_image(path1)
    img2 = load_image(path2)

    with torch.no_grad():
        f1 = model.encode_image(img1)
        f2 = model.encode_image(img2)

        f1 = F.normalize(f1, dim=-1)
        f2 = F.normalize(f2, dim=-1)

        similarity = (f1 @ f2.T).item()

    return similarity


# ============================================================
# Find ground-truth images
# ============================================================

print("\nScanning ground-truth images...")

gt_images = {}

for path in GT_DIR.iterdir():

    if not path.is_file():
        continue

    if path.suffix.lower() not in IMAGE_EXTENSIONS:
        continue

    image_id = get_gt_id(path.name)

    if image_id is None:
        print(f"WARNING: Could not parse GT ID: {path}")
        continue

    if image_id in gt_images:
        raise RuntimeError(
            f"Duplicate GT ID {image_id}:\n"
            f"  {gt_images[image_id]}\n"
            f"  {path}"
        )

    gt_images[image_id] = path


print(f"Found {len(gt_images)} GT images.")


# ============================================================
# Find result images
# ============================================================

print("\nScanning result images...")

final_images = {}

for weather in WEATHER_FOLDERS:

    weather_dir = RESULTS_DIR / weather

    if not weather_dir.exists():
        print(f"WARNING: Weather directory does not exist: {weather_dir}")
        continue

    for path in weather_dir.iterdir():

        if not path.is_file():
            continue

        if path.suffix.lower() not in IMAGE_EXTENSIONS:
            continue

        image_id = get_final_id(path.name)

        if image_id is None:
            print(f"WARNING: Could not parse Final ID: {path}")
            continue

        if image_id in final_images:
            raise RuntimeError(
                f"Duplicate Final ID {image_id}:\n"
                f"  {final_images[image_id]['path']}\n"
                f"  {path}"
            )

        final_images[image_id] = {
            "path": path,
            "weather": weather,
        }


print(f"Found {len(final_images)} result images.")


# ============================================================
# Check matching IDs
# ============================================================

gt_ids = set(gt_images.keys())
final_ids = set(final_images.keys())

missing_results = sorted(gt_ids - final_ids)
missing_gt = sorted(final_ids - gt_ids)

if missing_results:
    print("\nWARNING: GT images without matching Final images:")
    for image_id in missing_results:
        print(f"  ID {image_id}")

if missing_gt:
    print("\nWARNING: Final images without matching GT images:")
    for image_id in missing_gt:
        print(f"  ID {image_id}")


matching_ids = sorted(gt_ids & final_ids)

print(f"\nMatched image pairs: {len(matching_ids)}")


# ============================================================
# Calculate similarities
# ============================================================

results = []

print("\nCalculating CLIP similarities...\n")

for index, image_id in enumerate(matching_ids, start=1):

    gt_path = gt_images[image_id]

    final_info = final_images[image_id]
    final_path = final_info["path"]
    weather = final_info["weather"]

    print(
        f"[{index}/{len(matching_ids)}] "
        f"ID {image_id} | {weather}"
    )

    similarity = scene_similarity(
        gt_path,
        final_path
    )

    print(f"    Similarity: {similarity:.6f}")

    results.append({
        "id": image_id,

        "source": {
            "filename": gt_path.name,
            "path": str(gt_path),
            "weather": "ground_truth",
        },

        "target": {
            "filename": final_path.name,
            "path": str(final_path),
            "weather": weather,
        },

        "similarity": round(similarity, 6),
    })


# ============================================================
# Write YAML
# ============================================================

output = {
    "model": "ViT-B/32",
    "metric": "cosine_similarity",
    "source_directory": str(GT_DIR),
    "target_directory": str(RESULTS_DIR),
    "weather_conditions": WEATHER_FOLDERS,
    "number_of_pairs": len(results),
    "pairs": results,
}


with open(OUTPUT_FILE, "w", encoding="utf-8") as file:
    yaml.safe_dump(
        output,
        file,
        sort_keys=False,
        allow_unicode=True
    )


# ============================================================
# Summary
# ============================================================

print("\n" + "=" * 60)
print("Finished!")
print("=" * 60)

print(f"GT images:       {len(gt_images)}")
print(f"Final images:    {len(final_images)}")
print(f"Matched pairs:   {len(results)}")
print(f"Missing results: {len(missing_results)}")
print(f"Missing GT:      {len(missing_gt)}")

print(f"\nResults written to:")
print(f"  {OUTPUT_FILE}")