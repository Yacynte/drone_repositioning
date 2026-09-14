"""
ENGEL Stop Detector — Supervised GRU Classifier
================================================
Output: 3 booleans — stop_x, stop_y, stop_z

Labels are derived automatically from your ground truth data using
per-axis thresholds on pixel error and velocity.

Axis convention:
  X → lateral (left/right in image)
  Y → vertical (up/down in image)
  Z → depth (forward/backward)
"""
import pandas as pd
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report
import matplotlib.pyplot as plt
from pathlib import Path

# =============================================================================
# 1. LABEL GENERATION
#    Derives stop_x, stop_y, stop_z from your ground truth logged data.
#    "Stop on axis A" = pixel error on A is small AND velocity on A is small.
# =============================================================================

def generate_stop_labels(
    px_error_x: np.ndarray,   # shape (N,) — pixel error lateral
    px_error_y: np.ndarray,   # shape (N,) — pixel error vertical
    depth_error: np.ndarray,  # shape (N,) — GT depth error (meters) or Z pixel error
    vel_x: np.ndarray,        # shape (N,) — linear velocity X
    vel_y: np.ndarray,        # shape (N,) — linear velocity Y
    vel_z: np.ndarray,        # shape (N,) — linear velocity Z
    # --- Thresholds: tune these to your scenario ---
    px_thresh_xy: float = 5.0,    # pixels — lateral/vertical stop threshold
    depth_thresh: float = 0.1,    # meters — depth stop threshold
    vel_thresh: float = 0.05,     # m/s   — velocity stop threshold per axis
) -> np.ndarray:
    """
    Returns labels of shape (N, 3): [stop_x, stop_y, stop_z]
    Value is 1.0 (stop) or 0.0 (continue) per axis.
    
    Logic:
      stop_x = |px_error_x| < px_thresh  AND  |vel_x| < vel_thresh
      stop_y = |px_error_y| < px_thresh  AND  |vel_y| < vel_thresh
      stop_z = |depth_error| < depth_thresh  AND  |vel_z| < vel_thresh
    """
    stop_x = (np.abs(px_error_x) < px_thresh_xy) & (np.abs(vel_x) < vel_thresh)
    stop_y = (np.abs(px_error_y) < px_thresh_xy) & (np.abs(vel_y) < vel_thresh)
    stop_z = (np.abs(depth_error) < depth_thresh) & (np.abs(vel_z) < vel_thresh)

    labels = np.stack([stop_x, stop_y, stop_z], axis=1).astype(np.float32)
    return labels


# =============================================================================
# 2. DATASET
#    Wraps your feature matrix + labels into sliding windows for the GRU.
#    Each sample is a sequence of T consecutive timesteps.
# =============================================================================

class StopDataset(Dataset):
    def __init__(
        self,
        features: np.ndarray,   # shape (N, F) — N timesteps, F features
        labels: np.ndarray,     # shape (N, 3) — stop_x, stop_y, stop_z
        window_size: int = 10,  # how many past timesteps the GRU sees
    ):
        self.window_size = window_size
        # Pad the beginning so every timestep has a full window
        pad = np.repeat(features[:1], window_size - 1, axis=0)
        self.features = np.concatenate([pad, features], axis=0)
        self.labels = labels

    def __len__(self):
        return len(self.labels)

    def __getitem__(self, idx):
        # Window: [idx, idx + window_size) in padded array → label at idx
        x = self.features[idx: idx + self.window_size]          # (T, F)
        y = self.labels[idx]                                     # (3,)
        return torch.tensor(x, dtype=torch.float32), torch.tensor(y, dtype=torch.float32)


# =============================================================================
# 3. MODEL
#    GRU encoder → per-axis binary output
#    Each axis is independent at the output head (3 sigmoids).
# =============================================================================

class StopDetectorGRU(nn.Module):
    def __init__(
        self,
        input_size: int,        # number of features F
        hidden_size: int = 64,
        num_layers: int = 2,
        dropout: float = 0.3,
    ):
        super().__init__()
        self.gru = nn.GRU(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True,
            dropout=dropout if num_layers > 1 else 0.0,
        )
        # Shared trunk
        self.trunk = nn.Sequential(
            nn.Linear(hidden_size, 64),
            nn.ReLU(),
            nn.Dropout(dropout),
        )
        # Per-axis heads — each is independent
        self.head_x = nn.Linear(64, 1)  # stop_x
        self.head_y = nn.Linear(64, 1)  # stop_y
        self.head_z = nn.Linear(64, 1)  # stop_z

    def forward(self, x):
        """
        x: (B, T, F)
        returns: (B, 3) — raw logits for [stop_x, stop_y, stop_z]
        """
        _, h_n = self.gru(x)           # h_n: (num_layers, B, H)
        h = h_n[-1]                    # last layer hidden state: (B, H)
        trunk_out = self.trunk(h)      # (B, 64)

        logit_x = self.head_x(trunk_out)  # (B, 1)
        logit_y = self.head_y(trunk_out)  # (B, 1)
        logit_z = self.head_z(trunk_out)  # (B, 1)

        return torch.cat([logit_x, logit_y, logit_z], dim=1)  # (B, 3)

    def predict(self, x, threshold: float = 0.5):
        """Inference: returns (B, 3) booleans."""
        with torch.no_grad():
            logits = self.forward(x)
            probs = torch.sigmoid(logits)
            return (probs >= threshold).bool()


# =============================================================================
# 4. CLASS WEIGHTS
#    Stop frames are rare → without weighting, model just predicts "continue"
#    always and gets high accuracy. This fixes that.
# =============================================================================

def compute_pos_weights(labels: np.ndarray) -> torch.Tensor:
    """
    BCEWithLogitsLoss pos_weight = n_negative / n_positive per axis.
    Shape: (3,)
    """
    n = len(labels)
    pos = labels.sum(axis=0).clip(min=1)       # avoid div/0
    neg = n - labels.sum(axis=0)
    weights = neg / pos
    print(f"  Class balance  →  stop_x: {pos[0]:.0f}/{n}  "
          f"stop_y: {pos[1]:.0f}/{n}  stop_z: {pos[2]:.0f}/{n}")
    print(f"  Pos weights    →  {weights}")
    return torch.tensor(weights, dtype=torch.float32)


# =============================================================================
# 5. TRAINING LOOP
# =============================================================================

def train(
    model: nn.Module,
    train_loader: DataLoader,
    val_loader: DataLoader,
    pos_weights: torch.Tensor,
    epochs: int = 50,
    lr: float = 1e-3,
    device: str = "cpu",
):
    model.to(device)
    pos_weights = pos_weights.to(device)

    criterion = nn.BCEWithLogitsLoss(pos_weight=pos_weights)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, patience=5, factor=0.5, verbose=True
    )

    history = {"train_loss": [], "val_loss": []}
    best_val_loss = float("inf")
    best_state = None

    for epoch in range(1, epochs + 1):
        # --- Train ---
        model.train()
        train_loss = 0.0
        for x_batch, y_batch in train_loader:
            x_batch, y_batch = x_batch.to(device), y_batch.to(device)
            optimizer.zero_grad()
            logits = model(x_batch)
            loss = criterion(logits, y_batch)
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()
            train_loss += loss.item()

        train_loss /= len(train_loader)

        # --- Validate ---
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for x_batch, y_batch in val_loader:
                x_batch, y_batch = x_batch.to(device), y_batch.to(device)
                logits = model(x_batch)
                val_loss += criterion(logits, y_batch).item()
        val_loss /= len(val_loader)

        scheduler.step(val_loss)
        history["train_loss"].append(train_loss)
        history["val_loss"].append(val_loss)

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            best_state = {k: v.clone() for k, v in model.state_dict().items()}

        if epoch % 10 == 0:
            print(f"Epoch {epoch:3d}/{epochs}  "
                  f"train_loss={train_loss:.4f}  val_loss={val_loss:.4f}")

    # Restore best weights
    model.load_state_dict(best_state)
    print(f"\nBest val loss: {best_val_loss:.4f}")
    return history


# =============================================================================
# 6. EVALUATION
# =============================================================================

def evaluate(model: nn.Module, loader: DataLoader, device: str = "cpu",
             threshold: float = 0.5):
    model.eval()
    all_preds, all_labels = [], []
    with torch.no_grad():
        for x_batch, y_batch in loader:
            x_batch = x_batch.to(device)
            logits = model(x_batch)
            preds = (torch.sigmoid(logits) >= threshold).cpu().numpy()
            all_preds.append(preds)
            all_labels.append(y_batch.numpy())

    preds  = np.concatenate(all_preds,  axis=0)
    labels = np.concatenate(all_labels, axis=0)

    axis_names = ["stop_x", "stop_y", "stop_z"]
    for i, name in enumerate(axis_names):
        print(f"\n--- {name} ---")
        print(classification_report(labels[:, i], preds[:, i],
                                    target_names=["continue", "stop"]))


def plot_loss(history: dict):
    plt.figure(figsize=(8, 4))
    plt.plot(history["train_loss"], label="Train loss")
    plt.plot(history["val_loss"],   label="Val loss")
    plt.xlabel("Epoch")
    plt.ylabel("BCE Loss")
    plt.title("Stop Detector Training")
    plt.legend()
    plt.tight_layout()
    plt.savefig("training_loss.png", dpi=150)
    plt.show()


# =============================================================================
# 7. INFERENCE HELPER  (plug into your C++ pipeline via ONNX or direct call)
# =============================================================================

def export_onnx(model: nn.Module, input_size: int, window_size: int,
                path: str = "stop_detector.onnx"):
    """Export to ONNX so you can load it from C++ / ROS node."""
    model.eval()
    dummy = torch.zeros(1, window_size, input_size)
    torch.onnx.export(
        model, dummy, path,
        input_names=["features"],
        output_names=["logits"],
        dynamic_axes={"features": {0: "batch_size"}},
        opset_version=17,
    )
    print(f"Exported to {path}")


# =============================================================================
# 8. MAIN — Wire Everything Together
#    Replace the synthetic data section with your real logged CSV/numpy data.
# =============================================================================

if __name__ == "__main__":
    repo_root = Path(__file__).parent
    # ------------------------------------------------------------------
    # REPLACE THIS BLOCK with your real data loading
    # e.g.:
    #   import pandas as pd
    #   df = pd.read_csv("engel_log.csv")
    #   px_error_x  = df["px_err_x"].values
    #   px_error_y  = df["px_err_y"].values
    #   depth_error = df["depth_err_m"].values
    #   vel_x       = df["vel_x"].values
    #   ...etc
    # ------------------------------------------------------------------
    motion_logs = sorted(list(repo_root.glob('data/motionLog2_6/MotionLog_*.csv')))
    algo_logs = sorted(list(repo_root.glob('logs/AlgoLog_*.csv')))

    algo_log_df = pd.read_csv(algo_logs[0])
    motion_log_df = pd.read_csv(motion_logs)

    px_error_x  = algo_log_df["px_error_x"].values
    px_error_y  = algo_log_df["px_error_y"].values
    reproj_err = algo_log_df["repo_error"].values
    vel_x = algo_log_df["v_x"].values
    vel_y = algo_log_df["v_y"].values
    vel_z = algo_log_df["v_z"].values
    ang_vel_x    = algo_log_df["w_x"].values
    ang_vel_y    = algo_log_df["w_y"].values
    ang_vel_z    = algo_log_df["w_z"].values

    N = 2000  # number of timesteps in your log
    np.random.seed(42)

    # --- Raw features (13 features total) ---
    px_error_x   = np.random.randn(N) * 15
    px_error_y   = np.random.randn(N) * 15
    depth_error  = np.random.randn(N) * 0.5
    vel_x        = np.random.randn(N) * 0.3
    vel_y        = np.random.randn(N) * 0.3
    vel_z        = np.random.randn(N) * 0.3
    ang_vel_x    = np.random.randn(N) * 0.1
    ang_vel_y    = np.random.randn(N) * 0.1
    ang_vel_z    = np.random.randn(N) * 0.1
    inlier_ratio = np.clip(np.random.rand(N), 0.3, 1.0)
    reproj_err   = np.abs(np.random.randn(N)) * 2.0
    parallax     = np.abs(np.random.randn(N)) * 10.0
    depth_z      = np.abs(np.random.randn(N)) * 3.0 + 1.0

    # Feature matrix — shape (N, 13)
    features = np.stack([
        px_error_x, px_error_y, depth_error,
        vel_x, vel_y, vel_z,
        ang_vel_x, ang_vel_y, ang_vel_z,
        inlier_ratio, reproj_err, parallax, depth_z
    ], axis=1)

    # ------------------------------------------------------------------
    # Derive labels from ground truth
    # ------------------------------------------------------------------
    labels = generate_stop_labels(
        px_error_x, px_error_y, depth_error,
        vel_x, vel_y, vel_z,
        px_thresh_xy=5.0,   # <-- tune to your pixel scale
        depth_thresh=0.1,   # <-- tune to your depth scale (meters)
        vel_thresh=0.05,    # <-- tune to your velocity scale (m/s)
    )

    print(f"\nDataset: {N} timesteps, {features.shape[1]} features")
    print(f"Labels shape: {labels.shape}")

    # ------------------------------------------------------------------
    # Train / val split (keep temporal order — do NOT shuffle episodes)
    # ------------------------------------------------------------------
    WINDOW   = 10
    split    = int(0.8 * N)
    f_train, f_val  = features[:split], features[split:]
    l_train, l_val  = labels[:split],   labels[split:]

    train_ds = StopDataset(f_train, l_train, window_size=WINDOW)
    val_ds   = StopDataset(f_val,   l_val,   window_size=WINDOW)

    train_loader = DataLoader(train_ds, batch_size=64, shuffle=True)
    val_loader   = DataLoader(val_ds,   batch_size=64, shuffle=False)

    # ------------------------------------------------------------------
    # Build + train
    # ------------------------------------------------------------------
    INPUT_SIZE = features.shape[1]  # 13
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"\nTraining on: {device}")

    pos_weights = compute_pos_weights(l_train)

    model = StopDetectorGRU(
        input_size=INPUT_SIZE,
        hidden_size=64,
        num_layers=2,
        dropout=0.3,
    )

    history = train(
        model, train_loader, val_loader,
        pos_weights=pos_weights,
        epochs=50,
        lr=1e-3,
        device=device,
    )

    # ------------------------------------------------------------------
    # Evaluate + export
    # ------------------------------------------------------------------
    print("\n=== Validation Metrics ===")
    evaluate(model, val_loader, device=device)

    plot_loss(history)

    torch.save(model.state_dict(), "stop_detector.pt")
    print("\nModel saved to stop_detector.pt")

    export_onnx(model, INPUT_SIZE, WINDOW, "stop_detector.onnx")