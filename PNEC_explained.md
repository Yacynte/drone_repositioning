# How PNEC works in the repositioning loop

An explanation of the PNEC relative-pose solver as implemented in this codebase
(`include/pnecOptimizer.hpp`, `src_cpp/ImageMatcher.cpp`, `src_cpp/main.cpp`).

---

## 1. The big picture

The drone has a **target image**, recorded at the pose it should return to, and a
**live image** from its gimbal camera. PNEC answers one question: *how is the current
camera rotated and displaced relative to the target camera?* The answer is:

- **R**, the relative rotation, which drives the gimbal;
- **t̂**, the direction of the translation (a unit vector), which drives the drone.

Each frame goes through six steps:

1. **Match:** SuperPoint + LightGlue in Python produce matches and a structure tensor H per match.
2. **Uncertainty:** each match gets a 3D covariance Σ₃D.
3. **Solve R** (PNEC).
4. **Solve t̂** (PNEC).
5. **Fix the sign of t̂ and compute the magnitude.**
6. **Turn the result into commands.**

## 2. Geometry: why one match constrains the pose

Each matched pixel is turned into a **bearing**, a unit vector pointing from the camera
centre towards the 3D point:

```
b = normalize(K⁻¹ [u, v, 1]ᵀ)
```

`b₁` is from the current image and `b₂` from the target image.

The two camera frames are related by `X_cur = R·X_tgt + t`. So `R·b₂` is the target
bearing expressed in current-camera axes, and t is the target camera's position seen
from the current camera.

**Key fact:** the current camera centre, the target camera centre and the 3D point form
a triangle. So `b₁`, `R·b₂` and `t` all lie in one plane, the **epipolar plane**. Three
coplanar vectors give:

```
b₁ · (t × R b₂) = 0    ⇔    t · nᵢ = 0,   with   nᵢ = b₁ × R b₂
```

`nᵢ` is the **normal of match i's epipolar plane**, and **t must be perpendicular to
every normal**. This is the "normal epipolar constraint" in the name.

Scale can't be recovered: doubling t (and the scene) gives the same images. So only the
**direction** t̂ is estimated.

## 3. What makes it "probabilistic"

With noisy matches, `t · nᵢ` isn't exactly zero; that's the residual `eᵢ`. Classical
methods (8-point, essential matrix) treat every match the same. PNEC says some matches
are more trustworthy than others, **in a direction-dependent way**:

```
eᵢ  = t̂ · (b₁ᵢ × R b₂ᵢ)
σᵢ² = variance of eᵢ, given the uncertainty of the bearing
E(R, t̂) = Σᵢ eᵢ² / σᵢ²
```

The intuition: if a keypoint is uncertain only along a direction that hardly changes the
epipolar plane, that uncertainty doesn't hurt, so the match keeps a high weight. If it's
uncertain in a direction that tilts the plane, the match is down-weighted. That's better
than plain inlier/outlier thinking, because uncertainty along an edge is anisotropic.

**How this code computes σᵢ:** it uses the simplified form `σᵢ² = t̂ᵀ Σ₃D,ᵢ t̂`, with
Σ₃D from the current-frame bearing. The original paper (Muhle et al., CVPR 2022)
propagates the covariance through `[b₁]× R` as well. If this is presented as PNEC, say
it's a simplified variance.

## 4. Where the uncertainty comes from

1. **Structure tensor (Python).** In a 5×5 patch around the current keypoint,
   `H = Σ [gx², gx·gy; gx·gy, gy²]`. Strong gradients in both directions mean a
   well-localized corner; a gradient in one direction only means an edge, uncertain
   along it.
2. **Pixel covariance (C++).** `Σ₂D = H⁻¹`, inverted once.
3. **Unscented transform.** Instead of differentiating the mapping from pixel to bearing:
   - take 5 "sigma points" around the keypoint: the centre plus ±√(3·eigenvalue) along
     each eigenvector of Σ₂D;
   - unproject each through K⁻¹ and normalize;
   - use the sample covariance of the resulting bearings as **Σ₃D** (3×3, in bearing
     space).

Recently fixed: H was inverted twice, the sigma points were centred at pixel (0,0), and
the per-frame covariance buffer was never cleared. Still open: the absolute scale of Σ
(`σ_I² · H⁻¹`) is uncalibrated, which matters for the Huber threshold below.

## 5. Solving the rotation: Gauss–Newton on rotations

t̂ is held fixed (the previous frame's estimate, or `(1,0,0)` at the start), and the
solver finds the R that minimizes `Σ eᵢ²/σᵢ²`.

- **Parameterization:** R isn't updated element by element. A small rotation vector δ
  is applied, `R ← exp([δ]×)·R`, so R always stays a valid rotation.
- **Linearizing:** a small rotation changes `R b₂` by `δ × R b₂`, so

  ```
  rᵢ = t̂·(b₁ᵢ × R b₂ᵢ) / σᵢ
  Jᵢ = (−[b₁ᵢ]× [R b₂ᵢ]×)ᵀ t̂ / σᵢ
  δ  = −(Σ wᵢ Jᵢ Jᵢᵀ + λI)⁻¹ Σ wᵢ Jᵢ rᵢ
  ```

  That's 3 unknowns, a 3×3 linear system, at most 10 iterations, and it stops when
  ‖δ‖ < 1e-6.
- **Robustness in the live `RelativePoseEstimator`:**
  - *Huber weights:* `wᵢ = 1` while |rᵢ| ≤ 1.345, otherwise 1.345/|rᵢ|. Outlier
    matches pull linearly instead of quadratically.
  - *LM damping:* λ = 1e-3 keeps the step stable when the system is poorly conditioned.
  - *Alternation:* R and t̂ are solved alternately for 5 passes, because σᵢ and the
    residual both depend on t̂.
  - *Warm start:* R and t̂ start from the previous frame.

| | `RelativePoseEstimatorOld` (baseline) | `RelativePoseEstimator` (live) |
|---|---|---|
| Passes | 1 | 5, alternating R ↔ t̂ |
| Loss | least squares | Huber, k = 1.345 |
| Damping λ | 0 | 1e-3 (LM) |
| σᵢ floor | 1e-3 rad | none |
| Warm start | previous frame | previous frame |

## 6. Solving the translation direction: an eigenvector

With R known, t̂ should be perpendicular to all normals nᵢ, so minimize
`Σ (t̂·nᵢ)² = t̂ᵀ M t̂`, where `‖t̂‖ = 1` and

```
M = (1/N) Σᵢ nᵢ nᵢᵀ
```

Minimizing that expression over unit vectors gives the **eigenvector of M's smallest
eigenvalue**. No iteration is needed. (This step doesn't use σ; it's a plain
least-squares solve. Dividing by N makes the thresholds independent of the number of
matches.)

The three eigenvalues `λ₀ ≤ λ₁ ≤ λ₂` are the most useful diagnostic in the whole
system:

- **λ₀:** what's left along t̂, i.e. the **noise floor**. In the logs it's ≈3·10⁻⁶,
  about 1.7 px of matching noise at fx = 960.
- **λ₁, λ₂:** how strongly the normals spread in the plane perpendicular to t̂, i.e.
  the **parallax energy**. If both are large and similar, t̂ is well defined.
- **λ₂ ≈ λ₀:** the normals are only noise, which happens under pure rotation or with no
  baseline.

## 7. The guards: when to say "no translation signal" (t̂ = 0)

| Check | t̂ = 0 when | What it catches |
|---|---|---|
| Parallax | λ₂/λ₀ < 10 (or λ₂ < 1e-7) | Pure rotation: no signal above noise |
| Conditioning | λ₁/λ₂ < 0.02 **and** parallax < 3 px | Parallax in only one direction (e.g. forward motion) |
| Null space | λ₀/λ₁ > 0.35 **and** parallax < 3 px | Two near-zero directions, so t̂ is ambiguous |

**Why the guards changed:** the original parallax check was a fixed threshold,
λ₂ < 10⁻³ (about 1.8° RMS parallax). In trials 3–5 of the 2026-09-25 Unreal run it fired
on **every frame from the very first one**, even though λ₂ was 23–56× the noise level.
The drone never moved, while rotation converged perfectly. Making the check relative to
noise accepts at least 72 of those 75 frames when replayed. This is offline only; it
hasn't been tested in closed loop yet.

## 8. The sign of t̂: cheirality

An eigenvector's sign is arbitrary: t̂ and −t̂ are both solutions. To pick one, the
solver triangulates each match's depth along the target bearing from
`b₁ ∥ d·R b₂ + t`:

```
dᵢ = −(b₁ᵢ × t̂)·(b₁ᵢ × R b₂ᵢ) / ‖b₁ᵢ × R b₂ᵢ‖²
```

Real points must lie in front of the camera (d > 0). If most dᵢ are negative, t̂ is
flipped. After this, **t̂ points from the current camera towards the target camera**, in
current-camera axes, which is the direction to fly.

Caveat: it's a majority vote, so with few or noisy matches it can choose the wrong sign.

## 9. The magnitude: de-rotated parallax in pixels

t̂ has length 1, so the control loop needs a separate "how far":

```
e_px  = fₓ · medianᵢ angle(b₁ᵢ, R b₂ᵢ)
e_pos = e_px · t̂
```

After undoing the rotation, any remaining displacement between matched bearings is
caused by translation. That displacement grows with baseline/depth and drops to the
noise floor (1–2 px) at the target. It's **unsigned**; the direction always comes
from t̂.

The Sampson error used before measured how well the model fit, not distance. It sits at
the noise level for any offset and is undefined when t̂ = 0. It is now only logged as a
diagnostic.

## 10. Into commands

- **Rotation:** R is converted to Euler angles: pitch about camera x, yaw about camera y.
  Roll is dropped because the gimbal can't roll. The rate is `5°/s · s(0.1·angle)` with
  `s(x) = 2/(1+e⁻ˣ) − 1`. The signs already match Unreal's.
- **Translation:** `e_pos` goes from OpenCV camera axes to Unreal camera axes as
  `(z, x, −y)`, then `v = 100 · s(0.25·e)` in cm/s. Unreal rotates v by the
  camera-to-drone rotation and integrates `pos += v·dt`.
- **Stopping:** an axis stops below its deadband (0.5° for rotation, 1.0 for
  translation). The drone counts as arrived when everything has been zero for 1 s.

## 11. Limitations to state openly

1. **Rotation–translation ambiguity.** With a narrow field of view and small parallax, a
   sideways offset looks almost like a small turn. In trial 1, t̂ said "forward" while
   the target was left and up.
2. **A wrong R corrupts both outputs.** Any rotation it fails to remove inflates the
   parallax magnitude: 130–380 px in the 2026-10-01 GT2 run, where the commands
   saturated.
3. **Few matches** after a large viewpoint change: 14 versus 59 for the same target.
   With that few, R becomes unreliable.
4. **Uncalibrated covariance scale** and a **majority-vote sign**.

Planned fixes: rotate first and translate only once rotation has converged; cap the
speed as a whole rather than per axis; skip frames with too few matches or a poor fit;
and check t̂ against logged ground truth.

## Likely questions

- *"Why not use the essential matrix with RANSAC?"* It weights all matches equally and
  mixes R and t in one 3×3 matrix. Here R is optimized with per-match uncertainty, and
  t̂ comes from a closed-form solve.
- *"Why do you get the direction of t but not its length?"* Two views give no scale. The
  pixel parallax stands in for distance, which works in a closed loop because the drone
  only needs to know which way to go and when to stop.
- *"How do you know when you've arrived?"* The parallax reaches the noise floor, the
  guards set t̂ to zero, and the deadbands stop all axes.
