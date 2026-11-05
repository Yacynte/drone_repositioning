# Handoff: PNEC translation channel freezing at zero

**Written for:** another Claude Code instance picking up this investigation cold.
**Owner:** Batchaya Noumeme Yacynte Divan (byacynte@gmail.com) — Master's thesis,
Project ENGEL, AImotion Bavaria, THI.
**Status:** root-cause chain identified and evidenced from two independent logs;
not yet fixed. This file is the starting point for fixing it.

---

## 1. What this project is

A closed-loop, vision-based UAV repositioning system. A drone returns to a
previously-recorded target pose (a "GT" — ground truth image + logged
position/orientation) after GNSS navigation leaves it several meters/degrees
off. A monocular camera feed is matched against the cached GT image
(SuperPoint + LightGlue, in a separate Python process), the match set is
turned into a relative pose `(R, t̂)` by a PNEC (Probabilistic Normal Epipolar
Constraint) solver, a Sampson-error pixel scale turns `t̂` into a position
error, and both the position and orientation error are mapped through a
bounded sigmoid into velocity/angular-rate commands. Those commands are sent
over UDP to **DroneManager** (https://github.com/AImotion-Bavaria/DroneManager),
which owns the actual MAVLink link to the PX4 flight controller and the
gimbal — this codebase never talks to PX4/MAVLink directly.

Full narrative write-up: `masters_thesis/` (LaTeX; `chapters/03_methodology.tex`
has the exact math, `chapters/04_Implementation.tex` has the software/hardware
architecture, `chapters/06_conclusion.tex` has known limitations).

Code layout:
- `src_cpp/main.cpp` — main loop: read matches → `ImageMatcher::getAlignment()`
  → sigmoid → send command.
- `src_cpp/ImageMatcher.cpp` — `getAlignment()` (the only entry point
  `main.cpp` calls) wires up covariance estimation, the unscented transform,
  and the PNEC solver.
- `include/pnecOptimizer.hpp` — the PNEC solver itself (rotation + translation).
- `src_cpp/MetadataClient.cpp` — `respositionFunc()`, the deadband + UDP send
  to DroneManager.
- `src_cpp/Utils.cpp` / `include/Utils.h` — the `activation()` sigmoid.
- `controls/clip_comparison.py` — offline evaluation script: matches each
  trial's Start/End capture against its GT image and motion log.

Data layout (all under `controls/data/`): `groundTruths{N}/imagesGT{N}/` (GT
images), `groundTruths{N}/*.csv` (GT motion logs), `motionLog{N}_{sub}/<weather>/`
(a run: `MotionLog_<trial>_<timestamp>.csv` + `images/imageStart*.png` /
`imageEnd*.png`), `results{N}_{sub}/` (this script's output). `controls/data/logs/`
holds raw `AlgoLog_*.csv` files from the live C++ control loop
(`img_ts,send_ts,dt_ms,px_error,w_x,w_y,w_z,v_x,v_y,v_z`).

---

## 2. The problem

Translation does not converge like rotation does — it mostly doesn't move
*at all*. This was first raised by the project owner as "translation doesn't
optimize as well as rotation" while reviewing `clip_comparison.py`'s output,
and turned out to be much more specific and severe than a convergence-speed
difference once the actual logged positions were checked.

### Evidence 1 — `controls/data/motionLog4_1/sunny/` (2026-09-25 run)

For each trial, position error = `‖(PosX,PosY,PosZ)_trial − (PosX,PosY,PosZ)_GT‖`
(GT taken as the last row of its motion log), rotation error = angular
distance between `(PitchCam,YawCam)_trial` and `(PitchCam,YawCam)_GT`:

| trial | duration | position error start → end | distinct `(PosX,PosY,PosZ)` samples | rotation error start → end |
|---|---|---|---|---|
| 1 | 114.8 s | 244 → **1499 cm** (diverged) | 988 / 1471 | 15.7° → 1.7° ✓ |
| 2 | 12.2 s | 286 → 247 cm (~14% closer) | 44 / 156 | 18.9° → 3.3° ✓ |
| 3 | 9.2 s | 248.2 → **248.2 cm (bit-identical)** | **1 / 121** | 11.3° → 0.8° ✓ |
| 4 | 10.5 s | 272.9 → **272.9 cm (bit-identical)** | **1 / 136** | 17.5° → 0.2° ✓ |
| 5 | 10.6 s | 248.3 → **248.3 cm (bit-identical)** | **1 / 136** | 18.4° → 1.1° ✓ |

Trials 3–5: `PosX/PosY/PosZ` is byte-identical across *every single logged
sample* for the whole trial — the vehicle receives a nonzero translation
command on **zero** frames. Rotation converges cleanly in every trial. Trial 2
moves almost nothing (~1.4 m of a required distance change of nothing — it's
mostly gimbal correction). Trial 1 is the outlier: it moves a lot, but away
from the target, and never converges.

Reproduce this with a stdlib-only Python snippet (no repo deps needed) reading
`data/groundTruths4/MotionLog_<id>_*.csv` and
`data/motionLog4_1/sunny/MotionLog_<id>_*.csv` — same columns as below.

### Evidence 2 — `controls/data/logs/AlgoLog_1784213749_423388.csv` (2026-07-16 run)

This is the live control loop's own per-frame log — `v_x,v_y,v_z` is the
**commanded** velocity actually sent onward, not a derived position. Early in
the file it's large and varying (e.g. `-44.5, 44.3, -47.5`, within the
`±V_max=50` bound). By the end of the file:

```
img_ts,send_ts,dt_ms,px_error,w_x,w_y,w_z,v_x,v_y,v_z
1784213875.370966,1784213875.756119,385.153,29.997,0.000,-0.443,0.000,0.000,0.000,0.000
1784213876.552668,1784213876.972240,419.573,21.896,0.000,-0.255,0.000,0.000,0.000,0.000
```

`v_x,v_y,v_z` are *exactly* `0.000` while `w_y` is still nonzero (rotation
still actively correcting) and `px_error` is still 20–30 (nowhere near
converged, and nowhere near the `minTrans=1` deadband). This is a second,
independent dataset (different date, different pipeline path — the live
control loop, not the offline motion log) showing the same signature: the
translation channel goes to exactly zero and stays there, decoupled from how
far off the vehicle actually still is.

**Conclusion: this is not "translation converges slower than rotation."
Something forces the translation command to exactly zero and keeps it there,
independent of the actual position error.**

---

## 3. Root-cause chain (code-verified, not yet fixed)

Traced end to end, file:line, in the order data flows:

1. **`include/pnecOptimizer.hpp:260-346`** — `RelativePoseEstimatorOld::solveTranslation()`.
   Computes `AtA = Σ n·nᵀ / N` (n = `bearing1 × R·bearing2`), eigenvalues
   `λ0 ≤ λ1 ≤ λ2`, and applies three guards, **any one of which sets
   `t = Vector3d::Zero()`** instead of returning a (possibly noisy) direction:
   - line 308: `λ2 < 1e-3` → "pure rotation / zero baseline"
   - line 316: `λ1/λ2 < 0.05` → "degenerate translation (conditioning)"
   - line 329: `λ0/λ1 > 0.2` → "ambiguous null space"
   These are exactly the τp=1e-3, τc=0.05, τn=0.2 guards documented in
   `masters_thesis/chapters/03_methodology.tex` §3.4 ("Why PNEC rather than
   the essential matrix alone") — this part of the thesis text is accurate.

2. **`src_cpp/ImageMatcher.cpp:1080`** — `getAlignment()` instantiates
   `pnec::RelativePoseEstimatorOld estimator(logger, K);` — **confirmed this
   is the live class**, not the other PNEC implementation also present in
   the same header (see §4 below). `getAlignment()` returns `t_dir` as
   `direction` at the tuple's 2nd position (`ImageMatcher.cpp:1136`).

3. **`src_cpp/main.cpp:426`** — `matcher.getAlignment(matches, frame)` receives
   `direction` (= `t̂`) and `transError_` (= Sampson pixel error `ε_px`).

4. **`src_cpp/main.cpp:450`** — `directionCv = direction * transError_` —
   this is `e_pos = ε_px · t̂` from the thesis. **If `direction` is
   `(0,0,0)`, `e_pos` is force-zeroed regardless of how large `transError_`
   is** — the guard's zero overrides the error magnitude entirely.

5. **`src_cpp/main.cpp:456`** — `cmdVx = kMaxVelocity * activation(translation, kTranslationGain)`.
   `activation()` is the sigmoid (`src_cpp/Utils.cpp:53`); sigmoid of an
   exact zero input is an exact zero output.

6. **`src_cpp/MetadataClient.cpp:479-504`** — `respositionFunc()`'s deadband
   check (`if (std::abs(trans_error.x) < minTrans) x = 0;` etc.) is a no-op
   here — the input was already exactly zero, not merely under the 1.0
   threshold. The deadband is not the cause; it's just where the zero passes
   through on its way to the UDP command sent to DroneManager.

**The self-reinforcing trap:** once `t̂ = 0`, the vehicle receives zero
translation command and doesn't move. On the *next* frame, the true baseline
between the current view and the target is therefore still ~0, so `λ2` (or
whichever eigenvalue ratio is marginal) looks just as degenerate as before —
there is nothing in this loop that perturbs the vehicle to create new
parallax once it's stuck. This is consistent with trials 3–5 showing the
*exact same* position for 121–136 consecutive samples: it isn't noisy
near-zero motion, it's a fixed point the guards never let it leave. Once
triggered (possibly on the very first frame, if the true offset happens to
look near-degenerate from that starting geometry — e.g. a large depth-to-
baseline ratio), there's no recovery mechanism.

Rotation is unaffected by any of this: `solveRotation` runs regardless of
`solveTranslation`'s outcome, which is exactly why the gimbal keeps
converging while the body never moves.

---

## 4. Related-but-distinct issues in the same area (don't conflate with the above)

These are pre-existing `\todo`s in `masters_thesis/chapters/03_methodology.tex`
that live in the same files but affect *different* parts of the pipeline —
worth being aware of while in this code, but they are not what's described
in §3 above:

- **Double covariance inversion** (`ImageMatcher.cpp` around line 1101,
  `covariances.push_back(H.inv(cv::DECOMP_SVD))`): the Python process already
  publishes `Σ_2D,i = H_i⁻¹`; inverting it again here recovers `H_i` itself,
  not `Σ_2D,i`. This feeds `cov3d` (via `UnscentedTransform::propagate2DTo3D`,
  used by `solveRotation`'s `σ_i` weighting), **not** `solveTranslation`'s
  `AtA` (which only uses raw bearings, no covariance). So this is a candidate
  explanation for weighting problems in the *rotation* solve, not for the
  translation freeze in §3.
- **UT sigma points centered at pixel `(0,0)` instead of the keypoint** —
  same function, same caveat: affects the covariance/weighting path, not
  `solveTranslation`'s guards directly.
- **Translation initialization** (`chapters/03_methodology.tex` §3.4.2,
  `t_init`/`t_dir` warm-starting from the previous frame) — could matter for
  *how quickly* a valid `t̂` is found once the guards do pass, but doesn't
  explain guards firing on effectively every frame.
- **The unused, more advanced `RelativePoseEstimator` class**
  (`include/pnecOptimizer.hpp:410-591`) — has a Huber-loss + LM-damped
  rotation solve and a *different*, unnormalized translation guard
  (`t_reliability = (λ1−λ0)/(λ2+1e-8) < 0.1`, plus `λ1<1e-2 && λ2<1e-2`).
  This is the "robust PNEC refinement" the thesis lists as future work
  (`chapters/06_conclusion.tex` §6.4) — **not currently instantiated
  anywhere** (`getAlignment()` uses `RelativePoseEstimatorOld`, confirmed).
  Don't assume switching to it is the fix without evidence; it has its own
  unvalidated guard formula and lacks the `/N` normalization the thesis
  argues for.

---

## 5. Suggested next steps

1. **Confirm the trap hypothesis directly from logs**, if per-frame PNEC logs
   are available for a frozen trial (the `logger.log("pnec", ...)` calls at
   `pnecOptimizer.hpp:296-299,317-321,330-334` print the eigenvalues and
   which guard fired, when a build has that logging enabled/captured). If
   none exist for trials 3–5, consider re-running one trial with that logger
   output captured, to see which of the three guards fires and whether it's
   the *same* guard on every frame.
2. **Check whether the guard thresholds are appropriate for this scene's
   depth/baseline regime.** τp=1e-3 (parallax) is the prime suspect for a
   scene where the target is far away relative to the ~2.5 m translation
   offset — at that depth-to-baseline ratio, the epipolar geometry may
   genuinely look close to pure-rotation even for real translation. This
   would be a scene/guard-mismatch, not strictly a "bug" — worth checking
   against the actual GT distances in `groundTruths4/*.csv` (`PosX/Y/Z` there
   vs. the trial's starting `PosX/Y/Z`) to get a real depth estimate.
3. **Check for a recovery mechanism**, or whether one needs to be added — e.g.
   don't let `t̂=0` persist indefinitely without at least a small exploratory
   motion, or re-attempt with a relaxed guard after N consecutive zero
   frames. This would need to preserve the property the thesis relies on
   (§3.4: `t̂=0` must still mean "no signal" for the deadband/convergence
   logic in `chapters/03_methodology.tex` §3.6) — don't just remove the
   guards.
4. **Verify against trial 1's divergence too** — if the fix changes when/how
   `t̂` is accepted, trial 1 (which moved a lot in the wrong direction) is the
   other case to re-check; a bad direction may be a symptom of the guards
   passing when they shouldn't, the flip side of freezing when they
   shouldn't fail.
5. Whatever the finding, it likely needs to be written up as a correction or
   an added finding in `masters_thesis/chapters/03_methodology.tex` (the PNEC
   section) and `chapters/06_conclusion.tex` (limitations) — the thesis
   currently presents the guards as working as intended.

## 6. Explicitly not in scope for this handoff

- Don't touch the DroneManager integration, MAVLink, or the UDP command
  format — those are correct and unrelated to this bug (see the "Integration
  with DroneManager" section of `chapters/04_Implementation.tex` if needed
  for context).
- Don't fabricate or assume specific numeric thesis results — the thesis's
  Results chapter is still a scaffold; any numbers produced while
  investigating this should go through the project owner before being
  treated as final.
- Don't switch the live code to `RelativePoseEstimator` (the unused robust
  variant) as a quick fix without validating its own guard formula first —
  see §4.

---

## 7. Update 2026-09-27 — guard identified from logs, first fix applied

**Per-frame PNEC logs exist** for the 2026-09-25 run: `LogData/log_2026_09_25_*.txt`
(staged for deletion in the working tree; read them via `git show HEAD:LogData/...`).
Log → trial mapping (from the "target image" line): 12_32_41=GT1, 12_34_56=GT2,
12_35_28=GT3, 12_35_58=GT4, 12_36_28=GT5, 12_36_59=GT1 (retry).

Replaying the three guards on the logged eigenvalues:

| log | frames | λ2<1e-3 | cond<0.05 | null>0.2 | accepted | median λ2/λ0 |
|---|---|---|---|---|---|---|
| GT1 12_32_41 | 236 | 18 | 33 | 0 | 185 | 319 |
| GT2 12_34_56 | 30 | 22 | 0 | 0 | 8 | 74 |
| GT3 12_35_28 | 23 | **23** | 0 | 0 | **0** | 31 |
| GT4 12_35_58 | 26 | **26** | 0 | 0 | **0** | 56 |
| GT5 12_36_28 | 26 | **26** | 0 | 0 | **0** | 23 |

- The frozen trials are 100% the **absolute parallax guard** (`λ2 < 1e-3`), and it
  fires **from frame 1**. The vehicle never moved, so §3's "self-reinforcing trap" is
  not how the freeze starts; it is present from the beginning. This guard also
  returned without logging, which is why those logs looked silent.
- The signal is real: λ0 ≈ 3e-6 (≈1.7 px matching noise), and λ2 is 23–56× above
  it. 1e-3 corresponds to ~1.8° RMS parallax, which a ~2.5 m offset in this scene
  never produces.
- With t̂ = 0 the Sampson error is also 0 (`transError: 0` in those logs), so e_pos is
  zero on both factors.

**Fix applied** (`include/pnecOptimizer.hpp`, `RelativePoseEstimatorOld::solveTranslation`):
the parallax guard is now noise-relative, `λ2/λ0 < 10` (plus absolute floor
`λ2 < 1e-7`), and it logs `error: Insufficient parallax (...)`. Offline replay with
the other two guards unchanged: GT3 11/23, GT4 26/26, GT5 24/26, GT2 30/30, GT1 201/236
frames accepted. It builds but has **not been flown/simulated yet**. The threshold 10 is a
first choice, not tuned. Note that the null-space guard already implies λ2 ≥ 5·λ0, so the
new test only adds anything above 5.

**New finding — trial 1 divergence is likely a direction/sign problem, not noise.**
In GT1 12_32_41 the accepted, UE-converted translation points almost constantly along
+x (per-quarter mean direction x = 0.55, 0.93, 0.98, 0.98; only 10 reversals across
185 frames) while position error grows 2.4 m → 15 m. A stable direction combined with
monotonic divergence suggests a convention error: `t` sign after
`checkAndFlipCheirality`, the camera→target vs target→camera meaning of t (with
`b1 = current`, `b2 = target`, the epipolar constraint gives X_cur = R·X_tgt + t), or
the `ConvertCVToUE` axis mapping. **Verify this before flying the relaxed guard**,
because accepting more frames with a wrong sign would make GT3–5 diverge like GT1
instead of freezing. `controls/data/motionLog4_1/` (needed to check the direction
against true positions) is not present in this checkout.
