#pragma once
#include <vector>
#include <numeric>
#include <opencv2/core.hpp>
#include <cmath>
// Detects when a drone has reached its target by identifying oscillation
// in the translation vector (reprojection_scale * recoverPose direction).
//
// Strategy:
//   1. Compute finite difference over k frames for noise robustness
//   2. Extract sign of each axis delta
//   3. Count negative-to-positive crossings only in a sliding window
//      (+ve to -ve = error still dropping = approaching, ignore)
//      (-ve to +ve = error rose after drop = overshot, count this)
//   4. If upward crossings exceed threshold -> oscillating around target -> stop
//
// Params:
//   history          - rolling buffer of translation vectors (caller manages push)
//   k                - frame stride for finite difference (recommended: 3-5)
//   window           - number of recent deltas to inspect (recommended: 8-12)
//   min_crossings    - how many sign changes required to call a stop (recommended: 3-4)
//   min_magnitude    - ignore deltas smaller than this (filters near-zero drift noise)

struct StopDetectorConfig {
    int   k              = 4;     // finite difference stride (frames)
    int   window         = 4;    // sliding window size (in delta samples)
    int   max_crossings  = 1;     // drone overshot and recovered this many times in recent history -> call it oscillation
    float min_magnitude  = 0.5f; // minimum delta magnitude to count (tune to your scale)
};

class StopDetector {
public:
    explicit StopDetector(StopDetectorConfig cfg = {}) : cfg_(cfg) {}

    // Call once per frame with the latest translation vector.
    // Returns true when oscillation is detected (stop condition met).
    bool update(const cv::Point3f& translation) {
        history_.push_back(translation);
        // history_.push_back(cv::norm(translation)); // consider magnitude only to detect oscillation regardless of direction
        // Need at least k+1 samples to compute one delta,
        // and window+1 deltas to evaluate crossings.
        const int required = cfg_.k + cfg_.window;
        if (static_cast<int>(history_.size()) < required)
            return false;

        // Keep buffer bounded — only retain what we need
        const int max_history = required + 50; // add some extra padding to avoid too-frequent erasing
        if (static_cast<int>(history_.size()) > max_history)
            history_.erase(history_.begin());

        // Build recent finite differences over stride k
        // delta[i] = history[i + k] - history[i]
        // std::vector<float> deltas;
        std::vector<cv::Point3f> deltas;
        deltas.reserve(cfg_.window);

        int start = static_cast<int>(history_.size()) - cfg_.window - cfg_.k;
        for (int i = start; i < start + cfg_.window; ++i) {
            deltas.push_back(history_[i + cfg_.k] - history_[i]);
        }

        // return hasPermanentSignChange(deltas.data(), cfg_.window);

        // Count only negative-to-positive crossings per axis.
        // +ve to -ve = error still dropping (drone approaching) -> ignore
        // -ve to +ve = error increased after decrease (drone overshot) -> oscillation
        auto count_crossings = [&](auto axis_fn) -> int {
            int crossings = 0;
            int prev_sign = 0;
            for (const auto& d : deltas) {
                float val = axis_fn(d);
                if (std::abs(val) < cfg_.min_magnitude) continue; // ignore noise floor
                int s = (val > 0.f) ? 1 : -1;
                if (prev_sign == -1 && s == 1)  // only -ve -> +ve counts
                    ++crossings;
                prev_sign = s;
            }
            return crossings;
        };

        int cx = count_crossings([](const cv::Point3f& p) { return p.x; });
        int cy = count_crossings([](const cv::Point3f& p) { return p.y; });
        int cz = count_crossings([](const cv::Point3f& p) { return p.z; });

        // Stop if ANY axis oscillates enough
        // (change to && if you want ALL axes to agree)
        return (0 < cx && cx <= cfg_.max_crossings) ||
                (0 < cy && cy <= cfg_.max_crossings) ||
                (0 < cz && cz <= cfg_.max_crossings);
    }

    void reset() { history_.clear(); }

    // Expose for debugging / visualization
    // const std::vector<float>& history() const { return history_; }

private:
    StopDetectorConfig        cfg_;
    std::vector<float>  history__;
    std::vector<cv::Point3f>  history_;

    bool hasPermanentSignChange(const float* deltas, int size, float zeroThreshold = 0.1f, float threshold = 0.8f) {
        // bool hasDirectionalShift(const float* deltas, int size, float zeroThreshold = 0.1f) {
        if (size < 4) return false;

        // 1. Map raw deltas to discrete direction states: -1, 0, or 1
        int states[12]; 
        int totalStateSum = 0;
        for (int i = 0; i < size; ++i) {
            if (deltas[i] > zeroThreshold) states[i] = 1;
            else if (deltas[i] < -zeroThreshold) states[i] = -1;
            else states[i] = 0;
            
            totalStateSum += states[i];
        }

        // 2. Linear Scan for the "Flip" point
        for (int k = 2; k <= size - 3; ++k) {
            int leftNegCount = 0;
            int rightNegCount = 0;
            int rightPosCount = 0;

            // 1. Check Left side: Was it mostly decreasing (-1)?
            for (int i = 0; i < k; ++i) {
                if (states[i] == -1 || states[i] == 0) leftNegCount++;
            }

            // 2. Check Right side: Is it now stable (0 or 1)?
            for (int i = k; i < size; ++i) {
                if (states[i] == -1 || states[i] == 0) rightNegCount++;
                if (states[i] == 1) rightPosCount++;
            }

            float leftNegRatio = (float)leftNegCount / k;
            float rightNegRatio = (float)rightNegCount / (size - k);
            float rightPosRatio = (float)rightPosCount / (size - k);
            int rightSize = size - k;

            // LOGIC: 
            // A) Left was mostly negative
            // B) Right has ALMOST NO negatives (it stopped falling)
            // C) Right has at least SOME positive or is purely zero
            if (leftNegRatio >= threshold && rightNegCount <= 1 - threshold && rightPosRatio >= threshold) { 
                
                // This confirms it didn't just 'pause' and go back to -1
                // It either stayed at 0 or climbed to +1
                return true; 
            }
        }
        return false;
    };
};