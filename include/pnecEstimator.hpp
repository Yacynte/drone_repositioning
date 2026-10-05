#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <sstream>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>

#include "Logger.h"
#include "pnecOptimizer.hpp"   // MatchData, Vector3d, Matrix3d

// Relative-pose solver following the Probabilistic Normal Epipolar Constraint as in
// Muhle et al., CVPR 2022 (https://dominikmuhle.github.io/publications/pnec/) and the
// reference implementation (github.com/tum-vision/pnec):
//
//   e_i       = t^T (b1_i x R b2_i)
//   sigma_i^2 = t^T [R b2_i]x Sigma1_i [R b2_i]x^T t + c      (noise on b1, the host /
//                                                              current-frame bearing)
//   E(R, t)   = sum_i e_i^2 / sigma_i^2,     ||t|| = 1
//
// Differences from RelativePoseEstimator, matching the reference:
//   - sigma_i^2 maps Sigma1 through [R b2]x (was t^T Sigma1 t, which ignores the
//     epipolar geometry and blows up the weight of central matches for forward t);
//   - t is solved with the PNEC weights (self-consistent-field iteration on
//     sum_i t^T A_i t / t^T B_i t, from the best of a sphere search), not the
//     unweighted NEC eigenvector;
//   - after a few weighted R/t rounds, R and t are refined JOINTLY (Levenberg-Marquardt,
//     numeric Jacobian, so sigma_i's dependence on R and t is included).
// Kept from this project (not part of PNEC): the translation-signal guards, the
// cheirality sign vote, and an optional Huber loss (off by default; PNEC itself uses
// RANSAC for outliers).
//
// MatchData::cov3d must be the covariance of bearing1 (ImageMatcher::getAlignment builds
// it from the current-frame keypoint).
namespace pnec {

struct PNECOptions {
    double regularization = 1e-13;   // c added to sigma^2 (reference configs: 1e-13)
    int sphereSamples = 200;         // translation init candidates (one hemisphere)
    int necIterations = 50;          // NEC rotation init (t eliminated)
    // Initialization: R from the essential matrix (OpenCV 5-point + RANSAC, which also
    // supplies the inlier set) or the previous frame's R, whichever has the lower NEC
    // energy. useNecRansac switches to the slower in-house NEC RANSAC instead.
    bool useNecRansac = true;
    double essentialThresholdPx = 1.5;
    double essentialConfidence = 0.999;
    int essentialMaxIterations = 1000;
    int ransacIterations = 200;      // in-house NEC RANSAC (useNecRansac, or fallback
                                     // when no essential matrix is found)
    int ransacSampleSize = 10;       // reference config: 10
    double ransacThresholdPx = 3.0;  // inlier if |t . n_i| < threshold / fx
    int weightedRounds = 3;          // R-only LM + weighted-t rounds before the joint step
    int rotationIterations = 5;      // LM iterations per R-only round
    int scfIterations = 20;          // weighted translation iterations
    int jointIterations = 30;        // joint R,t LM iterations
    double huberK = 0.0;             // > 0 enables Huber on the normalized residual
    // Translation-signal guards (same as RelativePoseEstimator::solveTranslation).
    double minParallaxSnr = 5.0;     // was 10: frames at 6-9.7 carried real signal (run
                                     // 2026-10-05 GT3); main.cpp's filter handles the noise
    double minConditioning = 0.02;
    double maxNullspaceRatio = 0.35;
    double alignedParallaxPx = 3.0;
    // The null-space guard only applies below this parallax. Near the target
    // lambda0/lambda1 ~ 0.5 is normal: at 3 px it zeroed 85% of the 2-3 px frames in
    // logData4_11, which starved main.cpp's gate of directions and made it stop early
    // (run 5 still got usable directions down to 1.0 px). main.cpp's consistency
    // filter handles the noisier directions.
    double ambiguousParallaxPx = 1.5;
};

class PNECEstimator {
public:
    PNECEstimator(Logger& logger, const Matrix3d& K, const PNECOptions& options = PNECOptions())
        : logger(logger), K_(K), opt_(options) {}

    // Same interface as RelativePoseEstimator::estimate: R is the initial guess and is
    // refined in place; t (unit, or zero = no translation signal) and totalError (final
    // PNEC energy) are outputs; t_init seeds the translation (zero = no seed).
    void estimate(const std::vector<MatchData>& allMatches,
                  Matrix3d& R, Vector3d& t,
                  const Vector3d& t_init,
                  double& totalError) {
        totalError = 0.0;
        t = Vector3d::Zero();
        timings_ = Timings();
        auto tic = std::chrono::steady_clock::now();
        auto lap = [&tic](double& slot) {
            const auto now = std::chrono::steady_clock::now();
            slot = std::chrono::duration<double, std::milli>(now - tic).count();
            tic = now;
        };
        if (allMatches.size() < 6 || !R.allFinite()) {
            logger.log("pnec", "error: too few matches or invalid R for PNEC");
            return;
        }

        // 0. Initialization and outlier removal (the PNEC stages are not robust on
        //    their own), then NEC refinement of R (t eliminated: lambda_min(M(R))).
        std::vector<MatchData> matches;
        Vector3d t_seed = t_init;
        if (opt_.useNecRansac) {
            matches = allMatches;
            if (opt_.ransacIterations > 0 &&
                static_cast<int>(allMatches.size()) > opt_.ransacSampleSize) {
                matches = ransacNec(allMatches, R);
            }
        } else {
            matches = initFromEssentialOrPrevious(allMatches, R, t_seed);
        }
        lap(timings_.initMs);
        if (matches.size() < 6) {
            logger.log("pnec", "error: too few inliers");
            return;
        }
        necInit(matches, R, opt_.necIterations);
        lap(timings_.necMs);

        // 1. Weighted rounds: R with t fixed, then t with the PNEC weights.
        Vector3d t_dir = initialTranslation(matches, R, t_seed);
        for (int round = 0; round < opt_.weightedRounds; ++round) {
            refine(matches, R, t_dir, /*rotationOnly=*/true, opt_.rotationIterations);
            t_dir = weightedTranslation(matches, R, t_dir);
        }
        lap(timings_.weightedMs);
        // 2. Joint refinement of R and t.
        refine(matches, R, t_dir, /*rotationOnly=*/false, opt_.jointIterations);
        totalError = robustCost(matches, R, t_dir);
        lap(timings_.jointMs);

        {
            std::ostringstream ss;
            ss << "[pnec] energy=" << totalError << " t=" << t_dir.transpose();
            logger.log("pnec", ss.str());
        }

        // 3. Is there a translation signal at all? If so, fix the sign of t.
        if (hasTranslationSignal(matches, R)) {
            t = t_dir;
            checkAndFlipCheirality(matches, R, t);
        }
        lap(timings_.guardsMs);
    }

    // Wall time of each stage of the last estimate() call, in ms.
    struct Timings {
        double initMs = 0;      // essential matrix / previous-R init and outlier removal
        double necMs = 0;       // NEC refinement of R
        double weightedMs = 0;  // weighted R / t rounds (incl. translation init)
        double jointMs = 0;     // joint R,t refinement
        double guardsMs = 0;    // translation-signal guards + cheirality
    };
    const Timings& lastTimings() const { return timings_; }

    // Diagnostics of the last estimate() call:
    // share of matches whose triangulated depth agreed with the chosen sign of t
    // (0.5 = coin flip, 1 = unanimous; 0 when t was zero) ...
    double lastVoteMargin() const { return voteMargin_; }
    // ... and the matching-noise level in pixels, fx * sqrt(lambda0) of the NEC
    // normal matrix (NaN if the eigen-decomposition was not reached).
    double lastNoisePx() const { return noisePx_; }

private:
    Logger& logger;
    Matrix3d K_;
    PNECOptions opt_;
    Timings timings_;
    double voteMargin_ = 0.0;   // share of matches agreeing with the chosen sign of t
    double noisePx_ = std::numeric_limits<double>::quiet_NaN();   // fx * sqrt(lambda0)

    using Vector5d = Eigen::Matrix<double, 5, 1>;

    static Matrix3d skew(const Vector3d& v) {
        Matrix3d S;
        S <<     0, -v(2),  v(1),
              v(2),     0, -v(0),
             -v(1),  v(0),     0;
        return S;
    }

    static Matrix3d expSO3(const Vector3d& w) {
        const double angle = w.norm();
        if (angle < 1e-12) return Matrix3d::Identity() + skew(w);
        return Eigen::AngleAxisd(angle, w / angle).toRotationMatrix();
    }

    // ── PNEC residual and energy ─────────────────────────────────────────────
    // A_i = n n^T and B_i = [R b2]x Sigma1 [R b2]x^T + c I, so e^2/sigma^2 = tAt / tBt.
    void buildAB(const std::vector<MatchData>& matches, const Matrix3d& R,
                 std::vector<Matrix3d>& A, std::vector<Matrix3d>& B) const {
        A.resize(matches.size());
        B.resize(matches.size());
        for (size_t i = 0; i < matches.size(); ++i) {
            const Vector3d Rb2 = R * matches[i].bearing2;
            const Vector3d n = matches[i].bearing1.cross(Rb2);
            const Matrix3d S = skew(Rb2);
            A[i] = n * n.transpose();
            B[i] = S * matches[i].cov3d * S.transpose()
                 + opt_.regularization * Matrix3d::Identity();
        }
    }

    static double objective(const std::vector<Matrix3d>& A, const std::vector<Matrix3d>& B,
                            const Vector3d& t) {
        double f = 0.0;
        for (size_t i = 0; i < A.size(); ++i) f += t.dot(A[i] * t) / t.dot(B[i] * t);
        return f;
    }

    double residual(const MatchData& m, const Matrix3d& R, const Vector3d& t) const {
        const Vector3d Rb2 = R * m.bearing2;
        const Matrix3d S = skew(Rb2);
        const double var = t.dot(S * m.cov3d * S.transpose() * t) + opt_.regularization;
        return t.dot(m.bearing1.cross(Rb2)) / std::sqrt(var);
    }

    double huberWeight(double r) const {
        if (opt_.huberK <= 0.0) return 1.0;
        const double a = std::abs(r);
        return a <= opt_.huberK ? 1.0 : opt_.huberK / a;
    }

    double robustCost(const std::vector<MatchData>& matches, const Matrix3d& R,
                      const Vector3d& t) const {
        double c = 0.0;
        for (const auto& m : matches) {
            const double r = residual(m, R, t);
            const double a = std::abs(r);
            if (opt_.huberK <= 0.0 || a <= opt_.huberK) c += r * r;
            else c += 2.0 * opt_.huberK * a - opt_.huberK * opt_.huberK;
        }
        return c;
    }

    // ── Translation: init + weighted (self-consistent-field) solve ──────────
    Vector3d initialTranslation(const std::vector<MatchData>& matches, const Matrix3d& R,
                                const Vector3d& t_init) const {
        std::vector<Matrix3d> A, B;
        buildAB(matches, R, A, B);

        std::vector<Vector3d> candidates;
        if (t_init.allFinite() && t_init.norm() > 1e-6) candidates.push_back(t_init.normalized());
        {   // unweighted NEC solution
            Matrix3d M = Matrix3d::Zero();
            for (const auto& Ai : A) M += Ai;
            Eigen::SelfAdjointEigenSolver<Matrix3d> es(M);
            candidates.push_back(es.eigenvectors().col(0));
        }
        // Fibonacci points on one hemisphere (E(t) = E(-t)).
        const double golden = M_PI * (3.0 - std::sqrt(5.0));
        for (int i = 0; i < opt_.sphereSamples; ++i) {
            const double z = (i + 0.5) / opt_.sphereSamples;
            const double r = std::sqrt(std::max(0.0, 1.0 - z * z));
            const double phi = golden * i;
            candidates.emplace_back(r * std::cos(phi), r * std::sin(phi), z);
        }

        Vector3d best = candidates.front();
        double bestCost = objective(A, B, best);
        for (const auto& c : candidates) {
            const double f = objective(A, B, c);
            if (f < bestCost) { bestCost = f; best = c; }
        }
        return scf(A, B, best);
    }

    Vector3d weightedTranslation(const std::vector<MatchData>& matches, const Matrix3d& R,
                                 const Vector3d& t) const {
        std::vector<Matrix3d> A, B;
        buildAB(matches, R, A, B);
        return scf(A, B, t);
    }

    // Stationary points of sum_i tAt/tBt on the sphere satisfy E(t) t = mu t with
    // E(t) = sum_i (A_i - f_i B_i) / (tB_it), f_i = tA_it / tB_it. Iterate: t <- the
    // eigenvector of E(t)'s smallest eigenvalue, keeping only steps that lower the cost.
    Vector3d scf(const std::vector<Matrix3d>& A, const std::vector<Matrix3d>& B,
                 Vector3d t) const {
        t.normalize();
        double f = objective(A, B, t);
        for (int k = 0; k < opt_.scfIterations; ++k) {
            Matrix3d E = Matrix3d::Zero();
            for (size_t i = 0; i < A.size(); ++i) {
                const double a = t.dot(A[i] * t);
                const double b = t.dot(B[i] * t);
                E += (A[i] - (a / b) * B[i]) / b;
            }
            Eigen::SelfAdjointEigenSolver<Matrix3d> es(E);
            if (es.info() != Eigen::Success) break;
            Vector3d tn = es.eigenvectors().col(0);
            if (tn.dot(t) < 0) tn = -tn;
            const double fn = objective(A, B, tn);
            if (!(fn < f)) break;
            const double step = (tn - t).norm();
            t = tn;
            f = fn;
            if (step < 1e-10) break;
        }
        return t;
    }

    // ── Levenberg-Marquardt on R (3 params) or R and t (5 params) ───────────
    // R <- exp([d0..2]x) R;  t <- normalize(t + d3 e1 + d4 e2), e1, e2 spanning t's
    // tangent plane. Numeric central-difference Jacobian, so sigma_i's dependence on
    // R and t is part of the derivative (as with Ceres autodiff in the reference).
    static void applyStep(const Vector5d& d, bool rotationOnly, Matrix3d& R, Vector3d& t) {
        R = expSO3(d.head<3>()) * R;
        if (!rotationOnly) {
            const Vector3d e1 = t.unitOrthogonal();
            const Vector3d e2 = t.cross(e1);
            t = (t + d(3) * e1 + d(4) * e2).normalized();
        }
    }

    void refine(const std::vector<MatchData>& matches, Matrix3d& R, Vector3d& t,
                bool rotationOnly, int iterations) const {
        const int p = rotationOnly ? 3 : 5;
        const int N = static_cast<int>(matches.size());
        const double h = 1e-6;
        double mu = 1e-3;
        double cost = robustCost(matches, R, t);

        for (int it = 0; it < iterations; ++it) {
            Eigen::VectorXd r(N), sw(N);
            for (int i = 0; i < N; ++i) {
                r(i) = residual(matches[i], R, t);
                sw(i) = std::sqrt(huberWeight(r(i)));
            }
            Eigen::MatrixXd J(N, p);
            for (int k = 0; k < p; ++k) {
                Vector5d d = Vector5d::Zero();
                d(k) = h;
                Matrix3d Rp = R, Rm = R;
                Vector3d tp = t, tm = t;
                applyStep(d, rotationOnly, Rp, tp);
                applyStep(-d, rotationOnly, Rm, tm);
                for (int i = 0; i < N; ++i)
                    J(i, k) = sw(i) * (residual(matches[i], Rp, tp) - residual(matches[i], Rm, tm)) / (2 * h);
            }
            const Eigen::VectorXd rw = sw.cwiseProduct(r);
            const Eigen::MatrixXd H = J.transpose() * J;
            const Eigen::VectorXd g = J.transpose() * rw;

            bool accepted = false;
            while (mu < 1e8) {
                Eigen::MatrixXd Hd = H;
                Hd.diagonal() += mu * H.diagonal() + Eigen::VectorXd::Constant(p, 1e-12);
                const Eigen::VectorXd step = Hd.ldlt().solve(-g);
                if (!step.allFinite()) break;
                Vector5d d = Vector5d::Zero();
                d.head(p) = step;
                Matrix3d Rn = R;
                Vector3d tn = t;
                applyStep(d, rotationOnly, Rn, tn);
                const double newCost = robustCost(matches, Rn, tn);
                if (newCost < cost) {
                    const double gain = cost - newCost;
                    R = Rn; t = tn; cost = newCost;
                    mu = std::max(mu / 3.0, 1e-9);
                    accepted = true;
                    if (gain < 1e-12 * cost || step.norm() < 1e-10) return;
                    break;
                }
                mu *= 4.0;
            }
            if (!accepted) return;
        }
    }

    // ── NEC initialization: R minimising lambda_min(M(R)), t eliminated ──────
    // Residuals r_i = t*(R) . n_i(R) with t*(R) the smallest eigenvector of
    // M(R) = sum n_i n_i^T, so sum r_i^2 = lambda_min(M(R)). t*'s sign is aligned with
    // tRef so the finite differences are consistent.
    Eigen::VectorXd necResiduals(const std::vector<MatchData>& matches, const Matrix3d& R,
                                 const Vector3d& tRef, Vector3d* tOut = nullptr) const {
        const int N = static_cast<int>(matches.size());
        Eigen::MatrixXd Nmat(N, 3);
        Matrix3d M = Matrix3d::Zero();
        for (int i = 0; i < N; ++i) {
            const Vector3d n = matches[i].bearing1.cross(R * matches[i].bearing2);
            Nmat.row(i) = n.transpose();
            M += n * n.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Matrix3d> es(M);
        Vector3d t = es.eigenvectors().col(0);
        if (t.dot(tRef) < 0) t = -t;
        if (tOut) *tOut = t;
        return Nmat * t;
    }

    void necInit(const std::vector<MatchData>& matches, Matrix3d& R, int iterations) const {
        const int N = static_cast<int>(matches.size());
        const double h = 1e-6;
        double mu = 1e-3;
        Vector3d tRef;
        Eigen::VectorXd r = necResiduals(matches, R, Vector3d::UnitZ(), &tRef);
        double cost = r.squaredNorm();
        for (int it = 0; it < iterations; ++it) {
            Eigen::MatrixXd J(N, 3);
            for (int k = 0; k < 3; ++k) {
                Vector3d d = Vector3d::Zero();
                d(k) = h;
                J.col(k) = (necResiduals(matches, expSO3(d) * R, tRef)
                          - necResiduals(matches, expSO3(-d) * R, tRef)) / (2 * h);
            }
            const Matrix3d H = J.transpose() * J;
            const Vector3d g = J.transpose() * r;
            bool accepted = false;
            while (mu < 1e8) {
                Matrix3d Hd = H;
                Hd.diagonal() += mu * H.diagonal() + Vector3d::Constant(1e-12);
                const Vector3d step = Hd.ldlt().solve(-g);
                if (!step.allFinite()) break;
                const Matrix3d Rn = expSO3(step) * R;
                Vector3d tn;
                const Eigen::VectorXd rn = necResiduals(matches, Rn, tRef, &tn);
                const double newCost = rn.squaredNorm();
                if (newCost < cost) {
                    const double gain = cost - newCost;
                    R = Rn; r = rn; tRef = tn; cost = newCost;
                    mu = std::max(mu / 3.0, 1e-9);
                    accepted = true;
                    if (gain < 1e-12 * cost || step.norm() < 1e-10) return;
                    break;
                }
                mu *= 4.0;
            }
            if (!accepted) return;
        }
    }

    // Essential-matrix candidate: OpenCV's 5-point RANSAC on normalized coordinates
    // (bearing / z, so focal = 1 and the pixel threshold is divided by fx). With
    // points1 = target and points2 = current, recoverPose returns X_cur = R X_tgt + t,
    // the convention used here. Near pure rotation E is ill-defined and its R can be
    // wrong, so it competes with the previous frame's R on the NEC energy (mean
    // lambda_min per match) over the inliers; the lower one is kept.
    // Returns the inlier matches; sets R (and t_seed when the E candidate wins).
    std::vector<MatchData> initFromEssentialOrPrevious(const std::vector<MatchData>& matches,
                                                       Matrix3d& R, Vector3d& t_seed) {
        std::vector<cv::Point2d> pTgt, pCur;
        std::vector<int> map;   // index into matches (bearings with z <= 0 are skipped)
        for (int i = 0; i < static_cast<int>(matches.size()); ++i) {
            const Vector3d& b1 = matches[i].bearing1;
            const Vector3d& b2 = matches[i].bearing2;
            if (b1.z() <= 1e-6 || b2.z() <= 1e-6) continue;
            pTgt.emplace_back(b2.x() / b2.z(), b2.y() / b2.z());
            pCur.emplace_back(b1.x() / b1.z(), b1.y() / b1.z());
            map.push_back(i);
        }

        bool haveE = false;
        Matrix3d R_E = Matrix3d::Identity();
        Vector3d t_E = Vector3d::Zero();
        std::vector<MatchData> inliers;
        if (pTgt.size() >= 5) {
            cv::Mat mask;
            const double thr = opt_.essentialThresholdPx / K_(0, 0);
            cv::Mat E = cv::findEssentialMat(pTgt, pCur, 1.0, cv::Point2d(0, 0), cv::RANSAC,
                                             opt_.essentialConfidence, thr,
                                             opt_.essentialMaxIterations, mask);
            if (E.rows == 3 && E.cols == 3) {   // several solutions come stacked: use the first
                cv::Mat Rcv, tcv;
                cv::recoverPose(E, pTgt, pCur, Rcv, tcv, 1.0, cv::Point2d(0, 0), mask);
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c) R_E(r, c) = Rcv.at<double>(r, c);
                t_E = Vector3d(tcv.at<double>(0), tcv.at<double>(1), tcv.at<double>(2));
                for (int k = 0; k < mask.rows; ++k)
                    if (mask.at<uchar>(k)) inliers.push_back(matches[map[k]]);
                haveE = R_E.allFinite() && inliers.size() >= 6;
            }
        }

        if (!haveE) {
            // No usable E (typically near pure rotation, where recoverPose's cheirality
            // check rejects almost everything): NEC RANSAC warm-started from the
            // previous R, which handles pure rotation and outliers.
            logger.log("pnec", "[pnec] init: no essential matrix, NEC RANSAC from previous R");
            if (static_cast<int>(matches.size()) > opt_.ransacSampleSize)
                return ransacNec(matches, R);
            return matches;
        }

        const double energyE = necEnergy(inliers, R_E);
        const double energyPrev = necEnergy(inliers, R);
        const bool useE = energyE < energyPrev;
        if (useE) {
            R = R_E;
            t_seed = t_E;
        }
        std::ostringstream ss;
        ss << "[pnec] init: " << (useE ? "essential matrix" : "previous R")
           << " (NEC energy E=" << energyE << ", prev=" << energyPrev << "), inliers "
           << inliers.size() << "/" << matches.size();
        logger.log("pnec", ss.str());
        return inliers;
    }

    // Mean NEC energy lambda_min(M(R)) / N.
    double necEnergy(const std::vector<MatchData>& matches, const Matrix3d& R) const {
        if (matches.empty() || !R.allFinite()) return std::numeric_limits<double>::infinity();
        return necResiduals(matches, R, Vector3d::UnitZ()).squaredNorm() / matches.size();
    }

    // Inliers of hypothesis R: t* is estimated from the hypothesis' own matches
    // (support), then every match is tested against that (R, t*).
    std::vector<int> necInliers(const std::vector<MatchData>& matches,
                                const std::vector<MatchData>& support,
                                const Matrix3d& R, double thr) const {
        Vector3d th;
        necResiduals(support, R, Vector3d::UnitZ(), &th);
        std::vector<int> inliers;
        for (int i = 0; i < static_cast<int>(matches.size()); ++i) {
            const Vector3d n = matches[i].bearing1.cross(R * matches[i].bearing2);
            if (std::abs(th.dot(n)) < thr) inliers.push_back(i);
        }
        return inliers;
    }

    // RANSAC: NEC rotation from random minimal-ish subsets (warm-started from R),
    // scored by the number of matches with |t* . n_i| below the pixel threshold.
    // Returns the inliers of the best hypothesis and sets R to it.
    std::vector<MatchData> ransacNec(const std::vector<MatchData>& matches, Matrix3d& R) {
        const int N = static_cast<int>(matches.size());
        const double thr = opt_.ransacThresholdPx / K_(0, 0);
        std::mt19937 rng(12345);   // fixed seed: reproducible per frame
        std::vector<int> idx(N);
        for (int i = 0; i < N; ++i) idx[i] = i;

        Matrix3d bestR = R;
        std::vector<int> bestInliers;
        std::vector<MatchData> sample(opt_.ransacSampleSize);
        for (int it = 0; it < opt_.ransacIterations; ++it) {
            for (int k = 0; k < opt_.ransacSampleSize; ++k) {
                std::uniform_int_distribution<int> pick(k, N - 1);
                std::swap(idx[k], idx[pick(rng)]);
                sample[k] = matches[idx[k]];
            }
            Matrix3d Rh = R;
            necInit(sample, Rh, 20);
            std::vector<int> inliers = necInliers(matches, sample, Rh, thr);
            if (inliers.size() > bestInliers.size()) {
                bestInliers = inliers;
                bestR = Rh;
            }
        }
        // Local optimisation: refit NEC on the inliers and recount until stable.
        for (int lo = 0; lo < 5 && bestInliers.size() >= 6; ++lo) {
            std::vector<MatchData> in;
            for (int i : bestInliers) in.push_back(matches[i]);
            Matrix3d Rh = bestR;
            necInit(in, Rh, opt_.necIterations);
            std::vector<int> inliers = necInliers(matches, in, Rh, thr);
            if (inliers.size() <= bestInliers.size()) {
                if (inliers.size() == bestInliers.size()) bestR = Rh;
                break;
            }
            bestInliers = inliers;
            bestR = Rh;
        }
        {
            std::ostringstream ss;
            ss << "[pnec] RANSAC inliers: " << bestInliers.size() << "/" << N;
            logger.log("pnec", ss.str());
        }
        if (bestInliers.size() < 6) return matches;   // no consensus: keep everything
        R = bestR;
        std::vector<MatchData> in;
        in.reserve(bestInliers.size());
        for (int i : bestInliers) in.push_back(matches[i]);
        return in;
    }

    // ── Translation-signal guards (as in RelativePoseEstimator) ──────────────
    bool hasTranslationSignal(const std::vector<MatchData>& matches, const Matrix3d& R) {
        Matrix3d M = Matrix3d::Zero();
        std::vector<double> parallax;
        parallax.reserve(matches.size());
        for (const auto& m : matches) {
            const Vector3d Rb2 = R * m.bearing2;
            const Vector3d n = m.bearing1.cross(Rb2);
            M += n * n.transpose();
            parallax.push_back(std::atan2(n.norm(), m.bearing1.dot(Rb2)));
        }
        M /= static_cast<double>(matches.size());
        if (!M.allFinite()) {
            logger.log("pnec", "error: M not finite");
            return false;
        }
        Eigen::SelfAdjointEigenSolver<Matrix3d> es(M);
        if (es.info() != Eigen::Success) {
            logger.log("pnec", "error: Eigensolver failed");
            return false;
        }
        const Vector3d ev = es.eigenvalues();
        const double l0 = ev(0), l1 = ev(1), l2 = ev(2);
        noisePx_ = K_(0, 0) * std::sqrt(std::max(l0, 0.0));
        auto mid = parallax.begin() + parallax.size() / 2;
        std::nth_element(parallax.begin(), mid, parallax.end());
        const double parallax_px = *mid * K_(0, 0);
        {
            std::ostringstream ss;
            ss << "[pnec] eigenvalues: " << ev.transpose() << " parallax_px: " << parallax_px;
            logger.log("pnec", ss.str());
        }

        const double snr = l2 / std::max(l0, 1e-12);
        if (l2 < 1e-7 || snr < opt_.minParallaxSnr) {
            std::ostringstream ss;
            ss << "error: Insufficient parallax (lambda2=" << l2 << ", snr=" << snr << ")";
            logger.log("pnec", ss.str());
            return false;
        }
        const bool aligned = parallax_px < opt_.alignedParallaxPx;
        const double conditioning = l1 / l2;
        if (conditioning < opt_.minConditioning && aligned) {
            std::ostringstream ss;
            ss << "error: Degenerate translation (conditioning=" << conditioning
               << ", parallax_px=" << parallax_px << ")";
            logger.log("pnec", ss.str());
            return false;
        }
        const double nullspace_ratio = l0 / (l1 + 1e-8);
        if (nullspace_ratio > opt_.maxNullspaceRatio && parallax_px < opt_.ambiguousParallaxPx) {
            std::ostringstream ss;
            ss << "error: Ambiguous null space (ratio=" << nullspace_ratio
               << ", parallax_px=" << parallax_px << ")";
            logger.log("pnec", ss.str());
            return false;
        }
        return true;
    }

    // Majority vote on the triangulated depth along R*b2 (b1 || d*R*b2 + t).
    void checkAndFlipCheirality(const std::vector<MatchData>& matches,
                                       const Matrix3d& R, Vector3d& t) {
        if (t.norm() < 1e-8) return;
        int positive = 0, negative = 0;
        for (const auto& m : matches) {
            const Vector3d cross_f = m.bearing1.cross(R * m.bearing2);
            const Vector3d cross_t = m.bearing1.cross(t);
            const double den = cross_f.squaredNorm();
            if (den > 1e-8) {
                const double depth2 = -cross_t.dot(cross_f) / den;
                (depth2 > 0 ? positive : negative)++;
            }
        }
        if (negative > positive) t = -t;
        const int votes = positive + negative;
        voteMargin_ = votes > 0 ? static_cast<double>(std::max(positive, negative)) / votes : 0.0;
    }
};

}  // namespace pnec
