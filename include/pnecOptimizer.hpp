#pragma once

// #include <ceres/ceres.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <cmath>
#include <memory>

namespace pnec {

using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using Matrix2d = Eigen::Matrix2d;
using Matrix3d = Eigen::Matrix3d;
using AngleAxisd = Eigen::AngleAxisd;
using MatrixXd = Eigen::MatrixXd;

// ============================================================================
// STRUCTURES FOR DATA PASSING
// ============================================================================

struct MatchData {
    Vector3d bearing1;      // Bearing vector in frame 1
    Vector3d bearing2;      // Bearing vector in frame 2
    // double confidence;      // LightGlue confidence [0, 1]
    Matrix3d cov3d;         // 3D covariance from unscented transform
};

struct OptimizationResult {
    Matrix3d rotation;      // Estimated 3x3 rotation matrix
    double final_cost;      // Final optimization cost
    int num_iterations;     // Number of iterations
    bool success;           // Convergence success
    std::vector<double> residuals;  // Per-match residuals
};

// ============================================================================
// STEP 1: COVARIANCE ESTIMATION FROM CONFIDENCE
// ============================================================================

class CovarianceEstimator {
public:
    /**
     * Estimate 2D pixel covariance from LightGlue confidence.
     * 
     * Heuristic: std_dev = base_std / (1 + confidence_power)
     * Higher confidence -> lower uncertainty
     */
    static Matrix2d estimateFromConfidence(double confidence, 
                                           double base_std = 0.5) {
        // Confidence in [0, 1]
        double std_dev = base_std / (1.0 + 2.0 * confidence);
        Matrix2d cov_2d = Matrix2d::Identity() * (std_dev * std_dev);
        return cov_2d;
    }
    
    /**
     * Alternative: estimate from descriptor distance
     */
    static Matrix2d estimateFromDescriptorDistance(double desc_distance,
                                                  double base_std = 0.5,
                                                  double scale = 0.1) {
        double std_dev = base_std + scale * desc_distance;
        Matrix2d cov_2d = Matrix2d::Identity() * (std_dev * std_dev);
        return cov_2d;
    }
};

// ============================================================================
// STEP 2: UNSCENTED TRANSFORM (2D -> 3D PROPAGATION)
// ============================================================================

class UnscentedTransform {
public:
    /**
     * Propagate 2D covariance through unprojection to get 3D covariance.
     * 
     * This is the most expensive operation but crucial for accuracy.
     * We use eigendecomposition to generate sigma points.
     */
    Matrix3d propagate2DTo3D(const Matrix2d& cov_2d,
                                    const Matrix3d& K) {
        // Unscented transform parameters
        const int n = 2;  // 2D space
        const double lambda = 3.0 - n;  // kappa = 3 - n
        
        // Eigendecomposition of 2D covariance
        Eigen::SelfAdjointEigenSolver<Matrix2d> solver(cov_2d);
        Vector2d evals = solver.eigenvalues();
        Matrix2d evecs = solver.eigenvectors();
        
        // Generate sigma points (2n + 1 = 5 points)
        std::vector<Vector2d> sigma_points;
        sigma_points.push_back(Vector2d::Zero());  // Mean point
        
        for (int i = 0; i < n; ++i) {
            double std = std::sqrt(std::max(evals(i), 0.0) * (n + lambda));
            sigma_points.push_back(evecs.col(i) * std);
            sigma_points.push_back(evecs.col(i) * (-std));
        }
        
        // Transform sigma points through unprojection
        Matrix3d K_inv = K.inverse();
        std::vector<Vector3d> transformed_points;
        
        for (const auto& pt_2d : sigma_points) {
            // Homogeneous coordinates: [x, y, 1]
            Vector3d pt_homog(pt_2d(0), pt_2d(1), 1.0);
            Vector3d bearing = K_inv * pt_homog;
            bearing.normalize();
            transformed_points.push_back(bearing);
        }
        
        // Compute mean of transformed points (on unit sphere)
        Vector3d mean_bearing = Vector3d::Zero();
        for (const auto& p : transformed_points) {
            mean_bearing += p;
        }
        mean_bearing.normalize();
        
        // Compute covariance of transformed points
        Matrix3d cov_3d = Matrix3d::Zero();
        for (const auto& p : transformed_points) {
            Vector3d diff = p - mean_bearing;
            cov_3d += diff * diff.transpose();
        }
        cov_3d /= (2.0 * n);
        
        // Ensure positive semidefinite
        Eigen::SelfAdjointEigenSolver<Matrix3d> eigen_solver(cov_3d);
        Vector3d evals_3d = eigen_solver.eigenvalues();
        evals_3d = evals_3d.array().max(1e-8);  // Floor negative eigenvalues
        cov_3d = eigen_solver.eigenvectors() * 
                 evals_3d.asDiagonal() * 
                 eigen_solver.eigenvectors().transpose();
        
        return cov_3d;
    }
};

// // ============================================================================
// // STEP 3: PNEC RESIDUAL FOR CERES
// // ============================================================================

// struct PNECResidual {
//     Vector3d bearing1;      // f in frame 1
//     Vector3d bearing2;      // f' in frame 2
//     Matrix3d cov_3d;        // 3D covariance
//     Vector3d t;             // Translation (known)
    
//     PNECResidual(const Vector3d& f, const Vector3d& f_prime,
//                  const Matrix3d& cov, const Vector3d& trans)
//         : bearing1(f), bearing2(f_prime), cov_3d(cov), t(trans) {}
    
//     /**
//      * Compute residual for given rotation (angle-axis parameterization).
//      * 
//      * Energy: E = sum_i (e_i^2 / sigma_i^2)
//      * 
//      * Jacobian is auto-computed by Ceres.
//      */
//     template <typename T>
//     bool operator()(const T* const angle_axis_array,
//                    T* residuals) const {
//         // Unpack rotation from angle-axis
//         Eigen::Map<const Eigen::Matrix<T, 3, 1>> angle_axis(angle_axis_array);
        
//         // Avoid division by zero for small angles
//         T angle = angle_axis.norm();
//         Eigen::Matrix<T, 3, 3> R;
        
//         if (angle > T(1e-10)) {
//             Eigen::AngleAxis<T> aa(angle, angle_axis / angle);
//             R = aa.matrix();
//         } else {
//             // Use first-order approximation for small angles
//             R = Eigen::Matrix<T, 3, 3>::Identity() +
//                 skewSymmetric(angle_axis);
//         }
        
//         // Convert data to working type T
//         Eigen::Matrix<T, 3, 1> f_t = bearing1.cast<T>();
//         Eigen::Matrix<T, 3, 1> f_p_t = bearing2.cast<T>();
//         Eigen::Matrix<T, 3, 1> t_t = t.cast<T>();
//         Eigen::Matrix<T, 3, 3> cov_t = cov_3d.cast<T>();
        
//         // Epipolar error: e = t^T * (f x R*f')
//         Eigen::Matrix<T, 3, 1> Rf = R * f_p_t;
//         Eigen::Matrix<T, 3, 1> cross = f_t.cross(Rf);
//         T e = t_t.dot(cross);
        
//         // Uncertainty scaling: sigma^2 = t^T * Sigma * t
//         Eigen::Matrix<T, 3, 1> Sigma_t = cov_t * t_t;
//         T sigma_sq = t_t.dot(Sigma_t);
        
//         // Weighted residual (add epsilon to avoid division by zero)
//         T eps = T(1e-8);
//         residuals[0] = e / (T(std::sqrt(double(sigma_sq) + eps)));
        
//         return true;
//     }
    
// private:
//     template <typename T>
//     static Eigen::Matrix<T, 3, 3> skewSymmetric(const Eigen::Matrix<T, 3, 1>& v) {
//         Eigen::Matrix<T, 3, 3> result = Eigen::Matrix<T, 3, 3>::Zero();
//         result(0, 1) = -v(2);
//         result(0, 2) = v(1);
//         result(1, 0) = v(2);
//         result(1, 2) = -v(0);
//         result(2, 0) = -v(1);
//         result(2, 1) = v(0);
//         return result;
//     }
// };

// ============================================================================
// STEP 4: MAIN PNEC OPTIMIZER
// ============================================================================

// class PNECOptimizer {
// public:
//     /**
//      * Initialize PNEC optimizer.
//      * 
//      * @param K Camera intrinsic matrix (3x3)
//      * @param base_std Base standard deviation for covariance estimation
//      */
//     explicit PNECOptimizer(const Matrix3d& K = Matrix3d::Identity(),
//                            double base_std = 0.5)
//         : K_(K), base_std_(base_std) {}
    
//     /**
//      * Main optimization function.
//      * 
//      * Takes matched bearing pairs with LightGlue confidence,
//      * computes covariances, and optimizes rotation using PNEC.
//      * 
//      * @param matches Vector of MatchData structures
//      * @param t Translation vector (unit direction)
//      * @param verbose Print optimization progress
//      * @return OptimizationResult with rotation and diagnostics
//      */
//     OptimizationResult optimize(const std::vector<MatchData>& matches,
//                                 const Vector3d& t,
//                                 bool verbose = false) {
//         OptimizationResult result;
        
//         if (matches.size() < 3) {
//             result.success = false;
//             result.final_cost = std::numeric_limits<double>::infinity();
//             return result;
//         }
        
//         // Pre-compute 3D covariances
//         std::vector<Matrix3d> covariances_3d;
//         covariances_3d.reserve(matches.size());
        
//         for (const auto& match : matches) {
//             Matrix2d cov_2d = 
//                 CovarianceEstimator::estimateFromConfidence(
//                     match.confidence, base_std_);
//             Matrix3d cov_3d = 
//                 UnscentedTransform::propagate2DTo3D(cov_2d, K_);
//             covariances_3d.push_back(cov_3d);
//         }
        
//         // Setup Ceres problem
//         ceres::Problem problem;
//         double angle_axis[3] = {0.0, 0.0, 0.0};  // Identity rotation
        
//         // Add residual blocks
//         for (size_t i = 0; i < matches.size(); ++i) {
//             const auto& match = matches[i];
//             const auto& cov_3d = covariances_3d[i];
            
//             ceres::CostFunction* cost_function =
//                 new ceres::AutoDiffCostFunction<PNECResidual, 1, 3>(
//                     new PNECResidual(match.bearing1, match.bearing2, 
//                                     cov_3d, t));
            
//             problem.AddResidualBlock(cost_function, nullptr, angle_axis);
//         }
        
//         // Solver options
//         ceres::Solver::Options options;
//         options.linear_solver_type = ceres::DENSE_QR;
//         options.minimizer_progress_to_stdout = verbose;
//         options.max_num_iterations = 100;
//         options.function_tolerance = 1e-8;
//         options.gradient_tolerance = 1e-8;
//         options.parameter_tolerance = 1e-8;
        
//         // Solve
//         ceres::Solver::Summary summary;
//         ceres::Solve(options, &problem, &summary);
        
//         // Extract rotation from angle-axis
//         AngleAxisd aa(angle_axis[0], angle_axis[1], angle_axis[2]);
//         result.rotation = aa.matrix();
//         result.final_cost = summary.final_cost;
//         result.num_iterations = summary.iterations.size();
//         result.success = summary.IsSolutionUsable();
        
//         // Compute residuals for diagnostics
//         std::vector<double> residuals_vec;
//         computeResiduals(matches, covariances_3d, result.rotation, t, 
//                         residuals_vec);
//         result.residuals = residuals_vec;
        
//         return result;
//     }
    
//     /**
//      * Fast version: compute residuals without optimizing.
//      * Useful for diagnostic/validation.
//      */
//     void computeResiduals(const std::vector<MatchData>& matches,
//                          const std::vector<Matrix3d>& covariances_3d,
//                          const Matrix3d& R,
//                          const Vector3d& t,
//                          std::vector<double>& residuals) {
//         residuals.clear();
//         residuals.reserve(matches.size());
        
//         for (size_t i = 0; i < matches.size(); ++i) {
//             const auto& match = matches[i];
//             const auto& cov_3d = covariances_3d[i];
            
//             // Epipolar error
//             Vector3d Rf = R * match.bearing2;
//             Vector3d cross = match.bearing1.cross(Rf);
//             double e = t.dot(cross);
            
//             // Uncertainty scaling
//             Vector3d Sigma_t = cov_3d * t;
//             double sigma_sq = t.dot(Sigma_t);
            
//             // Residual
//             double residual = e / std::sqrt(sigma_sq + 1e-8);
//             residuals.push_back(residual);
//         }
//     }

    
    
// private:
//     Matrix3d K_;        // Camera intrinsics
//     double base_std_;   // Base standard deviation for covariance
// };


class RelativePoseEstimatorOld {
public:
    explicit RelativePoseEstimatorOld(const Matrix3d& K) : K_(K) {}

    void estimate(const std::vector<MatchData>& matches,
                  Matrix3d& R, Vector3d& t,
                  const Vector3d& t_init,
                  double& totalError) {
        Vector3d t_dir = t_init.normalized();
        solveRotation(matches, R, t_dir);
        solveTranslation(matches, R, t);
        // ── FIX 2: Cheirality Check (Resolve t sign ambiguity) ──
        checkAndFlipCheirality(matches, R, t);
        
        for (const auto& m : matches) {
            totalError += sampsonError(m.bearing1, m.bearing2, R, t);
        }
    }

private:
    Matrix3d K_;

    void solveRotation(const std::vector<MatchData>& matches, Matrix3d& R, const Vector3d& t_dir) 
    {
        // // ── Check inputs ──────────────────────────────────────────
        // if (!R.allFinite()) {
        //     std::cout << "[pnec] R_init is not finite\n";
        //     return;
        // }
        // if (!t_dir.allFinite() || t_dir.norm() < 1e-8) {
        //     std::cout << "[pnec] t_dir is not finite or zero: " << t_dir.transpose() << "\n";
        //     return;
        // }
        // if (matches.empty()) {
        //     std::cout << "[pnec] no matches\n";
        //     return;
        // }

        for (int iter = 0; iter < 10; iter++) {
            Matrix3d JtJ = Matrix3d::Zero();
            Vector3d Jtr = Vector3d::Zero();

            // for (const auto& m : matches) {
            for (int i = 0; i < matches.size(); i++) {
                const auto& m = matches[i];

                // // ── Check each match ──────────────────────────────
                // if (!m.bearing1.allFinite() || !m.bearing2.allFinite()) {
                //     std::cout << "[pnec] match " << i << " bearing not finite\n"
                //             << "  b1: " << m.bearing1.transpose() << "\n"
                //             << "  b2: " << m.bearing2.transpose() << "\n";
                //     continue;
                // }
                // if (!m.cov3d.allFinite()) {
                //     std::cout << "[pnec] match " << i << " cov3d not finite\n";
                //     continue;
                // }

                Vector3d Rf    = R * m.bearing2;
                Vector3d cross = m.bearing1.cross(Rf);

                double sigma_sq = t_dir.dot(m.cov3d * t_dir);
                double sigma    = std::sqrt(sigma_sq + 1e-8);
                double r        = t_dir.dot(cross) / sigma;

                Matrix3d skew_Rf, skew_f;
                skew_Rf <<             0, -Rf(2),  Rf(1),
                              Rf(2),       0, -Rf(0),
                             -Rf(1),  Rf(0),       0;
                skew_f  <<                    0, -m.bearing1(2),  m.bearing1(1),
                              m.bearing1(2),                  0, -m.bearing1(0),
                             -m.bearing1(1),  m.bearing1(0),                  0;

                Vector3d J = (-skew_f * skew_Rf).transpose() * t_dir / sigma;

                JtJ        += J * J.transpose();
                Jtr        += J * r;
            }

            Vector3d delta = JtJ.ldlt().solve(-Jtr);

            // if (verbose)
            //     std::cout << "iter " << iter
            //               << "  delta=" << delta.norm()
            //               << "  sampson=" << total_error / matches.size() << "\n";

            if (delta.norm() < 1e-6) break; // ✓ check before update
            R = Eigen::AngleAxisd(delta.norm(), delta.normalized()).matrix() * R;
        }
    }

    void solveTranslation(const std::vector<MatchData>& matches,
                          const Matrix3d& R,
                          Vector3d& t) {
        Matrix3d AtA = Matrix3d::Zero();
        for (const auto& m : matches) {
            Vector3d n = m.bearing1.cross(R * m.bearing2);
            AtA += n * n.transpose();
        }
        // ── FIX 1: Normalize by N to make absolute thresholds scale-invariant ──
        // double N = static_cast<double>(matches.size());
        // AtA /= N;

        // ── Guard: AtA must be valid ──────────────────────────────
        if (!R.allFinite()) {
            std::cout << "[pnec] R is not finite:\n" << R << "\n";
            t = Vector3d::Zero();
            // t_reliability = 0.0;
            return;
        }
        if (!AtA.allFinite()) {
            t = Vector3d::Zero();
            // t_reliability = 0.0;
            std::cout << "[pnec] AtA not finite\n";
            return;
        }
        Eigen::SelfAdjointEigenSolver<Matrix3d> solver(AtA);

        if (solver.info() != Eigen::Success) {
            t = Vector3d::Zero();
            // t_reliability = 0.0;
            std::cout << "[pnec] Eigensolver failed\n";
            return;
        }

        Vector3d eigenvalues = solver.eigenvalues();

        // ── Print for debugging ───────────────────────────────────
        std::cout << "[pnec] eigenvalues: " << eigenvalues.transpose() << "\n";


        // Vector3d eigenvalues = solver.eigenvalues();

        // Ratio of smallest to second smallest eigenvalue
        // Eigenvalues sorted ascending: λ₀ ≤ λ₁ ≤ λ₂
        double lambda0 = eigenvalues(0);  // should be near 0 (null space)
        double lambda1 = eigenvalues(1);  // should be >> 0 for good translation
        double lambda2 = eigenvalues(2);  // largest eigenvalue

        // Good translation: λ₁ is large → gap is large
        // Pure rotation:    λ₁ ≈ 0     → gap is near zero
        double gap = lambda1 - lambda0;

        // Normalize by largest eigenvalue to make it scale-invariant
        // double t_reliability = gap / (lambda2 + 1e-8);

        // t_reliability ≈ 1.0 → well conditioned (good translation)
        // t_reliability ≈ 0.0 → degenerate (pure rotation)

        // if (t_reliability < 0.1 || (lambda1 < 1e-2 && lambda2 < 1e-2)) {
        //     // Degenerate — pure rotation or near-pure rotation
        //     t = Vector3d::Zero();  // signal that t is unreliable
        //     return;
        // }
        // 1. Must have minimum total parallax energy
        if (lambda2 < 1e-3) {
            t = Vector3d::Zero(); // Pure rotation / zero baseline
            return;
        }
        // 2. Nullspace check: λ₀ must be significantly smaller than λ₁ (e.g., at least 5x-10x smaller)
        // In your runs: λ₁ / λ₀ ≈ 76x, which is a strong pass.
        if (lambda1 / (lambda0 + 1e-8) < 5) {
            t = Vector3d::Zero(); // Ambiguous translation direction
            return;
        }
        // std::cout << " [pnec Optimizer] Translation reliability: " << t_reliability << std::endl;

        t = solver.eigenvectors().col(0);
        t = t.normalized();
        std::cout << " [pnec translation Optimizer] Translation: " << t << std::endl;
    }

    // ── FIX 2 Helper: Depth test for Cheirality ──
    void checkAndFlipCheirality(const std::vector<MatchData>& matches, const Matrix3d& R, Vector3d& t) {
        if (t.norm() < 1e-8) return;

        int positive_depth_count = 0;
        int negative_depth_count = 0;

        for (const auto& m : matches) {
            Vector3d Rf2 = R * m.bearing2;
            Vector3d f1 = m.bearing1;

            // Approximate depth by triangulating
            // f1 x (depth2 * Rf2 + t) = 0
            Vector3d cross_f = f1.cross(Rf2);
            Vector3d cross_t = f1.cross(t);
            
            double den = cross_f.squaredNorm();
            if (den > 1e-8) {
                double depth2 = -cross_t.dot(cross_f) / den;
                if (depth2 > 0) {
                    positive_depth_count++;
                } else {
                    negative_depth_count++;
                }
            }
        }

        // If majority of points fall behind the camera, the translation sign is backwards
        if (negative_depth_count > positive_depth_count) {
            t = -t;
        }
    }

    double sampsonError(const Vector3d& f, const Vector3d& f_prime,
                        const Matrix3d& R, const Vector3d& t_dir) {
        Matrix3d t_skew;
        t_skew <<          0, -t_dir(2),  t_dir(1),
                  t_dir(2),           0, -t_dir(0),
                 -t_dir(1),  t_dir(0),           0;
        Matrix3d E = t_skew * R;

        double num = f.dot(E * f_prime);
        num *= num;

        Vector3d Ef  = E * f_prime;
        Vector3d Etf = E.transpose() * f;
        double den = Ef(0)*Ef(0)  + Ef(1)*Ef(1)
                   + Etf(0)*Etf(0) + Etf(1)*Etf(1);

        return num / (den + 1e-8); // ✓ guard against zero denominator
    }
};


class RelativePoseEstimator {
public:
    explicit RelativePoseEstimator(const Matrix3d& K) : K_(K) {}

    void estimate(const std::vector<MatchData>& matches,
                  Matrix3d& R, Vector3d& t,
                  const Vector3d& t_init,
                  double& totalError) {
                  
        Vector3d t_dir = t_init.normalized();
        if (!t_dir.allFinite() || t_dir.norm() < 1e-6) {
            t_dir = Vector3d(1, 0, 0); // Fallback to avoid NaNs
        }

        // ── FIX 1: Alternating Optimization (Jointly update R and t) ──
        const int max_outer_iters = 5;
        for (int outer_iter = 0; outer_iter < max_outer_iters; outer_iter++) {
            solveRotation(matches, R, t_dir);
            solveTranslation(matches, R, t);
            
            // ── FIX 2: Cheirality Check (Resolve t sign ambiguity) ──
            checkAndFlipCheirality(matches, R, t);
            
            if (t.norm() > 1e-8) {
                t_dir = t.normalized();
            }
        }

        // Calculate final error
        totalError = 0.0;
        for (const auto& m : matches) {
            totalError += sampsonError(m.bearing1, m.bearing2, R, t_dir);
        }
    }

private:
    Matrix3d K_;

    void solveRotation(const std::vector<MatchData>& matches, Matrix3d& R, const Vector3d& t_dir) 
    {
        double lambda = 1e-3; // FIX 4: Levenberg-Marquardt / Tikhonov damping factor

        for (int iter = 0; iter < 10; iter++) {
            Matrix3d JtJ = Matrix3d::Zero();
            Vector3d Jtr = Vector3d::Zero();

            for (const auto& m : matches) {
                Vector3d Rf    = R * m.bearing2;
                Vector3d cross = m.bearing1.cross(Rf);

                // FIX 5 mitigation: Using provided cov3d, but adding robust floor
                double sigma_sq = t_dir.dot(m.cov3d * t_dir);
                double sigma    = std::sqrt(sigma_sq + 1e-8);
                double r        = t_dir.dot(cross) / sigma;

                Matrix3d skew_Rf, skew_f;
                skew_Rf <<            0, -Rf(2),  Rf(1),
                              Rf(2),      0, -Rf(0),
                             -Rf(1),  Rf(0),      0;
                skew_f  <<                 0, -m.bearing1(2),  m.bearing1(1),
                              m.bearing1(2),               0, -m.bearing1(0),
                             -m.bearing1(1),  m.bearing1(0),               0;

                Vector3d J = (-skew_f * skew_Rf).transpose() * t_dir / sigma;

                // ── FIX 3: Robust Huber Loss ──
                double abs_r = std::abs(r);
                double delta_huber = 1.345; // 95% efficiency tuning constant
                double weight = (abs_r <= delta_huber) ? 1.0 : (delta_huber / abs_r);

                JtJ += weight * (J * J.transpose());
                Jtr += weight * (J * r);
            }

            // ── FIX 4: Apply Damping to prevent instability ──
            JtJ += lambda * Matrix3d::Identity();

            Vector3d delta = JtJ.ldlt().solve(-Jtr);
            double step_norm = delta.norm();

            if (step_norm < 1e-6 || !std::isfinite(step_norm)) break; 

            // Safe angle-axis update (avoids NaN if delta is perfectly 0)
            R = Eigen::AngleAxisd(step_norm, delta / step_norm).matrix() * R;
        }
    }

    void solveTranslation(const std::vector<MatchData>& matches,
                          const Matrix3d& R,
                          Vector3d& t) {
        Matrix3d AtA = Matrix3d::Zero();
        for (const auto& m : matches) {
            Vector3d n = m.bearing1.cross(R * m.bearing2);
            AtA += n * n.transpose();
        }
        
        if (!R.allFinite() || !AtA.allFinite()) {
            t = Vector3d::Zero();
            return;
        }
        
        Eigen::SelfAdjointEigenSolver<Matrix3d> solver(AtA);

        if (solver.info() != Eigen::Success) {
            t = Vector3d::Zero();
            return;
        }

        Vector3d eigenvalues = solver.eigenvalues();
        double lambda0 = eigenvalues(0);  
        double lambda1 = eigenvalues(1);  
        double lambda2 = eigenvalues(2);  

        double gap = lambda1 - lambda0;
        double t_reliability = gap / (lambda2 + 1e-8);

        if (t_reliability < 0.1 || (lambda1 < 1e-2 && lambda2 < 1e-2)) {
            t = Vector3d::Zero(); 
            return;
        }

        t = solver.eigenvectors().col(0);
    }

    // ── FIX 2 Helper: Depth test for Cheirality ──
    void checkAndFlipCheirality(const std::vector<MatchData>& matches, const Matrix3d& R, Vector3d& t) {
        if (t.norm() < 1e-8) return;

        int positive_depth_count = 0;
        int negative_depth_count = 0;

        for (const auto& m : matches) {
            Vector3d Rf2 = R * m.bearing2;
            Vector3d f1 = m.bearing1;

            // Approximate depth by triangulating
            // f1 x (depth2 * Rf2 + t) = 0
            Vector3d cross_f = f1.cross(Rf2);
            Vector3d cross_t = f1.cross(t);
            
            double den = cross_f.squaredNorm();
            if (den > 1e-8) {
                double depth2 = -cross_t.dot(cross_f) / den;
                if (depth2 > 0) {
                    positive_depth_count++;
                } else {
                    negative_depth_count++;
                }
            }
        }

        // If majority of points fall behind the camera, the translation sign is backwards
        if (negative_depth_count > positive_depth_count) {
            t = -t;
        }
    }

    double sampsonError(const Vector3d& f, const Vector3d& f_prime,
                        const Matrix3d& R, const Vector3d& t_dir) {
        if (t_dir.norm() < 1e-8) return 0.0; // Prevent NaN

        Matrix3d t_skew;
        t_skew <<          0, -t_dir(2),  t_dir(1),
                    t_dir(2),         0, -t_dir(0),
                   -t_dir(1),  t_dir(0),         0;
        Matrix3d E = t_skew * R;

        double num = f.dot(E * f_prime);
        num *= num;

        Vector3d Ef  = E * f_prime;
        Vector3d Etf = E.transpose() * f;
        double den = Ef(0)*Ef(0)  + Ef(1)*Ef(1)
                   + Etf(0)*Etf(0) + Etf(1)*Etf(1);

        return num / (den + 1e-8); 
    }
};


}  // namespace pnec