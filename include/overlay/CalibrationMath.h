// SPDX-License-Identifier: AGPL-3.0-only
// Added by Shinyflvres, 2026-11-20. Part of SpaceSync, a modified version
// of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md
//
// Pure-math slice of the calibration pipeline, header-only so tests and the
// real code share the same implementation. Only pulls in Eigen; no OpenVR,
// no CalCtx, no logging. Any symbol that logs or mutates program state
// belongs in Calibration.cpp, not here.

#pragma once

#include <Dense>
#include <algorithm>
#include <cmath>
#include <vector>

namespace calmath {

// Delta between a pair of calibration samples, in rotation-vector form.
// The weight is the prior from the Frame-confidence pair heuristic; the
// IRLS loop then multiplies that by a Cauchy robust factor each iteration.
struct DSample
{
    bool valid = false;
    Eigen::Vector3d ref = Eigen::Vector3d::Zero();
    Eigen::Vector3d target = Eigen::Vector3d::Zero();
    double weight = 1.0;
};

// Per-run IRLS stats so callers (and tests) can see residual convergence.
struct IRLSStats
{
    double initialRmsRad = 0.0;
    double finalRmsRad = 0.0;
    double cauchyScaleRad = 0.0;
    int downWeighted = 0;   // final weight (robust factor) below 0.5
    int rejected = 0;       // final weight (robust factor) below 0.1
    int iterations = 0;
    std::vector<double> finalRobustFactors;
};

// One weighted Kabsch pass. deltas and weights must have the same size.
// Weights are expected to be non-negative; negatives are clamped to zero.
inline Eigen::Matrix3d SolveKabschWeighted(const std::vector<DSample>& deltas,
                                           const std::vector<double>& weights)
{
    const size_t n = deltas.size();
    Eigen::MatrixXd refPts(n, 3), tgtPts(n, 3);
    for (size_t i = 0; i < n; i++)
    {
        double s = std::sqrt((std::max)(0.0, weights[i]));
        refPts.row(i) = s * deltas[i].ref;
        tgtPts.row(i) = s * deltas[i].target;
    }
    Eigen::Matrix3d cross = refPts.transpose() * tgtPts;

    Eigen::BDCSVD<Eigen::MatrixXd> svd;
    svd.compute(cross, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::Matrix3d flip = Eigen::Matrix3d::Identity();
    if ((svd.matrixU() * svd.matrixV().transpose()).determinant() < 0)
        flip(2, 2) = -1;

    Eigen::Matrix3d rot = svd.matrixV() * flip * svd.matrixU().transpose();
    rot.transposeInPlace();
    return rot;
}

// IRLS-reweighted Kabsch. Starts from priorWeights (clamped non-negative)
// and runs `iterations` passes of: compute per-delta residual against
// current rot, derive Cauchy scale from 2*MAD (floored at 1 deg), update
// weights, solve again. Three iterations is usually enough; fewer if the
// prior is already clean.
inline Eigen::Matrix3d SolveKabschIRLS(const std::vector<DSample>& deltas,
                                       const std::vector<double>& priorWeights,
                                       IRLSStats* stats = nullptr,
                                       int iterations = 3)
{
    std::vector<double> w(priorWeights);
    Eigen::Matrix3d rot = SolveKabschWeighted(deltas, w);

    IRLSStats local{};
    const double minScale = 1.0 * EIGEN_PI / 180.0;

    for (int iter = 0; iter < iterations; iter++)
    {
        std::vector<double> residuals(deltas.size());
        for (size_t k = 0; k < deltas.size(); k++)
        {
            Eigen::Vector3d targetInRef = rot * deltas[k].target;
            residuals[k] = (deltas[k].ref - targetInRef).norm();
        }

        double sumSq = 0.0;
        for (double r : residuals) sumSq += r * r;
        double rms = deltas.empty() ? 0.0 : std::sqrt(sumSq / deltas.size());
        if (iter == 0) local.initialRmsRad = rms;
        local.finalRmsRad = rms;

        std::vector<double> sorted = residuals;
        std::sort(sorted.begin(), sorted.end());
        double median = sorted.empty() ? 0.0 : sorted[sorted.size() / 2];
        std::vector<double> absDev(sorted.size());
        for (size_t k = 0; k < sorted.size(); k++)
            absDev[k] = std::fabs(sorted[k] - median);
        std::sort(absDev.begin(), absDev.end());
        double mad = absDev.empty() ? 0.0 : absDev[absDev.size() / 2];
        double scale = (std::max)(2.0 * mad, minScale);
        local.cauchyScaleRad = scale;

        local.downWeighted = 0;
        local.rejected = 0;
        local.finalRobustFactors.assign(deltas.size(), 1.0);
        for (size_t k = 0; k < deltas.size(); k++)
        {
            double r = residuals[k] / scale;
            double robust = 1.0 / (1.0 + r * r);
            local.finalRobustFactors[k] = robust;
            w[k] = priorWeights[k] * robust;
            if (robust < 0.5) local.downWeighted++;
            if (robust < 0.1) local.rejected++;
        }

        rot = SolveKabschWeighted(deltas, w);
        local.iterations = iter + 1;
    }

    if (stats) *stats = local;
    return rot;
}

inline double WrapDeg180(double d)
{
    while (d > 180.0) d -= 360.0;
    while (d <= -180.0) d += 360.0;
    return d;
}

// Eigen's eulerAngles(2,1,0) returns the first (Z = roll) angle in [0, 180],
// so a level rotation with a slightly negative roll comes out on the other
// branch: roll ~ +180, pitch ~ +-180, yaw mirrored. Rz(a)Ry(b)Rx(c) equals
// Rz(a+180)Ry(180-b)Rx(c+180), so both triples are the same rotation; pick
// the one with the smaller tilt so "level" reads as level (otherwise the
// gravity lock sees a 180 deg pitch and refuses to snap).
inline Eigen::Vector3d CanonicalizeEulerZYX(const Eigen::Vector3d& eulerDeg)
{
    Eigen::Vector3d a(WrapDeg180(eulerDeg[0]), WrapDeg180(eulerDeg[1]), WrapDeg180(eulerDeg[2]));
    Eigen::Vector3d b(WrapDeg180(a[0] + 180.0), WrapDeg180(180.0 - a[1]), WrapDeg180(a[2] + 180.0));
    double tiltA = std::fabs(a[0]) + std::fabs(a[2]);
    double tiltB = std::fabs(b[0]) + std::fabs(b[2]);
    return tiltB < tiltA ? b : a;
}

// Snap pitch/roll to zero when both are small, keeping only yaw. Returns
// the possibly-modified euler triple (ZYX = roll/yaw/pitch, degrees) and
// sets `appliedOut` true when the snap happened.
inline Eigen::Vector3d ApplyGravityLock(const Eigen::Vector3d& eulerDeg,
                                        double thresholdDeg,
                                        bool& appliedOut)
{
    Eigen::Vector3d out = CanonicalizeEulerZYX(eulerDeg);
    double pitch = out[2], roll = out[0];
    if (std::fabs(pitch) < thresholdDeg && std::fabs(roll) < thresholdDeg)
    {
        out[0] = 0.0;
        out[2] = 0.0;
        appliedOut = true;
    }
    else
    {
        appliedOut = false;
    }
    return out;
}

// Validator for continuous recalibration — returns true if the proposed
// shift is small enough to apply silently. Caller is responsible for the
// rotation/translation units: both euler triples are in degrees (ZYX),
// both translations in centimetres (same as CalibrationContext).
struct ContRecalGate
{
    double maxYawDeltaDeg;
    double maxTransDeltaM;
};

inline bool ContRecalAccept(const Eigen::Vector3d& newEulerDeg,
                            const Eigen::Vector3d& oldEulerDeg,
                            const Eigen::Vector3d& newTransCm,
                            const Eigen::Vector3d& oldTransCm,
                            const ContRecalGate& gate,
                            double& yawDeltaDegOut,
                            double& transDeltaMOut)
{
    double d = newEulerDeg[1] - oldEulerDeg[1];
    while (d > 180.0)  d -= 360.0;
    while (d < -180.0) d += 360.0;
    yawDeltaDegOut = d;
    transDeltaMOut = (newTransCm - oldTransCm).norm() * 0.01;
    return std::fabs(d) <= gate.maxYawDeltaDeg
        && transDeltaMOut   <= gate.maxTransDeltaM;
}

} // namespace calmath
