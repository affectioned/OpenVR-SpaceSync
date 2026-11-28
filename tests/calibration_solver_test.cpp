// SPDX-License-Identifier: AGPL-3.0-only
// Tests for the pure-math calibration pipeline in include/overlay/CalibrationMath.h.
//
// Covers the three features that are testable in isolation:
//   - IRLS-reweighted Kabsch: clean samples, outlier injection, low-weight prior.
//   - Gravity-lock projection: small pitch/roll snaps to zero, large pitch/roll kept.
//   - Continuous-recal validator: in-bounds accept, yaw exceed, translation exceed.
//
// Standalone binary, no gtest. ctest just checks the exit code.
//
// Build: enabled under ENABLE_SPACESYNC_TESTS.

#include "CalibrationMath.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

using calmath::DSample;
using calmath::IRLSStats;

static int failures = 0;

#define CHECK(cond, ...) do { \
    if (!(cond)) { \
        std::fprintf(stderr, "FAIL (%s:%d): %s\n  ", __FILE__, __LINE__, #cond); \
        std::fprintf(stderr, __VA_ARGS__); \
        std::fprintf(stderr, "\n"); \
        failures++; \
    } \
} while (0)

#define SECTION(name) std::printf("\n== %s ==\n", name)

// Build a 3-DOF rotation from yaw/pitch/roll degrees (SteamVR's ZYX
// convention, same as Calibration.cpp uses).
static Eigen::Matrix3d MakeRot(double yawDeg, double pitchDeg, double rollDeg)
{
    double y = yawDeg   * EIGEN_PI / 180.0;
    double p = pitchDeg * EIGEN_PI / 180.0;
    double r = rollDeg  * EIGEN_PI / 180.0;
    return (Eigen::AngleAxisd(r, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(p, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

// Synthesize N delta samples where each ref = truth * target + noise.
static std::vector<DSample> MakeDeltas(const Eigen::Matrix3d& truth,
                                       int n, double noiseRad, uint32_t seed)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> axU(-1.0, 1.0);
    std::uniform_real_distribution<double> angU(0.1, 1.5);  // radians
    std::normal_distribution<double> noiseU(0.0, noiseRad);

    std::vector<DSample> out;
    out.reserve(n);
    for (int i = 0; i < n; i++)
    {
        Eigen::Vector3d axis(axU(rng), axU(rng), axU(rng));
        if (axis.norm() < 1e-6) axis = Eigen::Vector3d::UnitY();
        axis.normalize();
        double angle = angU(rng);
        Eigen::Vector3d targetVec = axis * angle;

        Eigen::Vector3d refVec = truth * targetVec;
        refVec += Eigen::Vector3d(noiseU(rng), noiseU(rng), noiseU(rng));

        DSample d;
        d.valid = true;
        d.target = targetVec;
        d.ref = refVec;
        d.weight = 1.0;
        out.push_back(d);
    }
    return out;
}

static double AngleBetweenRot(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b)
{
    Eigen::AngleAxisd d(a.transpose() * b);
    return std::fabs(d.angle()) * 180.0 / EIGEN_PI;
}

// --- Test 1: clean samples converge to truth ---
static void TestCleanKabsch()
{
    SECTION("IRLS clean");
    Eigen::Matrix3d truth = MakeRot(37.0, 0.0, 0.0);
    auto deltas = MakeDeltas(truth, 30, 0.002, 1u);
    std::vector<double> priors(deltas.size(), 1.0);

    IRLSStats stats;
    Eigen::Matrix3d fit = calmath::SolveKabschIRLS(deltas, priors, &stats, 3);
    double err = AngleBetweenRot(truth, fit);
    std::printf("  fit error=%.4f deg, RMS %.4f -> %.4f deg, rejected=%d\n",
                err, stats.initialRmsRad * 180 / EIGEN_PI,
                stats.finalRmsRad * 180 / EIGEN_PI, stats.rejected);
    CHECK(err < 0.1, "clean fit should land within 0.1 deg of truth, got %.4f", err);
    CHECK(stats.rejected == 0, "clean samples should produce no rejections, got %d", stats.rejected);
    // Raw RMS is unweighted; IRLS minimises the weighted version, so we
    // only assert the raw RMS is still in the same ballpark, not strictly
    // non-increasing.
    CHECK(stats.finalRmsRad < stats.initialRmsRad * 1.05 + 1e-6,
          "IRLS shouldn't inflate raw RMS by more than 5%% on clean data");
}

// --- Test 2: outliers get down-weighted, fit still correct ---
static void TestOutlierRejection()
{
    SECTION("IRLS with 5 injected outliers");
    Eigen::Matrix3d truth = MakeRot(20.0, 0.0, 0.0);
    auto deltas = MakeDeltas(truth, 30, 0.002, 2u);

    // Inject 5 outliers by overwriting their ref vectors with a wildly
    // different rotation (yaw 90 deg).
    Eigen::Matrix3d wrong = MakeRot(90.0, 0.0, 0.0);
    std::vector<int> outlierIdx = { 2, 7, 13, 19, 25 };
    for (int k : outlierIdx)
        deltas[k].ref = wrong * deltas[k].target;

    std::vector<double> priors(deltas.size(), 1.0);
    IRLSStats stats;
    Eigen::Matrix3d fit = calmath::SolveKabschIRLS(deltas, priors, &stats, 3);
    double err = AngleBetweenRot(truth, fit);
    std::printf("  fit error=%.4f deg, final rejected=%d, down-weighted=%d\n",
                err, stats.rejected, stats.downWeighted);
    CHECK(err < 0.3, "with 5 outliers, fit should still land within 0.3 deg of truth, got %.4f", err);
    CHECK(stats.rejected >= 4, "at least 4 of 5 injected outliers should be rejected (robust<0.1), got %d", stats.rejected);

    // All 5 injected outliers should have robust factors below 0.1 individually.
    int flagged = 0;
    for (int k : outlierIdx)
        if (stats.finalRobustFactors[k] < 0.1) flagged++;
    CHECK(flagged >= 4, "at least 4 of the 5 specific outliers should be flagged, got %d", flagged);
}

// --- Test 3: low-weight prior still converges ---
static void TestLowWeightPrior()
{
    SECTION("IRLS with mixed-weight prior");
    Eigen::Matrix3d truth = MakeRot(-15.0, 0.0, 0.0);
    auto deltas = MakeDeltas(truth, 40, 0.002, 3u);

    std::vector<double> priors(deltas.size());
    for (size_t i = 0; i < deltas.size(); i++)
        priors[i] = (i % 2 == 0) ? 0.1 : 1.0;  // alternating low/high-confidence

    IRLSStats stats;
    Eigen::Matrix3d fit = calmath::SolveKabschIRLS(deltas, priors, &stats, 3);
    double err = AngleBetweenRot(truth, fit);
    std::printf("  fit error=%.4f deg\n", err);
    CHECK(err < 0.15, "mixed-weight prior should still converge within 0.15 deg, got %.4f", err);
}

// --- Test 4: gravity-lock ---
static void TestGravityLock()
{
    SECTION("ApplyGravityLock");

    // Case A: tiny pitch + roll should snap to zero.
    Eigen::Vector3d small(0.8, 37.0, 1.5);  // ZYX = roll, yaw, pitch
    bool applied = false;
    Eigen::Vector3d outA = calmath::ApplyGravityLock(small, 3.0, applied);
    CHECK(applied, "small pitch+roll should trigger snap");
    CHECK(std::fabs(outA[0]) < 1e-9, "roll should be zeroed, got %.4f", outA[0]);
    CHECK(std::fabs(outA[2]) < 1e-9, "pitch should be zeroed, got %.4f", outA[2]);
    CHECK(std::fabs(outA[1] - 37.0) < 1e-9, "yaw should be preserved, got %.4f", outA[1]);

    // Case B: pitch exceeds threshold → keep both.
    Eigen::Vector3d largePitch(0.5, 20.0, 5.0);
    Eigen::Vector3d outB = calmath::ApplyGravityLock(largePitch, 3.0, applied);
    CHECK(!applied, "large pitch should skip the snap");
    CHECK(outB == largePitch, "input should pass through unchanged on skip");

    // Case C: roll exceeds threshold → keep both.
    Eigen::Vector3d largeRoll(5.0, 20.0, 0.5);
    Eigen::Vector3d outC = calmath::ApplyGravityLock(largeRoll, 3.0, applied);
    CHECK(!applied, "large roll should skip the snap");
    CHECK(outC == largeRoll, "input should pass through unchanged on skip");

    // Case D: Eigen's other euler branch for a nearly level rotation (real
    // calibration log: roll 179.36, yaw -27.35, pitch -179.74). It is
    // roll -0.64 / yaw -152.65 / pitch 0.26 and must snap.
    Eigen::Vector3d flipped(179.36, -27.35, -179.74);
    auto toMat = [](const Eigen::Vector3d& e) {
        Eigen::Vector3d r = e * EIGEN_PI / 180.0;
        return (Eigen::AngleAxisd(r(0), Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(r(1), Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(r(2), Eigen::Vector3d::UnitX())).toRotationMatrix();
    };
    Eigen::Vector3d canon = calmath::CanonicalizeEulerZYX(flipped);
    double same = (toMat(canon) - toMat(flipped)).norm();
    CHECK(same < 1e-9, "canonical euler must be the same rotation, matrix diff %.3g", same);
    CHECK(std::fabs(canon[0] + 0.64) < 1e-6 && std::fabs(canon[2] - 0.26) < 1e-6,
        "canonical tilt should be roll -0.64 / pitch 0.26, got %.4f / %.4f", canon[0], canon[2]);
    Eigen::Vector3d outD = calmath::ApplyGravityLock(flipped, 3.0, applied);
    CHECK(applied, "flipped-branch level rotation should snap");
    CHECK(std::fabs(outD[1] + 152.65) < 1e-6, "yaw should be -152.65, got %.4f", outD[1]);
}

// --- Test 5: continuous-recal validator ---
static void TestContRecalGate()
{
    SECTION("ContRecalAccept");
    calmath::ContRecalGate gate{ 2.0, 0.05 };  // 2 deg yaw, 5 cm trans

    // In bounds.
    Eigen::Vector3d a(0, 30.0, 0), b(0, 31.0, 0);
    Eigen::Vector3d ta(0, 0, 0), tb(2.0, 0, 0);  // 2 cm → 0.02 m
    double yawD = 0, transD = 0;
    bool ok = calmath::ContRecalAccept(a, b, ta, tb, gate, yawD, transD);
    CHECK(ok, "within-bounds shift should accept");
    CHECK(std::fabs(yawD + 1.0) < 1e-9, "yaw delta should be -1 deg, got %.4f", yawD);
    CHECK(std::fabs(transD - 0.02) < 1e-9, "trans delta should be 0.02 m, got %.4f", transD);

    // Yaw exceeds.
    Eigen::Vector3d c(0, 35.0, 0);
    ok = calmath::ContRecalAccept(c, b, ta, tb, gate, yawD, transD);
    CHECK(!ok, "yaw shift of 4 deg should reject");

    // Translation exceeds.
    Eigen::Vector3d tc(10.0, 0, 0);  // 10 cm → 0.10 m
    ok = calmath::ContRecalAccept(a, b, tc, tb, gate, yawD, transD);
    CHECK(!ok, "translation shift of 8 cm should reject");

    // Wrap-around at 180: 179 - (-179) = 358, which normalises to -2.
    // Magnitude is 2.0, exactly on the bound → accept.
    Eigen::Vector3d dpos(0, 179.0, 0), dneg(0, -179.0, 0);
    ok = calmath::ContRecalAccept(dpos, dneg, ta, ta, gate, yawD, transD);
    CHECK(ok, "179 vs -179 should wrap to ±2 deg and accept at 2.0 bound");
    CHECK(std::fabs(std::fabs(yawD) - 2.0) < 1e-9,
          "wrap should produce |yawD|=2 deg, got %.4f", yawD);

    // Wrap just past bound: 179 - (-178) = 357 → -3. Reject.
    Eigen::Vector3d dneg2(0, -178.0, 0);
    ok = calmath::ContRecalAccept(dpos, dneg2, ta, ta, gate, yawD, transD);
    CHECK(!ok, "wrap of 3 deg should reject at 2.0 bound");
}

int main()
{
    std::printf("calibration_solver_test: CalibrationMath.h\n");
    TestCleanKabsch();
    TestOutlierRejection();
    TestLowWeightPrior();
    TestGravityLock();
    TestContRecalGate();

    if (failures)
    {
        std::fprintf(stderr, "\n%d assertion(s) failed.\n", failures);
        return 1;
    }
    std::printf("\nall tests passed.\n");
    return 0;
}
