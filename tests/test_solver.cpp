#include "example_geometry.hpp"
#include "rss1u/solver.hpp"
#include "test_geometries.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
    if (!condition) {
        ++failures;
        std::cerr << "[FAIL] " << message << '\n';
    }
}

void checkNear(double actual, double expected, double tolerance,
               const std::string& message) {
    check(std::abs(actual - expected) <= tolerance,
          message + " actual=" + std::to_string(actual)
          + " expected=" + std::to_string(expected));
}

void checkRoundTrip(
    const rss1u::Geometry& geometry,
    const rss1u::Vec2& target,
    const std::string& name) {
    using namespace rss1u;

    Solver solver(geometry);
    check(solver.geometryValid(), name + ": geometry is valid");
    const InverseResult zero = solver.inverse(Vec2::Zero(), Vec2::Zero());
    check(zero.ok(), name + ": zero-pose IK succeeds");
    if (zero.ok()) {
        check(zero.actuator_angle.norm() < 1e-8,
              name + ": zero pose selects the calibrated zero branch");
        check(solver.closureLengthError(Vec2::Zero(), zero.actuator_angle).norm()
                  < 1e-9,
              name + ": zero pose closes both branches");
    }

    const InverseResult ik = solver.inverse(target, Vec2::Zero());
    check(ik.ok(), name + ": nonzero IK succeeds");
    if (!ik.ok()) {
        return;
    }
    check(solver.closureLengthError(target, ik.actuator_angle).norm() < 1e-9,
          name + ": IK satisfies both rod-length constraints");

    const ForwardResult fk = solver.forward(ik.actuator_angle, target * 0.8);
    check(fk.ok(), name + ": warm-started FK converges");
    if (fk.ok()) {
        check((fk.joint_angle - target).norm() < 1e-8,
              name + ": IK/FK round trip returns the target pose");
        check(solver.closureLengthError(fk.joint_angle, ik.actuator_angle).norm()
                  < 1e-8,
              name + ": reconstructed pose closes both branches");
    }
}

void checkJacobianFiniteDifference(
    const rss1u::Geometry& geometry,
    const rss1u::Vec2& q,
    const std::string& name) {
    using namespace rss1u;

    Solver solver(geometry);
    const InverseResult ik = solver.inverse(q, Vec2::Zero());
    check(ik.ok(), name + ": Jacobian test IK succeeds");
    if (!ik.ok()) {
        return;
    }
    const JacobianResult analytic = solver.jacobian(q, ik.actuator_angle);
    check(analytic.ok(), name + ": analytic Jacobian is nonsingular");
    if (!analytic.ok()) {
        return;
    }

    constexpr double epsilon = 1e-7;
    Mat2 numerical;
    for (int column = 0; column < 2; ++column) {
        Vec2 plus = q;
        Vec2 minus = q;
        plus(column) += epsilon;
        minus(column) -= epsilon;
        const InverseResult plus_ik = solver.inverse(plus, ik.actuator_angle);
        const InverseResult minus_ik = solver.inverse(minus, ik.actuator_angle);
        check(plus_ik.ok() && minus_ik.ok(),
              name + ": central-difference IK succeeds");
        if (!plus_ik.ok() || !minus_ik.ok()) {
            return;
        }
        numerical.col(column) =
            (plus_ik.actuator_angle - minus_ik.actuator_angle) / (2.0 * epsilon);
    }
    check(analytic.matrix.isApprox(numerical, 2e-6),
          name + ": analytic Jacobian matches central finite differences");
}

void testPaperCompatibleGeometry() {
    checkRoundTrip(
        rss1u::examples::currentAnkleExample(),
        rss1u::Vec2(0.08, -0.05),
        "paper-compatible geometry");
}

void testArbitraryMotorAxes() {
    checkRoundTrip(
        rss1u::examples::arbitraryAxisExample(),
        rss1u::Vec2(0.06, -0.04),
        "synthetic arbitrary-axis geometry");
    checkRoundTrip(
        rss1u::tests::fixedXAxisGeometry(),
        rss1u::Vec2(0.06, -0.04),
        "configuration I fixed-x geometry");
    checkRoundTrip(
        rss1u::tests::spatialAxisGeometry(),
        rss1u::Vec2(0.06, -0.04),
        "saved arbitrary spatial-axis geometry");
}

void testAnalyticJacobian() {
    checkJacobianFiniteDifference(
        rss1u::examples::arbitraryAxisExample(),
        rss1u::Vec2(0.055, -0.035),
        "synthetic arbitrary-axis geometry");
    checkJacobianFiniteDifference(
        rss1u::tests::fixedXAxisGeometry(),
        rss1u::Vec2(0.05, -0.03),
        "configuration I fixed-x geometry");
    checkJacobianFiniteDifference(
        rss1u::tests::spatialAxisGeometry(),
        rss1u::Vec2(0.05, -0.03),
        "saved arbitrary spatial-axis geometry");
}

void testVirtualWorkAndDualRoots() {
    using namespace rss1u;
    Solver solver(examples::currentAnkleExample());
    const Vec2 q(0.07, -0.045);
    const InverseResult ik = solver.inverse(q, Vec2::Zero());
    const JacobianResult jacobian = solver.jacobian(q, ik.actuator_angle);
    check(ik.ok() && jacobian.ok(), "state for virtual-work test is valid");

    const Vec2 q_dot(0.24, -0.17);
    const Vec2 tau_q(12.0, -7.0);
    const Vec2 theta_dot = solver.actuatorVelocity(jacobian.matrix, q_dot);
    Vec2 recovered_q_dot;
    check(solver.jointVelocity(jacobian.matrix, theta_dot, recovered_q_dot),
          "actuator-to-joint velocity solve succeeds");
    check(recovered_q_dot.isApprox(q_dot, 1e-12),
          "velocity mappings are mutually consistent");

    Vec2 tau_theta;
    check(solver.actuatorTorque(jacobian.matrix, tau_q, tau_theta),
          "joint-to-actuator torque solve succeeds");
    check(solver.jointTorque(jacobian.matrix, tau_theta).isApprox(tau_q, 1e-12),
          "torque mappings are mutually consistent");
    checkNear(tau_q.dot(q_dot), tau_theta.dot(theta_dot), 1e-10,
              "Jacobian torque mapping preserves virtual power");

    for (std::size_t branch = 0; branch < 2; ++branch) {
        const BranchRoots roots = solver.branchRoots(branch, q, 0.0);
        check(roots.status == Status::Ok, "both analytic roots are available");
        for (double root : roots.angle) {
            Vec2 theta = ik.actuator_angle;
            theta(static_cast<Eigen::Index>(branch)) = root;
            check(std::abs(solver.closureLengthError(q, theta)(
                      static_cast<Eigen::Index>(branch))) < 1e-9,
                  "each analytic root satisfies its branch closure");
        }

        for (int root_index = 0; root_index < 2; ++root_index) {
            if (!roots.within_limits[static_cast<std::size_t>(root_index)]) {
                continue;
            }
            Vec2 reference = ik.actuator_angle;
            reference(static_cast<Eigen::Index>(branch)) =
                roots.angle[static_cast<std::size_t>(root_index)];
            const InverseResult selected = solver.inverse(q, reference);
            check(selected.ok(), "reference-guided assembly-root selection succeeds");
            if (selected.ok()) {
                check(selected.selected_root[branch] == root_index,
                      "nearest valid assembly root follows the reference state");
            }
        }
    }
}

}  // namespace

int main() {
    testPaperCompatibleGeometry();
    testArbitraryMotorAxes();
    testAnalyticJacobian();
    testVirtualWorkAndDualRoots();

    if (failures == 0) {
        std::cout << "All 2-RSS-1U tests passed.\n";
        return EXIT_SUCCESS;
    }
    std::cerr << failures << " test(s) failed.\n";
    return EXIT_FAILURE;
}

