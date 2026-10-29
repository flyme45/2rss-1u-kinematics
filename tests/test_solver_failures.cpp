#include "example_geometry.hpp"
#include "rss1u/solver.hpp"
#include "test_geometries.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
    if (!condition) {
        ++failures;
        std::cerr << "[FAIL] " << message << '\n';
    }
}

void testInvalidGeometryAndOptions() {
    using namespace rss1u;

    Geometry zero_axis = examples::currentAnkleExample();
    zero_axis.branches[0].axis.setZero();
    Solver zero_axis_solver(zero_axis);
    check(!zero_axis_solver.geometryValid(), "zero motor axis rejects geometry");
    check(zero_axis_solver.inverse(Vec2::Zero(), Vec2::Zero()).status
              == Status::InvalidGeometry,
          "invalid geometry propagates through IK");
    check(zero_axis_solver.jacobian(Vec2::Zero(), Vec2::Zero()).status
              == Status::InvalidGeometry,
          "invalid geometry propagates through Jacobian");
    check(zero_axis_solver.forward(Vec2::Zero(), Vec2::Zero()).status
              == Status::InvalidGeometry,
          "invalid geometry propagates through FK");
    check(!zero_axis_solver.closureLengthError(Vec2::Zero(), Vec2::Zero()).allFinite(),
          "invalid geometry closure result is explicitly non-finite");

    Geometry negative_length = examples::currentAnkleExample();
    negative_length.branches[0].rod_length = -1.0;
    check(!Solver(negative_length).geometryValid(), "negative rod length is rejected");

    Geometry reversed_limit = examples::currentAnkleExample();
    reversed_limit.branches[0].motor_limit = AngleLimit{1.0, -1.0};
    check(!Solver(reversed_limit).geometryValid(), "reversed motor limits are rejected");

    Geometry axial_crank = examples::currentAnkleExample();
    axial_crank.branches[0].crank_zero = axial_crank.branches[0].axis;
    check(!Solver(axial_crank).geometryValid(),
          "crank vector parallel to motor axis is rejected");

    SolverOptions invalid_options;
    invalid_options.maximum_iterations = 0;
    check(!Solver(examples::currentAnkleExample(), invalid_options).geometryValid(),
          "non-positive iteration limit is rejected");
}

void testNonFiniteInput() {
    using namespace rss1u;
    Solver solver(examples::currentAnkleExample());
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();

    check(solver.inverse(Vec2(nan, 0.0), Vec2::Zero()).status
              == Status::NonFiniteInput,
          "IK rejects NaN joint input");
    check(solver.inverse(Vec2::Zero(), Vec2(infinity, 0.0)).status
              == Status::NonFiniteInput,
          "IK rejects infinite reference input");
    check(solver.branchRoots(0, Vec2::Zero(), nan).status
              == Status::NonFiniteInput,
          "branch solver rejects NaN reference");
    check(solver.jacobian(Vec2::Zero(), Vec2(0.0, infinity)).status
              == Status::NonFiniteInput,
          "Jacobian rejects infinite actuator input");
    check(solver.forward(Vec2(nan, 0.0), Vec2::Zero()).status
              == Status::NonFiniteInput,
          "FK rejects NaN actuator input");
    check(!solver.closureLengthError(Vec2::Zero(), Vec2(nan, 0.0)).allFinite(),
          "closure error exposes invalid non-finite input");
}

void testInverseFailureStatuses() {
    using namespace rss1u;

    Geometry impossible = examples::currentAnkleExample();
    impossible.branches[0].rod_length = 1.0;
    check(Solver(impossible).inverse(Vec2::Zero(), Vec2::Zero()).status
              == Status::Unreachable,
          "unreachable pose returns Unreachable");

    Geometry limited = examples::currentAnkleExample();
    limited.branches[0].motor_limit = AngleLimit{4.0, 5.0};
    check(Solver(limited).inverse(Vec2::Zero(), Vec2::Zero()).status
              == Status::JointLimit,
          "two out-of-limit roots return JointLimit");

    Geometry coefficient_singular = examples::currentAnkleExample();
    BranchGeometry& branch = coefficient_singular.branches[0];
    branch.axis_point = Vec3::Zero();
    branch.axis = Vec3::UnitZ();
    branch.crank_zero = Vec3::UnitX();
    branch.platform_offset = Vec3::Zero();
    branch.platform_point = 2.0 * Vec3::UnitZ();
    branch.rod_length = 1.0;
    Solver coefficient_solver(coefficient_singular);
    check(coefficient_solver.geometryValid(),
          "coefficient-singular fixture remains valid geometry");
    check(coefficient_solver.branchRoots(0, Vec2::Zero(), 0.0).status
              == Status::CoefficientSingular,
          "vanishing K/M amplitude returns CoefficientSingular");

    check(coefficient_solver.branchRoots(2, Vec2::Zero(), 0.0).status
              == Status::InvalidGeometry,
          "out-of-range branch index is rejected");
}

void testJacobianAndLinearSolveFailures() {
    using namespace rss1u;

    Geometry duplicate = examples::currentAnkleExample();
    duplicate.branches[1] = duplicate.branches[0];
    Solver duplicate_solver(duplicate);
    const Vec2 q(0.08, -0.05);
    const InverseResult ik = duplicate_solver.inverse(q, Vec2::Zero());
    check(ik.ok(), "duplicate-branch fixture IK succeeds");
    if (ik.ok()) {
        check(duplicate_solver.jacobian(q, ik.actuator_angle).status
                  == Status::JacobianSingular,
              "rank-deficient 2x2 Jacobian returns JacobianSingular");
    }

    Solver solver(examples::currentAnkleExample());
    Mat2 singular;
    singular << 1.0, 2.0, 2.0, 4.0;
    Vec2 output;
    check(!solver.jointVelocity(singular, Vec2::Ones(), output),
          "singular velocity solve returns false");
    check(!solver.actuatorTorque(singular, Vec2::Ones(), output),
          "singular torque solve returns false");
}

void testForwardIterationLimit() {
    using namespace rss1u;
    SolverOptions options;
    options.maximum_iterations = 1;
    options.residual_tolerance = 1e-14;
    Solver solver(tests::spatialAxisGeometry(), options);
    const Vec2 target(0.08, -0.05);
    const InverseResult ik = solver.inverse(target, Vec2::Zero());
    check(ik.ok(), "iteration-limit fixture IK succeeds");
    if (!ik.ok()) {
        return;
    }
    const ForwardResult fk = solver.forward(ik.actuator_angle, Vec2(-0.08, 0.06));
    check(fk.status == Status::IterationLimit,
          "insufficient Newton iterations return IterationLimit");
    check(fk.iterations == 1, "iteration count reports the configured limit");
}

void testStatusMessages() {
    using namespace rss1u;
    const Status statuses[] = {
        Status::Ok,
        Status::InvalidGeometry,
        Status::NonFiniteInput,
        Status::Unreachable,
        Status::CoefficientSingular,
        Status::JointLimit,
        Status::JacobianSingular,
        Status::IterationLimit,
    };
    for (Status status : statuses) {
        check(statusMessage(status) != nullptr && statusMessage(status)[0] != '\0',
              "every public status has a diagnostic message");
    }
}

}  // namespace

int main() {
    testInvalidGeometryAndOptions();
    testNonFiniteInput();
    testInverseFailureStatuses();
    testJacobianAndLinearSolveFailures();
    testForwardIterationLimit();
    testStatusMessages();

    if (failures == 0) {
        std::cout << "All 2-RSS-1U failure and degeneracy tests passed.\n";
        return EXIT_SUCCESS;
    }
    std::cerr << failures << " failure/degeneracy test(s) failed.\n";
    return EXIT_FAILURE;
}
