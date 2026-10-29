#pragma once

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <limits>

namespace rss1u {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Mat2 = Eigen::Matrix2d;
using Mat3 = Eigen::Matrix3d;

enum class Status {
    Ok,
    InvalidGeometry,
    NonFiniteInput,
    Unreachable,
    CoefficientSingular,
    JointLimit,
    JacobianSingular,
    IterationLimit
};

[[nodiscard]] const char* statusMessage(Status status) noexcept;

struct AngleLimit {
    double lower{-std::numeric_limits<double>::infinity()};
    double upper{std::numeric_limits<double>::infinity()};

    [[nodiscard]] bool contains(double value, double tolerance = 1e-12) const noexcept;
};

// One R-S-S branch. Vectors within a branch share one calculation frame.
// Branch frames may differ by a constant translation, but their axes remain
// parallel so that the same platform rotation matrix applies to both.
struct BranchGeometry {
    Vec3 axis_point{Vec3::Zero()};       // Any point on the motor revolute axis.
    Vec3 axis{Vec3::UnitY()};            // Positive motor-axis direction.
    Vec3 crank_zero{Vec3::UnitX()};      // Axis point -> crank ball at theta = 0.
    Vec3 platform_offset{Vec3::Zero()};  // Fixed coordinates of U-joint center O (legacy AP).
    Vec3 platform_point{Vec3::Zero()};   // O -> platform ball B at zero pose (legacy PB).
    double rod_length{1.0};              // Distance between the two S joints.
    double motor_zero_offset{0.0};       // Physical angle = encoder + offset.
    AngleLimit motor_limit{};
};

struct Geometry {
    std::array<BranchGeometry, 2> branches{};
};

struct SolverOptions {
    double coefficient_tolerance{1e-10};
    double jacobian_tolerance{1e-10};
    double residual_tolerance{1e-9};
    double maximum_step{0.20};
    int maximum_iterations{12};
};

struct BranchRoots {
    Status status{Status::InvalidGeometry};
    std::array<double, 2> angle{{0.0, 0.0}};
    std::array<bool, 2> within_limits{{false, false}};
    double K{0.0};
    double M{0.0};
    double N{0.0};
    double amplitude{0.0};
};

struct InverseResult {
    Status status{Status::InvalidGeometry};
    Vec2 actuator_angle{Vec2::Zero()};
    std::array<int, 2> selected_root{{-1, -1}};
    std::array<BranchRoots, 2> branches{};

    [[nodiscard]] bool ok() const noexcept { return status == Status::Ok; }
};

struct JacobianResult {
    Status status{Status::InvalidGeometry};
    Mat2 matrix{Mat2::Zero()};           // theta_dot = J * q_dot.
    Vec2 branch_denominator{Vec2::Zero()};
    double determinant{0.0};
    double condition_frobenius{std::numeric_limits<double>::infinity()};

    [[nodiscard]] bool ok() const noexcept { return status == Status::Ok; }
};

struct ForwardResult {
    Status status{Status::InvalidGeometry};
    Vec2 joint_angle{Vec2::Zero()};      // [pitch, roll].
    Vec2 actuator_residual{Vec2::Zero()};
    JacobianResult jacobian{};
    int iterations{0};

    [[nodiscard]] bool ok() const noexcept { return status == Status::Ok; }
};

class Solver {
public:
    explicit Solver(
        Geometry geometry,
        SolverOptions options = SolverOptions()) noexcept;

    [[nodiscard]] bool geometryValid() const noexcept { return geometry_valid_; }
    [[nodiscard]] const Geometry& geometry() const noexcept { return geometry_; }
    [[nodiscard]] const SolverOptions& options() const noexcept { return options_; }

    // Platform orientation: R = Ry(pitch) * Rx(roll).
    [[nodiscard]] static Mat3 platformRotation(double pitch, double roll) noexcept;

    // Rodrigues rotation for an arbitrary motor-axis direction.
    [[nodiscard]] static Mat3 axisRotation(const Vec3& unit_axis, double angle) noexcept;

    [[nodiscard]] BranchRoots branchRoots(
        std::size_t branch_index,
        const Vec2& joint_angle,
        double reference_angle) const noexcept;

    // Analytic IK. The reference selects the closest assembly branch.
    [[nodiscard]] InverseResult inverse(
        const Vec2& joint_angle,
        const Vec2& actuator_reference) const noexcept;

    // Analytic differential relation theta_dot = J * [pitch_dot, roll_dot].
    [[nodiscard]] JacobianResult jacobian(
        const Vec2& joint_angle,
        const Vec2& actuator_angle) const noexcept;

    // Paper-compatible Newton FK using IK(q) - theta_measured as the residual.
    // For real-time use, pass the previous successful joint angle as initial_guess.
    [[nodiscard]] ForwardResult forward(
        const Vec2& actuator_angle,
        const Vec2& initial_guess) const noexcept;

    [[nodiscard]] Vec2 closureLengthError(
        const Vec2& joint_angle,
        const Vec2& actuator_angle) const noexcept;

    [[nodiscard]] bool jointVelocity(
        const Mat2& jacobian,
        const Vec2& actuator_velocity,
        Vec2& joint_velocity) const noexcept;

    [[nodiscard]] Vec2 actuatorVelocity(
        const Mat2& jacobian,
        const Vec2& joint_velocity) const noexcept;

    // Virtual work: tau_q = J^T * tau_theta.
    [[nodiscard]] Vec2 jointTorque(
        const Mat2& jacobian,
        const Vec2& actuator_torque) const noexcept;

    [[nodiscard]] bool actuatorTorque(
        const Mat2& jacobian,
        const Vec2& joint_torque,
        Vec2& actuator_torque) const noexcept;

private:
    [[nodiscard]] Vec3 movingPoint(
        const BranchGeometry& branch,
        const Vec2& joint_angle) const noexcept;
    [[nodiscard]] Vec3 crankPoint(
        const BranchGeometry& branch,
        double actuator_angle) const noexcept;
    [[nodiscard]] bool solve2x2(
        const Mat2& matrix,
        const Vec2& rhs,
        Vec2& solution) const noexcept;

    Geometry geometry_{};
    SolverOptions options_{};
    bool geometry_valid_{false};
};

}  // namespace rss1u

