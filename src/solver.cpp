#include "rss1u/solver.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace rss1u {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

bool finiteVector(const Vec2& value) noexcept {
    return value.allFinite();
}

bool finiteVector(const Vec3& value) noexcept {
    return value.allFinite();
}

double nearAngle(double angle, double reference) noexcept {
    return reference + std::remainder(angle - reference, 2.0 * kPi);
}

bool validOptions(const SolverOptions& options) noexcept {
    return std::isfinite(options.coefficient_tolerance)
        && std::isfinite(options.jacobian_tolerance)
        && std::isfinite(options.residual_tolerance)
        && std::isfinite(options.maximum_step)
        && options.coefficient_tolerance > 0.0
        && options.jacobian_tolerance > 0.0
        && options.residual_tolerance > 0.0
        && options.maximum_step > 0.0
        && options.maximum_iterations > 0;
}

bool validBranch(const BranchGeometry& branch) noexcept {
    if (!finiteVector(branch.axis_point) || !finiteVector(branch.axis)
        || !finiteVector(branch.crank_zero) || !finiteVector(branch.platform_offset)
        || !finiteVector(branch.platform_point) || !std::isfinite(branch.rod_length)
        || !std::isfinite(branch.motor_zero_offset)
        || branch.axis.norm() < 1e-12 || branch.rod_length <= 0.0
        || branch.motor_limit.lower > branch.motor_limit.upper) {
        return false;
    }
    const Vec3 unit_axis = branch.axis.normalized();
    const Vec3 transverse = branch.crank_zero
        - unit_axis * unit_axis.dot(branch.crank_zero);
    return transverse.norm() > 1e-12;
}

}  // namespace

const char* statusMessage(Status status) noexcept {
    switch (status) {
        case Status::Ok: return "ok";
        case Status::InvalidGeometry: return "invalid geometry or solver options";
        case Status::NonFiniteInput: return "non-finite input";
        case Status::Unreachable: return "target pose is unreachable";
        case Status::CoefficientSingular: return "inverse equation is indeterminate";
        case Status::JointLimit: return "both inverse roots violate motor limits";
        case Status::JacobianSingular: return "Jacobian is singular";
        case Status::IterationLimit: return "forward kinematics iteration limit";
    }
    return "unknown status";
}

bool AngleLimit::contains(double value, double tolerance) const noexcept {
    return value >= lower - tolerance && value <= upper + tolerance;
}

Solver::Solver(Geometry geometry, SolverOptions options) noexcept
    : geometry_(std::move(geometry)), options_(options) {
    geometry_valid_ = validOptions(options_);
    for (BranchGeometry& branch : geometry_.branches) {
        geometry_valid_ = geometry_valid_ && validBranch(branch);
        if (branch.axis.norm() > 1e-12) {
            branch.axis.normalize();
        }
    }
}

Mat3 Solver::platformRotation(double pitch, double roll) noexcept {
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    Mat3 rotation;
    rotation << cp, sp * sr, sp * cr,
                0.0, cr, -sr,
               -sp, cp * sr, cp * cr;
    return rotation;
}

Mat3 Solver::axisRotation(const Vec3& unit_axis, double angle) noexcept {
    Mat3 cross;
    cross << 0.0, -unit_axis.z(), unit_axis.y(),
             unit_axis.z(), 0.0, -unit_axis.x(),
            -unit_axis.y(), unit_axis.x(), 0.0;
    return Mat3::Identity() + std::sin(angle) * cross
        + (1.0 - std::cos(angle)) * (cross * cross);
}

Vec3 Solver::movingPoint(
    const BranchGeometry& branch,
    const Vec2& joint_angle) const noexcept {
    return branch.platform_offset
        + platformRotation(joint_angle.x(), joint_angle.y()) * branch.platform_point;
}

Vec3 Solver::crankPoint(
    const BranchGeometry& branch,
    double actuator_angle) const noexcept {
    return branch.axis_point
        + axisRotation(branch.axis, actuator_angle + branch.motor_zero_offset)
            * branch.crank_zero;
}

BranchRoots Solver::branchRoots(
    std::size_t branch_index,
    const Vec2& joint_angle,
    double reference_angle) const noexcept {
    BranchRoots result;
    if (!geometry_valid_ || branch_index >= geometry_.branches.size()) {
        result.status = Status::InvalidGeometry;
        return result;
    }
    if (!finiteVector(joint_angle) || !std::isfinite(reference_angle)) {
        result.status = Status::NonFiniteInput;
        return result;
    }

    const BranchGeometry& branch = geometry_.branches[branch_index];
    const Vec3 displacement = movingPoint(branch, joint_angle) - branch.axis_point;
    const Vec3 parallel = branch.axis * branch.axis.dot(branch.crank_zero);
    const Vec3 perpendicular = branch.crank_zero - parallel;
    const Vec3 sine_vector = branch.axis.cross(branch.crank_zero);

    result.K = -2.0 * displacement.dot(perpendicular);
    result.M = -2.0 * displacement.dot(sine_vector);
    result.N = displacement.squaredNorm() + branch.crank_zero.squaredNorm()
        - branch.rod_length * branch.rod_length
        - 2.0 * displacement.dot(parallel);
    result.amplitude = std::hypot(result.K, result.M);

    if (!std::isfinite(result.amplitude) || !std::isfinite(result.N)) {
        result.status = Status::NonFiniteInput;
        return result;
    }
    if (result.amplitude < options_.coefficient_tolerance) {
        result.status = Status::CoefficientSingular;
        return result;
    }

    const double normalized = -result.N / result.amplitude;
    if (normalized < -1.0 - options_.coefficient_tolerance
        || normalized > 1.0 + options_.coefficient_tolerance) {
        result.status = Status::Unreachable;
        return result;
    }

    const double phase = std::atan2(result.M, result.K);
    const double opening = std::acos(std::clamp(normalized, -1.0, 1.0));
    const double first = phase + opening - branch.motor_zero_offset;
    const double second = phase - opening - branch.motor_zero_offset;
    result.angle[0] = nearAngle(first, reference_angle);
    result.angle[1] = nearAngle(second, reference_angle);
    result.within_limits[0] = branch.motor_limit.contains(
        result.angle[0], options_.coefficient_tolerance);
    result.within_limits[1] = branch.motor_limit.contains(
        result.angle[1], options_.coefficient_tolerance);
    result.status = (!result.within_limits[0] && !result.within_limits[1])
        ? Status::JointLimit : Status::Ok;
    return result;
}

InverseResult Solver::inverse(
    const Vec2& joint_angle,
    const Vec2& actuator_reference) const noexcept {
    InverseResult result;
    if (!geometry_valid_) {
        result.status = Status::InvalidGeometry;
        return result;
    }
    if (!finiteVector(joint_angle) || !finiteVector(actuator_reference)) {
        result.status = Status::NonFiniteInput;
        return result;
    }

    for (std::size_t i = 0; i < geometry_.branches.size(); ++i) {
        result.branches[i] = branchRoots(i, joint_angle, actuator_reference(i));
        if (result.branches[i].status != Status::Ok) {
            result.status = result.branches[i].status;
            return result;
        }
        const BranchRoots& roots = result.branches[i];
        const double d0 = roots.within_limits[0]
            ? std::abs(roots.angle[0] - actuator_reference(i))
            : std::numeric_limits<double>::infinity();
        const double d1 = roots.within_limits[1]
            ? std::abs(roots.angle[1] - actuator_reference(i))
            : std::numeric_limits<double>::infinity();
        const int selected = d0 <= d1 ? 0 : 1;
        result.selected_root[i] = selected;
        result.actuator_angle(i) = roots.angle[selected];
    }
    result.status = Status::Ok;
    return result;
}

JacobianResult Solver::jacobian(
    const Vec2& joint_angle,
    const Vec2& actuator_angle) const noexcept {
    JacobianResult result;
    if (!geometry_valid_) {
        result.status = Status::InvalidGeometry;
        return result;
    }
    if (!finiteVector(joint_angle) || !finiteVector(actuator_angle)) {
        result.status = Status::NonFiniteInput;
        return result;
    }

    const double pitch = joint_angle.x();
    const Mat3 rotation = platformRotation(pitch, joint_angle.y());
    const Vec3 pitch_axis = Vec3::UnitY();
    const Vec3 roll_axis(std::cos(pitch), 0.0, -std::sin(pitch));

    for (std::size_t i = 0; i < geometry_.branches.size(); ++i) {
        const BranchGeometry& branch = geometry_.branches[i];
        const Vec3 rotated_platform_point = rotation * branch.platform_point;
        const Vec3 moving = branch.platform_offset + rotated_platform_point;
        const Vec3 crank = crankPoint(branch, actuator_angle(i));
        const Vec3 rod = moving - crank;
        const Vec3 crank_velocity = branch.axis.cross(crank - branch.axis_point);
        const double denominator = rod.dot(crank_velocity);
        result.branch_denominator(i) = denominator;

        const double scale = std::max(1.0, rod.norm() * crank_velocity.norm());
        if (!std::isfinite(denominator)
            || std::abs(denominator) < options_.jacobian_tolerance * scale) {
            result.status = Status::JacobianSingular;
            return result;
        }
        result.matrix(i, 0) = rod.dot(pitch_axis.cross(rotated_platform_point))
            / denominator;
        result.matrix(i, 1) = rod.dot(roll_axis.cross(rotated_platform_point))
            / denominator;
    }

    result.determinant = result.matrix.determinant();
    if (!result.matrix.allFinite()
        || std::abs(result.determinant) < options_.jacobian_tolerance) {
        result.status = Status::JacobianSingular;
        return result;
    }
    Mat2 inverse;
    Vec2 first_column;
    Vec2 second_column;
    if (!solve2x2(result.matrix, Vec2::UnitX(), first_column)
        || !solve2x2(result.matrix, Vec2::UnitY(), second_column)) {
        result.status = Status::JacobianSingular;
        return result;
    }
    inverse.col(0) = first_column;
    inverse.col(1) = second_column;
    result.condition_frobenius = result.matrix.norm() * inverse.norm();
    result.status = Status::Ok;
    return result;
}

ForwardResult Solver::forward(
    const Vec2& actuator_angle,
    const Vec2& initial_guess) const noexcept {
    ForwardResult result;
    result.joint_angle = initial_guess;
    if (!geometry_valid_) {
        result.status = Status::InvalidGeometry;
        return result;
    }
    if (!finiteVector(actuator_angle) || !finiteVector(initial_guess)) {
        result.status = Status::NonFiniteInput;
        return result;
    }

    for (int iteration = 0; iteration < options_.maximum_iterations; ++iteration) {
        result.iterations = iteration + 1;
        const InverseResult predicted = inverse(result.joint_angle, actuator_angle);
        if (!predicted.ok()) {
            result.status = predicted.status;
            return result;
        }
        result.actuator_residual = predicted.actuator_angle - actuator_angle;
        result.jacobian = jacobian(result.joint_angle, predicted.actuator_angle);
        if (!result.jacobian.ok()) {
            result.status = result.jacobian.status;
            return result;
        }
        if (result.actuator_residual.cwiseAbs().maxCoeff()
            < options_.residual_tolerance) {
            result.status = Status::Ok;
            return result;
        }

        Vec2 step;
        if (!solve2x2(result.jacobian.matrix, result.actuator_residual, step)) {
            result.status = Status::JacobianSingular;
            return result;
        }
        const double maximum_component = step.cwiseAbs().maxCoeff();
        const double scale = maximum_component > options_.maximum_step
            ? options_.maximum_step / maximum_component : 1.0;
        result.joint_angle -= scale * step;
    }

    result.status = Status::IterationLimit;
    return result;
}

Vec2 Solver::closureLengthError(
    const Vec2& joint_angle,
    const Vec2& actuator_angle) const noexcept {
    Vec2 error = Vec2::Constant(std::numeric_limits<double>::quiet_NaN());
    if (!geometry_valid_ || !finiteVector(joint_angle) || !finiteVector(actuator_angle)) {
        return error;
    }
    for (std::size_t i = 0; i < geometry_.branches.size(); ++i) {
        const BranchGeometry& branch = geometry_.branches[i];
        error(i) = (movingPoint(branch, joint_angle)
            - crankPoint(branch, actuator_angle(i))).norm() - branch.rod_length;
    }
    return error;
}

bool Solver::solve2x2(
    const Mat2& matrix,
    const Vec2& rhs,
    Vec2& solution) const noexcept {
    const double determinant = matrix(0, 0) * matrix(1, 1)
        - matrix(0, 1) * matrix(1, 0);
    if (!std::isfinite(determinant)
        || std::abs(determinant) < options_.jacobian_tolerance) {
        return false;
    }
    solution.x() = (matrix(1, 1) * rhs.x() - matrix(0, 1) * rhs.y()) / determinant;
    solution.y() = (-matrix(1, 0) * rhs.x() + matrix(0, 0) * rhs.y()) / determinant;
    return solution.allFinite();
}

bool Solver::jointVelocity(
    const Mat2& jacobian_matrix,
    const Vec2& actuator_velocity,
    Vec2& joint_velocity) const noexcept {
    return solve2x2(jacobian_matrix, actuator_velocity, joint_velocity);
}

Vec2 Solver::actuatorVelocity(
    const Mat2& jacobian_matrix,
    const Vec2& joint_velocity) const noexcept {
    return jacobian_matrix * joint_velocity;
}

Vec2 Solver::jointTorque(
    const Mat2& jacobian_matrix,
    const Vec2& actuator_torque) const noexcept {
    return jacobian_matrix.transpose() * actuator_torque;
}

bool Solver::actuatorTorque(
    const Mat2& jacobian_matrix,
    const Vec2& joint_torque,
    Vec2& actuator_torque) const noexcept {
    return solve2x2(jacobian_matrix.transpose(), joint_torque, actuator_torque);
}

}  // namespace rss1u

