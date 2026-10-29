#include "interactive_state.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace rss1u::studio {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kRangeStep = kPi / 180.0;  // 工作区扫描步长为一度。
constexpr int kRefinementIterations = 24;

bool finite(double value) noexcept {
    return std::isfinite(value);
}

AngleRange finiteMotorRange(const AngleLimit& limit) noexcept {
    // 无限位转动坐标具有周期性；界面取零位附近一整圈作为有限显示范围。
    // 只要存在已标定的有限机械限位，就始终优先采用机械限位。
    return AngleRange{
        finite(limit.lower) ? limit.lower : -kPi,
        finite(limit.upper) ? limit.upper : kPi};
}

}  // 匿名命名空间

bool AngleRange::contains(double value) const noexcept {
    return finite(value) && value >= lower - 1e-12 && value <= upper + 1e-12;
}

InteractiveState::InteractiveState(Geometry geometry)
    : geometry_(std::move(geometry)), solver_(geometry_) {
    if (!solver_.geometryValid()) {
        initialization_error_ = "机构几何参数无效";
        return;
    }

    const Vec2 zero = Vec2::Zero();
    const InverseResult inverse = solver_.inverse(zero, zero);
    if (!inverse.ok()) {
        initialization_error_ = "零位初始化失败：" + statusReason(inverse.status);
        return;
    }

    snapshot_ = makeSnapshot(zero, inverse.actuator_angle, SolveDirection::Initial, 0);
    limits_.pitch = discoverJointRange(0);
    limits_.roll = discoverJointRange(1);
    for (std::size_t i = 0; i < geometry_.branches.size(); ++i) {
        limits_.actuator[i] = finiteMotorRange(geometry_.branches[i].motor_limit);
    }
    valid_ = true;
}

UpdateOutcome InteractiveState::update(
    ControlledVariable variable,
    double requested_value) noexcept {
    if (!valid_) {
        return reject(Status::InvalidGeometry, initialization_error_);
    }
    if (!finite(requested_value)) {
        return reject(Status::NonFiniteInput, "输入必须是有限数值");
    }

    if (variable == ControlledVariable::Pitch || variable == ControlledVariable::Roll) {
        const std::size_t index = variable == ControlledVariable::Pitch ? 0U : 1U;
        const AngleRange& range = index == 0 ? limits_.pitch : limits_.roll;
        if (!range.contains(requested_value)) {
            return reject(Status::JointLimit, "目标平台角超出当前构型的连续有效工作区间");
        }

        Vec2 candidate_joint = snapshot_.joint_angle;
        candidate_joint(static_cast<Eigen::Index>(index)) = requested_value;
        const InverseResult inverse = solver_.inverse(
            candidate_joint, snapshot_.actuator_angle);
        if (!inverse.ok()) {
            return reject(inverse.status, "IK 失败：" + statusReason(inverse.status));
        }

        const InteractiveSnapshot candidate = makeSnapshot(
            candidate_joint, inverse.actuator_angle, SolveDirection::Inverse, 0);
        if (!candidate.closure_error.allFinite()
            || candidate.closure_error.cwiseAbs().maxCoeff()
                > std::max(1e-7, solver_.options().residual_tolerance * 100.0)) {
            return reject(Status::Unreachable, "IK 闭环长度残差超出容差");
        }
        snapshot_ = candidate;
    } else {
        const std::size_t index = variable == ControlledVariable::Theta1 ? 0U : 1U;
        if (!limits_.actuator[index].contains(requested_value)) {
            return reject(Status::JointLimit, "目标驱动角超出该电机机械限位");
        }

        Vec2 candidate_actuator = snapshot_.actuator_angle;
        candidate_actuator(static_cast<Eigen::Index>(index)) = requested_value;
        const ForwardResult forward = solver_.forward(
            candidate_actuator, snapshot_.joint_angle);
        if (!forward.ok()) {
            return reject(forward.status, "FK 失败：" + statusReason(forward.status));
        }

        const InteractiveSnapshot candidate = makeSnapshot(
            forward.joint_angle,
            candidate_actuator,
            SolveDirection::Forward,
            forward.iterations);
        if (!candidate.closure_error.allFinite()
            || candidate.closure_error.cwiseAbs().maxCoeff()
                > std::max(1e-7, solver_.options().residual_tolerance * 100.0)) {
            return reject(Status::Unreachable, "FK 闭环长度残差超出容差");
        }
        snapshot_ = candidate;
    }

    return UpdateOutcome{true, Status::Ok, "求解成功", snapshot_};
}

UpdateOutcome InteractiveState::reject(Status status, std::string reason) const {
    return UpdateOutcome{false, status, std::move(reason), snapshot_};
}

InteractiveSnapshot InteractiveState::makeSnapshot(
    const Vec2& joint_angle,
    const Vec2& actuator_angle,
    SolveDirection direction,
    int forward_iterations) const noexcept {
    InteractiveSnapshot result;
    result.joint_angle = joint_angle;
    result.actuator_angle = actuator_angle;
    result.closure_error = solver_.closureLengthError(joint_angle, actuator_angle);
    result.jacobian = solver_.jacobian(joint_angle, actuator_angle);
    result.direction = direction;
    result.forward_iterations = forward_iterations;
    return result;
}

AngleRange InteractiveState::discoverJointRange(std::size_t joint_index) const noexcept {
    const double lower = scanJointBound(joint_index, -1.0);
    const double upper = scanJointBound(joint_index, 1.0);
    // 即使零位附近接近退化，也保留极小的可操作区间；实际请求仍须通过
    // 求解器验证后才会提交。
    return AngleRange{std::min(lower, -1e-6), std::max(upper, 1e-6)};
}

double InteractiveState::scanJointBound(
    std::size_t joint_index,
    double direction) const noexcept {
    Vec2 reference = Vec2::Zero();
    double last_valid = 0.0;
    double first_invalid = direction * kPi;
    bool found_invalid = false;

    for (double magnitude = kRangeStep; magnitude <= kPi + 1e-12;
         magnitude += kRangeStep) {
        Vec2 joint = Vec2::Zero();
        joint(static_cast<Eigen::Index>(joint_index)) = direction * magnitude;
        const InverseResult inverse = solver_.inverse(joint, reference);
        if (!inverse.ok()) {
            first_invalid = direction * magnitude;
            found_invalid = true;
            break;
        }
        reference = inverse.actuator_angle;
        last_valid = direction * magnitude;
    }
    if (!found_invalid) {
        return direction * kPi;
    }

    double valid = last_valid;
    double invalid = first_invalid;
    for (int i = 0; i < kRefinementIterations; ++i) {
        const double middle = 0.5 * (valid + invalid);
        Vec2 joint = Vec2::Zero();
        joint(static_cast<Eigen::Index>(joint_index)) = middle;
        const InverseResult inverse = solver_.inverse(joint, reference);
        if (inverse.ok()) {
            valid = middle;
            reference = inverse.actuator_angle;
        } else {
            invalid = middle;
        }
    }
    return valid;
}

std::string InteractiveState::statusReason(Status status) {
    switch (status) {
        case Status::Ok: return "正常";
        case Status::InvalidGeometry: return "机构几何参数或求解器选项无效";
        case Status::NonFiniteInput: return "输入包含非有限数值";
        case Status::Unreachable: return "目标状态不可达";
        case Status::CoefficientSingular: return "IK 三角方程系数退化";
        case Status::JointLimit: return "解析根超出驱动关节限位";
        case Status::JacobianSingular: return "Jacobian 奇异或病态";
        case Status::IterationLimit: return "FK 迭代未在上限内收敛";
    }
    return "未知求解错误";
}

}  // 命名空间 rss1u::studio
