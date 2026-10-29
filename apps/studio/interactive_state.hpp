#pragma once

#include "rss1u/solver.hpp"

#include <array>
#include <string>

namespace rss1u::studio {

enum class ControlledVariable {
    Pitch,
    Roll,
    Theta1,
    Theta2
};

enum class SolveDirection {
    Initial,
    Inverse,
    Forward
};

struct AngleRange {
    double lower{0.0};
    double upper{0.0};

    [[nodiscard]] bool contains(double value) const noexcept;
};

struct InteractiveLimits {
    AngleRange pitch{};
    AngleRange roll{};
    std::array<AngleRange, 2> actuator{};
};

struct InteractiveSnapshot {
    Vec2 joint_angle{Vec2::Zero()};
    Vec2 actuator_angle{Vec2::Zero()};
    Vec2 closure_error{Vec2::Zero()};
    JacobianResult jacobian{};
    SolveDirection direction{SolveDirection::Initial};
    int forward_iterations{0};
};

struct UpdateOutcome {
    bool accepted{false};
    Status status{Status::InvalidGeometry};
    std::string reason{};
    InteractiveSnapshot snapshot{};
};

// Qt 适配器和单元测试共用的应用层状态机。
// 本类刻意不依赖 Qt：rss1u::Solver 是运动学结果的唯一来源；本类只负责
// 四个可编辑角度的装配分支连续性、成功提交和失败回滚。
class InteractiveState {
public:
    explicit InteractiveState(Geometry geometry);

    [[nodiscard]] bool valid() const noexcept { return valid_; }
    [[nodiscard]] const std::string& initializationError() const noexcept {
        return initialization_error_;
    }
    [[nodiscard]] const Geometry& geometry() const noexcept { return geometry_; }
    [[nodiscard]] const InteractiveSnapshot& snapshot() const noexcept {
        return snapshot_;
    }
    [[nodiscard]] const InteractiveLimits& limits() const noexcept { return limits_; }

    // 输入和内部状态均使用弧度。请求被拒绝时，已提交快照保持不变。
    [[nodiscard]] UpdateOutcome update(
        ControlledVariable variable,
        double requested_value) noexcept;

private:
    [[nodiscard]] UpdateOutcome reject(Status status, std::string reason) const;
    [[nodiscard]] InteractiveSnapshot makeSnapshot(
        const Vec2& joint_angle,
        const Vec2& actuator_angle,
        SolveDirection direction,
        int forward_iterations) const noexcept;
    [[nodiscard]] AngleRange discoverJointRange(std::size_t joint_index) const noexcept;
    [[nodiscard]] double scanJointBound(
        std::size_t joint_index,
        double direction) const noexcept;
    [[nodiscard]] static std::string statusReason(Status status);

    Geometry geometry_{};
    Solver solver_;
    InteractiveSnapshot snapshot_{};
    InteractiveLimits limits_{};
    bool valid_{false};
    std::string initialization_error_{};
};

}  // 命名空间 rss1u::studio
