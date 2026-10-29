#include "example_geometry.hpp"
#include "interactive_state.hpp"

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
    if (!condition) {
        ++failures;
        std::cerr << "[失败] " << message << '\n';
    }
}

void checkNear(
    double actual,
    double expected,
    double tolerance,
    const std::string& message) {
    check(
        std::abs(actual - expected) <= tolerance,
        message + " actual=" + std::to_string(actual)
            + " expected=" + std::to_string(expected));
}

void checkClosed(
    const rss1u::studio::InteractiveSnapshot& snapshot,
    const std::string& message) {
    std::ostringstream detail;
    detail << std::setprecision(17)
           << message << " norm=" << snapshot.closure_error.norm()
           << " value=[" << snapshot.closure_error.x()
           << ", " << snapshot.closure_error.y() << ']';
    check(snapshot.closure_error.allFinite()
              && snapshot.closure_error.norm() <= 1e-8,
          detail.str());
}

void testAutomaticInputRouting() {
    using namespace rss1u;
    using namespace rss1u::studio;

    InteractiveState state(examples::arbitraryAxisExample());
    check(state.valid(), "交互状态初始化成功");
    check(state.limits().pitch.lower < 0.0 && state.limits().pitch.upper > 0.0,
          "Pitch 滑块范围由可达工作区自动发现");
    check(state.limits().roll.lower < 0.0 && state.limits().roll.upper > 0.0,
          "Roll 滑块范围由可达工作区自动发现");

    const UpdateOutcome pitch = state.update(ControlledVariable::Pitch, 0.04);
    check(pitch.accepted, "Pitch 输入自动调用 IK");
    check(pitch.snapshot.direction == SolveDirection::Inverse,
          "Pitch 更新报告 IK 求解方向");
    checkNear(pitch.snapshot.joint_angle.x(), 0.04, 1e-12,
              "Pitch 采用请求值");
    checkNear(pitch.snapshot.joint_angle.y(), 0.0, 1e-12,
              "Pitch 更新保持 Roll 不变");
    checkClosed(pitch.snapshot, "Pitch IK 保持两根连杆定长");

    const UpdateOutcome roll = state.update(ControlledVariable::Roll, -0.03);
    check(roll.accepted, "Roll 输入自动调用 IK");
    checkNear(roll.snapshot.joint_angle.x(), 0.04, 1e-12,
              "Roll 更新保持 Pitch 不变");
    checkNear(roll.snapshot.joint_angle.y(), -0.03, 1e-12,
              "Roll 采用请求值");
    checkClosed(roll.snapshot, "Roll IK 保持两根连杆定长");

    const double theta2_before = roll.snapshot.actuator_angle.y();
    const double theta1_target = roll.snapshot.actuator_angle.x() + 0.008;
    const UpdateOutcome theta1 = state.update(
        ControlledVariable::Theta1, theta1_target);
    check(theta1.accepted, "θ₁ 输入自动调用 FK");
    check(theta1.snapshot.direction == SolveDirection::Forward,
          "θ₁ 更新报告 FK 求解方向");
    checkNear(theta1.snapshot.actuator_angle.x(), theta1_target, 1e-12,
              "θ₁ 采用请求值");
    checkNear(theta1.snapshot.actuator_angle.y(), theta2_before, 1e-12,
              "θ₁ 更新保持 θ₂ 不变");
    checkClosed(theta1.snapshot, "θ₁ FK 保持两根连杆定长");

    const double theta1_before = theta1.snapshot.actuator_angle.x();
    const double theta2_target = theta1.snapshot.actuator_angle.y() - 0.006;
    const UpdateOutcome theta2 = state.update(
        ControlledVariable::Theta2, theta2_target);
    check(theta2.accepted, "θ₂ 输入自动调用 FK");
    checkNear(theta2.snapshot.actuator_angle.x(), theta1_before, 1e-12,
              "θ₂ 更新保持 θ₁ 不变");
    checkNear(theta2.snapshot.actuator_angle.y(), theta2_target, 1e-12,
              "θ₂ 采用请求值");
    checkClosed(theta2.snapshot, "θ₂ FK 保持两根连杆定长");
}

void testRejectedInputKeepsLastValidState() {
    using namespace rss1u;
    using namespace rss1u::studio;

    InteractiveState state(examples::arbitraryAxisExample());
    const UpdateOutcome valid = state.update(ControlledVariable::Pitch, 0.03);
    check(valid.accepted, "测试前置状态求解成功");
    const InteractiveSnapshot before = state.snapshot();

    const UpdateOutcome bad_pitch = state.update(
        ControlledVariable::Pitch, state.limits().pitch.upper + 0.1);
    check(!bad_pitch.accepted, "超出工作区的 Pitch 被拒绝");
    check(state.snapshot().joint_angle.isApprox(before.joint_angle, 0.0)
              && state.snapshot().actuator_angle.isApprox(before.actuator_angle, 0.0),
          "Pitch 请求被拒绝后，已提交状态保持不变");

    const UpdateOutcome bad_theta = state.update(
        ControlledVariable::Theta1,
        state.limits().actuator[0].upper + 0.1);
    check(!bad_theta.accepted, "超出机械限位的驱动角被拒绝");
    check(state.snapshot().joint_angle.isApprox(before.joint_angle, 0.0)
              && state.snapshot().actuator_angle.isApprox(before.actuator_angle, 0.0),
          "驱动角请求被拒绝后，已提交状态保持不变");
}

}  // 匿名命名空间

int main() {
    testAutomaticInputRouting();
    testRejectedInputKeepsLastValidState();

    if (failures == 0) {
        std::cout << "RSS1U Studio 状态测试全部通过。\n";
        return EXIT_SUCCESS;
    }
    std::cerr << failures << " 项 Studio 状态测试失败。\n";
    return EXIT_FAILURE;
}
