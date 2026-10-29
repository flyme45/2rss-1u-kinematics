#include "studio_state_manager.hpp"

#include "example_geometry.hpp"

#include <QTimer>

#include <algorithm>
#include <cmath>
#include <limits>

namespace rss1u::studio {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

double matrixValue(const InteractiveSnapshot& snapshot, int row, int column) noexcept {
    return snapshot.jacobian.matrix(row, column);
}

}  // 匿名命名空间

StudioStateManager::StudioStateManager(QObject* parent)
    : QObject(parent), state_(examples::arbitraryAxisExample()) {
    if (!state_.valid()) {
        last_request_succeeded_ = false;
        status_text_ = QString::fromStdString(state_.initializationError());
    }
    refreshDisplayPoints();
}

double StudioStateManager::toDegrees(double radians) noexcept {
    return radians * 180.0 / kPi;
}

double StudioStateManager::toRadians(double degrees) noexcept {
    return degrees * kPi / 180.0;
}

double StudioStateManager::pitchDegrees() const noexcept {
    return toDegrees(state_.snapshot().joint_angle.x());
}

double StudioStateManager::rollDegrees() const noexcept {
    return toDegrees(state_.snapshot().joint_angle.y());
}

double StudioStateManager::theta1Degrees() const noexcept {
    return toDegrees(state_.snapshot().actuator_angle.x());
}

double StudioStateManager::theta2Degrees() const noexcept {
    return toDegrees(state_.snapshot().actuator_angle.y());
}

double StudioStateManager::pitchMinimumDegrees() const noexcept {
    return toDegrees(state_.limits().pitch.lower);
}

double StudioStateManager::pitchMaximumDegrees() const noexcept {
    return toDegrees(state_.limits().pitch.upper);
}

double StudioStateManager::rollMinimumDegrees() const noexcept {
    return toDegrees(state_.limits().roll.lower);
}

double StudioStateManager::rollMaximumDegrees() const noexcept {
    return toDegrees(state_.limits().roll.upper);
}

double StudioStateManager::theta1MinimumDegrees() const noexcept {
    return toDegrees(state_.limits().actuator[0].lower);
}

double StudioStateManager::theta1MaximumDegrees() const noexcept {
    return toDegrees(state_.limits().actuator[0].upper);
}

double StudioStateManager::theta2MinimumDegrees() const noexcept {
    return toDegrees(state_.limits().actuator[1].lower);
}

double StudioStateManager::theta2MaximumDegrees() const noexcept {
    return toDegrees(state_.limits().actuator[1].upper);
}

double StudioStateManager::closureError1() const noexcept {
    return state_.snapshot().closure_error.x();
}

double StudioStateManager::closureError2() const noexcept {
    return state_.snapshot().closure_error.y();
}

bool StudioStateManager::jacobianAvailable() const noexcept {
    return state_.snapshot().jacobian.ok();
}

double StudioStateManager::jacobian11() const noexcept {
    return matrixValue(state_.snapshot(), 0, 0);
}

double StudioStateManager::jacobian12() const noexcept {
    return matrixValue(state_.snapshot(), 0, 1);
}

double StudioStateManager::jacobian21() const noexcept {
    return matrixValue(state_.snapshot(), 1, 0);
}

double StudioStateManager::jacobian22() const noexcept {
    return matrixValue(state_.snapshot(), 1, 1);
}

double StudioStateManager::jacobianDeterminant() const noexcept {
    return state_.snapshot().jacobian.determinant;
}

double StudioStateManager::jacobianCondition() const noexcept {
    return state_.snapshot().jacobian.condition_frobenius;
}

int StudioStateManager::forwardIterations() const noexcept {
    return state_.snapshot().forward_iterations;
}

void StudioStateManager::submitPitch(double degrees) {
    queueRequest(ControlledVariable::Pitch, degrees);
}

void StudioStateManager::submitRoll(double degrees) {
    queueRequest(ControlledVariable::Roll, degrees);
}

void StudioStateManager::submitTheta1(double degrees) {
    queueRequest(ControlledVariable::Theta1, degrees);
}

void StudioStateManager::submitTheta2(double degrees) {
    queueRequest(ControlledVariable::Theta2, degrees);
}

void StudioStateManager::reportInputError(const QString& variable_name) {
    last_request_succeeded_ = false;
    status_text_ = variable_name + QStringLiteral(" 输入无效：请输入范围内的小数");
    last_input_label_ = variable_name;
    emit statusChanged();
}

void StudioStateManager::resetToZero() {
    // 若分别提交两个坐标，会执行两次求解并短暂暴露中间状态；重建状态可使
    // 复位成为原子操作，同时恢复与初始化一致的装配分支参考。
    state_ = InteractiveState(examples::arbitraryAxisExample());
    pending_request_.reset();
    request_scheduled_ = false;
    last_request_succeeded_ = state_.valid();
    status_text_ = state_.valid()
        ? QStringLiteral("已恢复零位")
        : QString::fromStdString(state_.initializationError());
    solve_indicator_ = QStringLiteral("初始化");
    last_input_label_ = QStringLiteral("复位");
    refreshDisplayPoints();
    ++revision_;
    emit stateChanged();
    emit statusChanged();
    emit revisionChanged();
}

void StudioStateManager::queueRequest(ControlledVariable variable, double degrees) {
    if (!std::isfinite(degrees)) {
        reportInputError(variableLabel(variable));
        return;
    }
    pending_request_ = PendingRequest{variable, toRadians(degrees)};
    if (request_scheduled_ || solving_) {
        return;
    }
    request_scheduled_ = true;
    QTimer::singleShot(0, this, &StudioStateManager::processLatestRequest);
}

void StudioStateManager::processLatestRequest() {
    request_scheduled_ = false;
    if (!pending_request_ || solving_) {
        return;
    }

    solving_ = true;
    const PendingRequest request = *pending_request_;
    pending_request_.reset();
    const UpdateOutcome outcome = state_.update(request.variable, request.radians);
    publishOutcome(outcome, request.variable);
    solving_ = false;

    // 后到的拖动事件覆盖旧请求，不在过期请求之后排队。
    if (pending_request_ && !request_scheduled_) {
        request_scheduled_ = true;
        QTimer::singleShot(0, this, &StudioStateManager::processLatestRequest);
    }
}

void StudioStateManager::publishOutcome(
    const UpdateOutcome& outcome,
    ControlledVariable source) {
    last_request_succeeded_ = outcome.accepted;
    status_text_ = QString::fromStdString(outcome.reason);
    last_input_label_ = variableLabel(source);
    solve_indicator_ = (source == ControlledVariable::Pitch
        || source == ControlledVariable::Roll)
        ? QStringLiteral("IK") : QStringLiteral("FK");
    if (outcome.accepted) {
        refreshDisplayPoints();
    }
    ++revision_;
    emit stateChanged();
    emit statusChanged();
    emit revisionChanged();
}

QVector3D StudioStateManager::toVector3D(const Vec3& vector) noexcept {
    return QVector3D(
        static_cast<float>(vector.x()),
        static_cast<float>(vector.y()),
        static_cast<float>(vector.z()));
}

void StudioStateManager::refreshDisplayPoints() {
    if (!state_.valid()) {
        return;
    }
    const InteractiveSnapshot& snapshot = state_.snapshot();
    const Mat3 rotation = Solver::platformRotation(
        snapshot.joint_angle.x(), snapshot.joint_angle.y());

    for (std::size_t i = 0; i < state_.geometry().branches.size(); ++i) {
        const BranchGeometry& branch = state_.geometry().branches[i];
        const Vec3 axis = branch.axis.normalized();
        const Vec3 m = branch.axis_point - branch.platform_offset;
        const Vec3 a = branch.axis_point
            + Solver::axisRotation(
                axis,
                snapshot.actuator_angle(static_cast<Eigen::Index>(i))
                    + branch.motor_zero_offset)
                * branch.crank_zero
            - branch.platform_offset;
        const Vec3 b = rotation * branch.platform_point;
        const double half_axis_length = std::max(65.0, 0.24 * branch.rod_length);
        points_[i].m = toVector3D(m);
        points_[i].a = toVector3D(a);
        points_[i].b = toVector3D(b);
        points_[i].axis_start = toVector3D(m - half_axis_length * axis);
        points_[i].axis_end = toVector3D(m + half_axis_length * axis);
    }
}

QString StudioStateManager::variableLabel(ControlledVariable variable) {
    switch (variable) {
        case ControlledVariable::Pitch: return QStringLiteral("Pitch");
        case ControlledVariable::Roll: return QStringLiteral("Roll");
        case ControlledVariable::Theta1: return QStringLiteral("θ₁");
        case ControlledVariable::Theta2: return QStringLiteral("θ₂");
    }
    return QStringLiteral("未知变量");
}

}  // 命名空间 rss1u::studio
