#include "studio_state_manager.hpp"

#include "example_geometry.hpp"

#include <Eigen/Geometry>

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QSettings>
#include <QTimer>
#include <QUuid>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace rss1u::studio {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kSharedOriginTolerance = 1e-10;
constexpr auto kSavedConfigurationKey = "studio/userConfigurationsV2";

double matrixValue(const InteractiveSnapshot& snapshot, int row, int column) noexcept {
    return snapshot.jacobian.matrix(row, column);
}

QJsonArray vectorJson(const Vec3& vector) {
    return QJsonArray{vector.x(), vector.y(), vector.z()};
}

QJsonObject branchJson(const BranchGeometry& branch) {
    QJsonObject result;
    result.insert(QStringLiteral("axisPoint"), vectorJson(branch.axis_point));
    result.insert(QStringLiteral("axis"), vectorJson(branch.axis));
    result.insert(QStringLiteral("crankZero"), vectorJson(branch.crank_zero));
    result.insert(QStringLiteral("platformOffset"), vectorJson(branch.platform_offset));
    result.insert(QStringLiteral("platformPoint"), vectorJson(branch.platform_point));
    result.insert(QStringLiteral("rodLength"), branch.rod_length);
    result.insert(QStringLiteral("motorOffset"), branch.motor_zero_offset);
    result.insert(QStringLiteral("motorLower"), branch.motor_limit.lower);
    result.insert(QStringLiteral("motorUpper"), branch.motor_limit.upper);
    return result;
}

QJsonArray geometryBranchesJson(const Geometry& geometry) {
    QJsonArray branches;
    for (const BranchGeometry& branch : geometry.branches) {
        branches.append(branchJson(branch));
    }
    return branches;
}

bool readVector(
    const QJsonObject& object,
    const QString& key,
    Vec3& result,
    QString& error,
    int branch_number) {
    const QJsonValue value = object.value(key);
    if (!value.isArray() || value.toArray().size() != 3) {
        error = QStringLiteral("支链 %1：%2 必须包含 x、y、z 三个数值")
                    .arg(branch_number).arg(key);
        return false;
    }
    const QJsonArray array = value.toArray();
    for (int component = 0; component < 3; ++component) {
        if (!array.at(component).isDouble()
            || !std::isfinite(array.at(component).toDouble())) {
            error = QStringLiteral("支链 %1：%2 的分量不是有限数值")
                        .arg(branch_number).arg(key);
            return false;
        }
        result(component) = array.at(component).toDouble();
    }
    return true;
}

bool readScalar(
    const QJsonObject& object,
    const QString& key,
    double& result,
    QString& error,
    int branch_number,
    double default_value = std::numeric_limits<double>::quiet_NaN()) {
    const QJsonValue value = object.value(key);
    if (value.isUndefined() && std::isfinite(default_value)) {
        result = default_value;
        return true;
    }
    if (!value.isDouble() || !std::isfinite(value.toDouble())) {
        error = QStringLiteral("支链 %1：%2 必须是有限数值")
                    .arg(branch_number).arg(key);
        return false;
    }
    result = value.toDouble();
    return true;
}

QString readTextFile(const QString& path) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return {};
    }
    return QString::fromUtf8(file.readAll());
}

double wrappedDistance(double left, double right) noexcept {
    return std::abs(std::remainder(left - right, 2.0 * kPi));
}

}  // namespace

StudioStateManager::StudioStateManager(QObject* parent)
    : QObject(parent), state_(examples::arbitraryAxisExample()) {
    loadConfigurations();
    if (!configurations_.empty()) {
        loadConfigurationAt(0, QStringLiteral("已载入构型 I：固定 x 轴"));
    } else {
        base_geometry_ = state_.geometry();
        current_geometry_ = base_geometry_;
        refreshAxisAngles();
        refreshDisplayPoints();
        if (!state_.valid()) {
            last_request_succeeded_ = false;
            status_text_ = QString::fromStdString(state_.initializationError());
        }
    }
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

QStringList StudioStateManager::configurationNames() const {
    QStringList result;
    result.reserve(static_cast<qsizetype>(configurations_.size()));
    for (const ConfigurationRecord& record : configurations_) {
        result.append(record.name);
    }
    return result;
}

QString StudioStateManager::currentConfigurationName() const {
    if (current_configuration_index_ < 0
        || current_configuration_index_ >= static_cast<int>(configurations_.size())) {
        return QStringLiteral("当前构型");
    }
    return configurations_[static_cast<std::size_t>(current_configuration_index_)].name;
}

bool StudioStateManager::currentConfigurationIsUser() const noexcept {
    return current_configuration_index_ >= 0
        && current_configuration_index_ < static_cast<int>(configurations_.size())
        && configurations_[static_cast<std::size_t>(current_configuration_index_)].user;
}

QString StudioStateManager::currentConfigurationJson() const {
    if (geometry_adjusted_) {
        return geometryJson(currentConfigurationName(), current_geometry_);
    }
    if (current_configuration_index_ >= 0
        && current_configuration_index_ < static_cast<int>(configurations_.size())) {
        return configurations_[static_cast<std::size_t>(current_configuration_index_)].json;
    }
    return geometryJson(currentConfigurationName(), current_geometry_);
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

QString StudioStateManager::kmnText() const {
    if (!state_.valid()) {
        return QStringLiteral("—");
    }
    const Solver solver(state_.geometry());
    const InteractiveSnapshot& snapshot = state_.snapshot();
    QStringList lines;
    for (std::size_t i = 0; i < 2; ++i) {
        const double reference = snapshot.actuator_angle(static_cast<Eigen::Index>(i));
        const BranchRoots roots = solver.branchRoots(i, snapshot.joint_angle, reference);
        if (roots.status != Status::Ok) {
            lines.append(QStringLiteral("支链 %1：系数不可用").arg(i + 1));
            continue;
        }
        const int selected = wrappedDistance(roots.angle[0], reference)
                <= wrappedDistance(roots.angle[1], reference)
            ? 0 : 1;
        lines.append(
            QStringLiteral("支链 %1  K=%2  M=%3\n        N=%4  根=%5")
                .arg(i + 1)
                .arg(QString::number(roots.K, 'g', 6))
                .arg(QString::number(roots.M, 'g', 6))
                .arg(QString::number(roots.N, 'g', 6))
                .arg(selected + 1));
    }
    return lines.join(QLatin1Char('\n'));
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
    state_ = InteractiveState(current_geometry_);
    pending_request_.reset();
    request_scheduled_ = false;
    last_request_succeeded_ = state_.valid();
    status_text_ = state_.valid()
        ? QStringLiteral("已恢复当前构型零位")
        : QString::fromStdString(state_.initializationError());
    solve_indicator_ = QStringLiteral("初始化");
    last_input_label_ = QStringLiteral("复位");
    refreshDisplayPoints();
    ++revision_;
    emit stateChanged();
    emit statusChanged();
    emit revisionChanged();
}

void StudioStateManager::selectConfiguration(int index) {
    loadConfigurationAt(index, QStringLiteral("已切换构型"));
}

void StudioStateManager::resetCurrentConfiguration() {
    loadConfigurationAt(current_configuration_index_, QStringLiteral("已恢复所选构型参数"));
}

void StudioStateManager::setAxis1AzimuthDegrees(double degrees) {
    setAxisDirection(0, degrees, axis_elevation_degrees_[0]);
}

void StudioStateManager::setAxis1ElevationDegrees(double degrees) {
    setAxisDirection(0, axis_azimuth_degrees_[0], degrees);
}

void StudioStateManager::setAxis2AzimuthDegrees(double degrees) {
    setAxisDirection(1, degrees, axis_elevation_degrees_[1]);
}

void StudioStateManager::setAxis2ElevationDegrees(double degrees) {
    setAxisDirection(1, axis_azimuth_degrees_[1], degrees);
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

void StudioStateManager::loadConfigurations() {
    configurations_.clear();
    const std::array<std::pair<const char*, const char*>, 3> built_ins{{
        {"configuration-one", ":/rss1u/configs/validation-I.json"},
        {"paper", ":/rss1u/configs/validation-II.json"},
        {"arbitrary", ":/rss1u/configs/validation-III.json"}}};

    for (const auto& [id, path] : built_ins) {
        const QString source = readTextFile(QString::fromLatin1(path));
        Geometry geometry;
        QString name;
        QString error;
        bool converted = false;
        const QString normalized = normalizedConfigurationJson(
            source, geometry, name, error, converted);
        if (!normalized.isEmpty()) {
            configurations_.push_back(ConfigurationRecord{
                QString::fromLatin1(id), name, normalized, false});
        }
    }

    if (configurations_.empty()) {
        const Geometry fallback = examples::arbitraryAxisExample();
        configurations_.push_back(ConfigurationRecord{
            QStringLiteral("arbitrary"),
            QStringLiteral("任意空间驱动轴"),
            geometryJson(QStringLiteral("任意空间驱动轴"), fallback),
            false});
    }

    const QByteArray stored = QSettings().value(QString::fromLatin1(kSavedConfigurationKey)).toByteArray();
    const QJsonDocument stored_document = QJsonDocument::fromJson(stored);
    if (!stored_document.isArray()) {
        return;
    }
    for (const QJsonValue& item : stored_document.array()) {
        if (!item.isObject()) {
            continue;
        }
        const QJsonObject object = item.toObject();
        const QString source = object.value(QStringLiteral("document")).toString();
        Geometry geometry;
        QString name;
        QString error;
        bool converted = false;
        const QString normalized = normalizedConfigurationJson(
            source, geometry, name, error, converted);
        if (normalized.isEmpty()) {
            continue;
        }
        const QString id = object.value(QStringLiteral("id")).toString(
            QUuid::createUuid().toString(QUuid::WithoutBraces));
        configurations_.push_back(ConfigurationRecord{id, name, normalized, true});
    }
}

void StudioStateManager::persistUserConfigurations() const {
    QJsonArray items;
    for (const ConfigurationRecord& record : configurations_) {
        if (!record.user) {
            continue;
        }
        QJsonObject item;
        item.insert(QStringLiteral("id"), record.id);
        item.insert(QStringLiteral("document"), record.json);
        items.append(item);
    }
    QSettings().setValue(
        QString::fromLatin1(kSavedConfigurationKey),
        QJsonDocument(items).toJson(QJsonDocument::Compact));
}

void StudioStateManager::loadConfigurationAt(int index, const QString& reason) {
    if (index < 0 || index >= static_cast<int>(configurations_.size())) {
        return;
    }
    const ConfigurationRecord& record = configurations_[static_cast<std::size_t>(index)];
    Geometry geometry;
    QString name;
    QString error;
    bool converted = false;
    const QString normalized = normalizedConfigurationJson(
        record.json, geometry, name, error, converted);
    if (normalized.isEmpty()) {
        last_request_succeeded_ = false;
        status_text_ = QStringLiteral("构型载入失败：") + error;
        emit statusChanged();
        return;
    }
    if (!replaceGeometry(geometry, reason + QStringLiteral("：") + name, false)) {
        return;
    }
    current_configuration_index_ = index;
    base_geometry_ = geometry;
    current_geometry_ = geometry;
    geometry_adjusted_ = false;
    refreshAxisAngles();
    emit configurationsChanged();
}

bool StudioStateManager::replaceGeometry(
    const Geometry& geometry,
    const QString& reason,
    bool preserve_pose) {
    InteractiveState candidate(geometry);
    if (!candidate.valid()) {
        last_request_succeeded_ = false;
        status_text_ = reason + QStringLiteral("失败：")
            + QString::fromStdString(candidate.initializationError());
        emit statusChanged();
        return false;
    }

    if (preserve_pose && state_.valid()) {
        const Vec2 previous_joint = state_.snapshot().joint_angle;
        const UpdateOutcome pitch = candidate.update(
            ControlledVariable::Pitch, previous_joint.x());
        if (!pitch.accepted) {
            last_request_succeeded_ = false;
            status_text_ = reason + QStringLiteral("失败：当前 Pitch 在新几何中不可达");
            emit statusChanged();
            return false;
        }
        const UpdateOutcome roll = candidate.update(
            ControlledVariable::Roll, previous_joint.y());
        if (!roll.accepted) {
            last_request_succeeded_ = false;
            status_text_ = reason + QStringLiteral("失败：当前 Roll 在新几何中不可达");
            emit statusChanged();
            return false;
        }
    }

    state_ = std::move(candidate);
    current_geometry_ = geometry;
    pending_request_.reset();
    request_scheduled_ = false;
    last_request_succeeded_ = true;
    status_text_ = reason;
    solve_indicator_ = QStringLiteral("几何");
    last_input_label_ = QStringLiteral("构型");
    refreshDisplayPoints();
    ++revision_;
    emit stateChanged();
    emit statusChanged();
    emit revisionChanged();
    return true;
}

void StudioStateManager::setAxisDirection(
    std::size_t branch_index,
    double azimuth,
    double elevation) {
    if (branch_index >= 2 || !std::isfinite(azimuth) || !std::isfinite(elevation)) {
        return;
    }
    const double bounded_azimuth = std::clamp(azimuth, -180.0, 180.0);
    const double bounded_elevation = std::clamp(elevation, -85.0, 85.0);
    const double old_azimuth = axis_azimuth_degrees_[branch_index];
    const double old_elevation = axis_elevation_degrees_[branch_index];
    axis_azimuth_degrees_[branch_index] = bounded_azimuth;
    axis_elevation_degrees_[branch_index] = bounded_elevation;
    const Geometry candidate = reorientedGeometry(
        base_geometry_, axis_azimuth_degrees_, axis_elevation_degrees_);
    if (!replaceGeometry(candidate, QStringLiteral("驱动轴方向已更新"), true)) {
        axis_azimuth_degrees_[branch_index] = old_azimuth;
        axis_elevation_degrees_[branch_index] = old_elevation;
        emit configurationsChanged();
        return;
    }
    geometry_adjusted_ = true;
    emit configurationsChanged();
}

Geometry StudioStateManager::reorientedGeometry(
    const Geometry& base,
    const std::array<double, 2>& azimuth_degrees,
    const std::array<double, 2>& elevation_degrees) {
    Geometry result = base;
    for (std::size_t i = 0; i < 2; ++i) {
        const double azimuth = toRadians(azimuth_degrees[i]);
        const double elevation = toRadians(elevation_degrees[i]);
        const double ce = std::cos(elevation);
        const Vec3 new_axis(
            ce * std::cos(azimuth),
            ce * std::sin(azimuth),
            std::sin(elevation));

        const BranchGeometry& source = base.branches[i];
        BranchGeometry& target = result.branches[i];
        const Vec3 old_axis = source.axis.normalized();
        const Vec3 neutral_crank_point = source.axis_point + source.crank_zero;
        const double axial_shift = old_axis.dot(source.crank_zero);
        const Vec3 old_transverse = source.crank_zero - axial_shift * old_axis;
        const double radius = old_transverse.norm();
        Vec3 projected = old_transverse - new_axis * new_axis.dot(old_transverse);
        if (projected.norm() < 1e-9) {
            const Vec3 helper = std::abs(new_axis.z()) < 0.9
                ? Vec3::UnitZ() : Vec3::UnitX();
            projected = helper.cross(new_axis);
        }
        const Vec3 transverse = radius * projected.normalized();
        target.axis = new_axis;
        target.crank_zero = transverse + axial_shift * new_axis;
        target.axis_point = neutral_crank_point - target.crank_zero;
    }
    return result;
}

void StudioStateManager::refreshAxisAngles() {
    for (std::size_t i = 0; i < 2; ++i) {
        const Vec3 axis = base_geometry_.branches[i].axis.normalized();
        axis_azimuth_degrees_[i] = toDegrees(std::atan2(axis.y(), axis.x()));
        axis_elevation_degrees_[i] = toDegrees(
            std::asin(std::clamp(axis.z(), -1.0, 1.0)));
    }
}

QString StudioStateManager::normalizedConfigurationJson(
    const QString& source,
    Geometry& geometry,
    QString& name,
    QString& error,
    bool& converted) const {
    converted = false;
    QJsonParseError parse_error;
    const QJsonDocument document = QJsonDocument::fromJson(source.toUtf8(), &parse_error);
    if (parse_error.error != QJsonParseError::NoError) {
        error = QStringLiteral("JSON 格式错误：") + parse_error.errorString();
        return {};
    }

    QJsonObject root;
    if (document.isObject()) {
        root = document.object();
    } else {
        error = QStringLiteral("JSON 顶层必须是对象");
        return {};
    }
    if (!root.value(QStringLiteral("branches")).isArray()
        && root.value(QStringLiteral("items")).isArray()
        && !root.value(QStringLiteral("items")).toArray().isEmpty()
        && root.value(QStringLiteral("items")).toArray().first().isObject()) {
        root = root.value(QStringLiteral("items")).toArray().first().toObject();
    }

    const QJsonArray branches = root.value(QStringLiteral("branches")).toArray();
    if (branches.size() != 2) {
        error = QStringLiteral("构型必须包含两条 RSS 支链");
        return {};
    }

    for (int index = 0; index < 2; ++index) {
        if (!branches.at(index).isObject()) {
            error = QStringLiteral("支链 %1 参数不存在").arg(index + 1);
            return {};
        }
        const QJsonObject object = branches.at(index).toObject();
        BranchGeometry& branch = geometry.branches[static_cast<std::size_t>(index)];
        if (!readVector(object, QStringLiteral("axisPoint"), branch.axis_point, error, index + 1)
            || !readVector(object, QStringLiteral("axis"), branch.axis, error, index + 1)
            || !readVector(object, QStringLiteral("crankZero"), branch.crank_zero, error, index + 1)
            || !readVector(object, QStringLiteral("platformOffset"), branch.platform_offset, error, index + 1)
            || !readVector(object, QStringLiteral("platformPoint"), branch.platform_point, error, index + 1)
            || !readScalar(object, QStringLiteral("rodLength"), branch.rod_length, error, index + 1)
            || !readScalar(object, QStringLiteral("motorOffset"), branch.motor_zero_offset, error, index + 1, 0.0)
            || !readScalar(object, QStringLiteral("motorLower"), branch.motor_limit.lower, error, index + 1, -2.6)
            || !readScalar(object, QStringLiteral("motorUpper"), branch.motor_limit.upper, error, index + 1, 2.6)) {
            return {};
        }
        if (branch.axis.norm() < 1e-12) {
            error = QStringLiteral("支链 %1：电机轴方向不能为零向量").arg(index + 1);
            return {};
        }
        branch.axis.normalize();
        const Vec3 transverse = branch.crank_zero
            - branch.axis * branch.axis.dot(branch.crank_zero);
        if (transverse.norm() < 1e-9) {
            error = QStringLiteral("支链 %1：曲柄零位向量不能与电机轴平行").arg(index + 1);
            return {};
        }
        if (branch.rod_length <= 0.0) {
            error = QStringLiteral("支链 %1：连杆长度必须大于 0 mm").arg(index + 1);
            return {};
        }
        if (branch.motor_limit.lower > branch.motor_limit.upper) {
            error = QStringLiteral("支链 %1：电机下限不能大于上限").arg(index + 1);
            return {};
        }
    }

    if ((geometry.branches[0].platform_offset
            - geometry.branches[1].platform_offset).norm() > kSharedOriginTolerance) {
        for (BranchGeometry& branch : geometry.branches) {
            branch.axis_point -= branch.platform_offset;
            branch.platform_offset.setZero();
        }
        root.remove(QStringLiteral("expressions"));
        converted = true;
    }

    const InteractiveState validation(geometry);
    if (!validation.valid()) {
        error = QStringLiteral("零位求解失败：")
            + QString::fromStdString(validation.initializationError());
        return {};
    }

    name = root.value(QStringLiteral("name")).toString().trimmed();
    if (name.isEmpty()) {
        name = QStringLiteral("未命名构型");
    }
    root.insert(QStringLiteral("version"), 2);
    root.insert(QStringLiteral("name"), name);
    root.insert(QStringLiteral("branches"), geometryBranchesJson(geometry));
    root.remove(QStringLiteral("_studioId"));
    return QString::fromUtf8(QJsonDocument(root).toJson(QJsonDocument::Indented));
}

QString StudioStateManager::geometryJson(
    const QString& name,
    const Geometry& geometry) {
    QJsonObject root;
    root.insert(QStringLiteral("version"), 2);
    root.insert(QStringLiteral("name"), name);
    root.insert(QStringLiteral("branches"), geometryBranchesJson(geometry));
    return QString::fromUtf8(QJsonDocument(root).toJson(QJsonDocument::Indented));
}

bool StudioStateManager::saveConfigurationJson(
    const QString& json,
    bool overwrite_current) {
    Geometry geometry;
    QString name;
    QString error;
    bool converted = false;
    const QString normalized = normalizedConfigurationJson(
        json, geometry, name, error, converted);
    if (normalized.isEmpty()) {
        last_request_succeeded_ = false;
        status_text_ = QStringLiteral("构型保存失败：") + error;
        emit statusChanged();
        return false;
    }

    int target_index = -1;
    if (overwrite_current && currentConfigurationIsUser()) {
        target_index = current_configuration_index_;
        ConfigurationRecord& record = configurations_[static_cast<std::size_t>(target_index)];
        record.name = name;
        record.json = normalized;
    } else {
        configurations_.push_back(ConfigurationRecord{
            QUuid::createUuid().toString(QUuid::WithoutBraces),
            name,
            normalized,
            true});
        target_index = static_cast<int>(configurations_.size()) - 1;
    }
    persistUserConfigurations();
    loadConfigurationAt(target_index, converted
        ? QStringLiteral("构型已保存，并已等效转换为公共 O 坐标系")
        : QStringLiteral("构型已检查并保存"));
    return current_configuration_index_ == target_index;
}

bool StudioStateManager::deleteCurrentConfiguration() {
    if (!currentConfigurationIsUser()) {
        last_request_succeeded_ = false;
        status_text_ = QStringLiteral("内置构型不能删除；可新建副本后编辑");
        emit statusChanged();
        return false;
    }
    configurations_.erase(configurations_.begin() + current_configuration_index_);
    current_configuration_index_ = -1;
    persistUserConfigurations();
    loadConfigurationAt(0, QStringLiteral("已删除保存构型并返回构型 I"));
    return true;
}

QString StudioStateManager::loadConfigurationFile(const QUrl& url) {
    if (!url.isLocalFile()) {
        last_request_succeeded_ = false;
        status_text_ = QStringLiteral("只能导入本地 JSON 文件");
        emit statusChanged();
        return {};
    }
    const QString source = readTextFile(url.toLocalFile());
    Geometry geometry;
    QString name;
    QString error;
    bool converted = false;
    const QString normalized = normalizedConfigurationJson(
        source, geometry, name, error, converted);
    if (normalized.isEmpty()) {
        last_request_succeeded_ = false;
        status_text_ = QStringLiteral("导入失败：") + error;
        emit statusChanged();
        return {};
    }
    last_request_succeeded_ = true;
    status_text_ = converted
        ? QStringLiteral("JSON 已载入，并已等效转换为公共 O 坐标系；请检查后保存")
        : QStringLiteral("JSON 已载入；请检查后保存");
    emit statusChanged();
    return normalized;
}

bool StudioStateManager::writeConfigurationFile(
    const QUrl& url,
    const QString& json) {
    if (!url.isLocalFile()) {
        last_request_succeeded_ = false;
        status_text_ = QStringLiteral("只能导出到本地文件");
        emit statusChanged();
        return false;
    }
    QString path = url.toLocalFile();
    if (!path.endsWith(QStringLiteral(".json"), Qt::CaseInsensitive)) {
        path += QStringLiteral(".json");
    }
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        last_request_succeeded_ = false;
        status_text_ = QStringLiteral("导出失败：无法写入所选文件");
        emit statusChanged();
        return false;
    }
    const QByteArray content = json.toUtf8();
    if (file.write(content) != content.size()) {
        last_request_succeeded_ = false;
        status_text_ = QStringLiteral("导出失败：文件未完整写入");
        emit statusChanged();
        return false;
    }
    last_request_succeeded_ = true;
    status_text_ = QStringLiteral("当前构型已导出为 JSON");
    emit statusChanged();
    return true;
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

}  // namespace rss1u::studio
