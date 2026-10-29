#pragma once

#include "interactive_state.hpp"

#include <QObject>
#include <QString>
#include <QVector3D>

#include <optional>

namespace rss1u::studio {

class StudioStateManager final : public QObject {
    Q_OBJECT

    Q_PROPERTY(double pitchDegrees READ pitchDegrees NOTIFY stateChanged)
    Q_PROPERTY(double rollDegrees READ rollDegrees NOTIFY stateChanged)
    Q_PROPERTY(double theta1Degrees READ theta1Degrees NOTIFY stateChanged)
    Q_PROPERTY(double theta2Degrees READ theta2Degrees NOTIFY stateChanged)

    Q_PROPERTY(double pitchMinimumDegrees READ pitchMinimumDegrees CONSTANT)
    Q_PROPERTY(double pitchMaximumDegrees READ pitchMaximumDegrees CONSTANT)
    Q_PROPERTY(double rollMinimumDegrees READ rollMinimumDegrees CONSTANT)
    Q_PROPERTY(double rollMaximumDegrees READ rollMaximumDegrees CONSTANT)
    Q_PROPERTY(double theta1MinimumDegrees READ theta1MinimumDegrees CONSTANT)
    Q_PROPERTY(double theta1MaximumDegrees READ theta1MaximumDegrees CONSTANT)
    Q_PROPERTY(double theta2MinimumDegrees READ theta2MinimumDegrees CONSTANT)
    Q_PROPERTY(double theta2MaximumDegrees READ theta2MaximumDegrees CONSTANT)

    Q_PROPERTY(bool lastRequestSucceeded READ lastRequestSucceeded NOTIFY statusChanged)
    Q_PROPERTY(QString statusText READ statusText NOTIFY statusChanged)
    Q_PROPERTY(QString solveIndicator READ solveIndicator NOTIFY statusChanged)
    Q_PROPERTY(QString lastInputLabel READ lastInputLabel NOTIFY statusChanged)
    Q_PROPERTY(qulonglong revision READ revision NOTIFY revisionChanged)

    Q_PROPERTY(double closureError1 READ closureError1 NOTIFY stateChanged)
    Q_PROPERTY(double closureError2 READ closureError2 NOTIFY stateChanged)
    Q_PROPERTY(bool jacobianAvailable READ jacobianAvailable NOTIFY stateChanged)
    Q_PROPERTY(double jacobian11 READ jacobian11 NOTIFY stateChanged)
    Q_PROPERTY(double jacobian12 READ jacobian12 NOTIFY stateChanged)
    Q_PROPERTY(double jacobian21 READ jacobian21 NOTIFY stateChanged)
    Q_PROPERTY(double jacobian22 READ jacobian22 NOTIFY stateChanged)
    Q_PROPERTY(double jacobianDeterminant READ jacobianDeterminant NOTIFY stateChanged)
    Q_PROPERTY(double jacobianCondition READ jacobianCondition NOTIFY stateChanged)
    Q_PROPERTY(int forwardIterations READ forwardIterations NOTIFY stateChanged)

    Q_PROPERTY(QVector3D origin READ origin CONSTANT)
    Q_PROPERTY(QVector3D m1 READ m1 NOTIFY stateChanged)
    Q_PROPERTY(QVector3D a1 READ a1 NOTIFY stateChanged)
    Q_PROPERTY(QVector3D b1 READ b1 NOTIFY stateChanged)
    Q_PROPERTY(QVector3D m2 READ m2 NOTIFY stateChanged)
    Q_PROPERTY(QVector3D a2 READ a2 NOTIFY stateChanged)
    Q_PROPERTY(QVector3D b2 READ b2 NOTIFY stateChanged)
    Q_PROPERTY(QVector3D axis1Start READ axis1Start NOTIFY stateChanged)
    Q_PROPERTY(QVector3D axis1End READ axis1End NOTIFY stateChanged)
    Q_PROPERTY(QVector3D axis2Start READ axis2Start NOTIFY stateChanged)
    Q_PROPERTY(QVector3D axis2End READ axis2End NOTIFY stateChanged)

public:
    explicit StudioStateManager(QObject* parent = nullptr);

    [[nodiscard]] double pitchDegrees() const noexcept;
    [[nodiscard]] double rollDegrees() const noexcept;
    [[nodiscard]] double theta1Degrees() const noexcept;
    [[nodiscard]] double theta2Degrees() const noexcept;

    [[nodiscard]] double pitchMinimumDegrees() const noexcept;
    [[nodiscard]] double pitchMaximumDegrees() const noexcept;
    [[nodiscard]] double rollMinimumDegrees() const noexcept;
    [[nodiscard]] double rollMaximumDegrees() const noexcept;
    [[nodiscard]] double theta1MinimumDegrees() const noexcept;
    [[nodiscard]] double theta1MaximumDegrees() const noexcept;
    [[nodiscard]] double theta2MinimumDegrees() const noexcept;
    [[nodiscard]] double theta2MaximumDegrees() const noexcept;

    [[nodiscard]] bool lastRequestSucceeded() const noexcept { return last_request_succeeded_; }
    [[nodiscard]] QString statusText() const { return status_text_; }
    [[nodiscard]] QString solveIndicator() const { return solve_indicator_; }
    [[nodiscard]] QString lastInputLabel() const { return last_input_label_; }
    [[nodiscard]] qulonglong revision() const noexcept { return revision_; }

    [[nodiscard]] double closureError1() const noexcept;
    [[nodiscard]] double closureError2() const noexcept;
    [[nodiscard]] bool jacobianAvailable() const noexcept;
    [[nodiscard]] double jacobian11() const noexcept;
    [[nodiscard]] double jacobian12() const noexcept;
    [[nodiscard]] double jacobian21() const noexcept;
    [[nodiscard]] double jacobian22() const noexcept;
    [[nodiscard]] double jacobianDeterminant() const noexcept;
    [[nodiscard]] double jacobianCondition() const noexcept;
    [[nodiscard]] int forwardIterations() const noexcept;

    [[nodiscard]] QVector3D origin() const noexcept { return QVector3D(); }
    [[nodiscard]] QVector3D m1() const noexcept { return points_[0].m; }
    [[nodiscard]] QVector3D a1() const noexcept { return points_[0].a; }
    [[nodiscard]] QVector3D b1() const noexcept { return points_[0].b; }
    [[nodiscard]] QVector3D m2() const noexcept { return points_[1].m; }
    [[nodiscard]] QVector3D a2() const noexcept { return points_[1].a; }
    [[nodiscard]] QVector3D b2() const noexcept { return points_[1].b; }
    [[nodiscard]] QVector3D axis1Start() const noexcept { return points_[0].axis_start; }
    [[nodiscard]] QVector3D axis1End() const noexcept { return points_[0].axis_end; }
    [[nodiscard]] QVector3D axis2Start() const noexcept { return points_[1].axis_start; }
    [[nodiscard]] QVector3D axis2End() const noexcept { return points_[1].axis_end; }

    Q_INVOKABLE void submitPitch(double degrees);
    Q_INVOKABLE void submitRoll(double degrees);
    Q_INVOKABLE void submitTheta1(double degrees);
    Q_INVOKABLE void submitTheta2(double degrees);
    Q_INVOKABLE void reportInputError(const QString& variable_name);
    Q_INVOKABLE void resetToZero();

signals:
    void stateChanged();
    void statusChanged();
    void revisionChanged();

private:
    struct PendingRequest {
        ControlledVariable variable{ControlledVariable::Pitch};
        double radians{0.0};
    };

    struct DisplayPoints {
        QVector3D m{};
        QVector3D a{};
        QVector3D b{};
        QVector3D axis_start{};
        QVector3D axis_end{};
    };

    void queueRequest(ControlledVariable variable, double degrees);
    void processLatestRequest();
    void refreshDisplayPoints();
    void publishOutcome(const UpdateOutcome& outcome, ControlledVariable source);
    [[nodiscard]] static QString variableLabel(ControlledVariable variable);
    [[nodiscard]] static double toDegrees(double radians) noexcept;
    [[nodiscard]] static double toRadians(double degrees) noexcept;
    [[nodiscard]] static QVector3D toVector3D(const Vec3& vector) noexcept;

    InteractiveState state_;
    std::optional<PendingRequest> pending_request_{};
    bool request_scheduled_{false};
    bool solving_{false};
    bool last_request_succeeded_{true};
    QString status_text_{QStringLiteral("初始化完成")};
    QString solve_indicator_{QStringLiteral("初始化")};
    QString last_input_label_{QStringLiteral("—")};
    qulonglong revision_{0};
    std::array<DisplayPoints, 2> points_{};
};

}  // 命名空间 rss1u::studio
