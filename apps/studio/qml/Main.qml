import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

ApplicationWindow {
    id: window
    width: 1460
    height: 920
    minimumWidth: 1120
    minimumHeight: 760
    visible: true
    title: "RSS1U Kinematics Studio"
    color: "#071017"

    function numberText(value, digits) {
        if (!Number.isFinite(value))
            return "—"
        if (value !== 0 && (Math.abs(value) >= 10000 || Math.abs(value) < 0.001))
            return Number(value).toExponential(3)
        return Number(value).toFixed(digits)
    }

    header: Rectangle {
        implicitHeight: 70
        color: "#09141c"
        border.color: "#1f3441"

        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 22
            anchors.rightMargin: 22
            spacing: 16

            ColumnLayout {
                spacing: 1
                Label {
                    text: "RSS1U KINEMATICS STUDIO"
                    color: "#62d7d5"
                    font.pixelSize: 10
                    font.bold: true
                    font.letterSpacing: 2
                }
                Label {
                    text: "2-RSS-1U 统一运动学工作台"
                    color: "#edf7fb"
                    font.pixelSize: 20
                    font.weight: Font.DemiBold
                }
            }

            Item { Layout.fillWidth: true }

            Rectangle {
                implicitWidth: modeRow.implicitWidth + 22
                implicitHeight: 36
                radius: 18
                color: studio.lastRequestSucceeded ? "#102a2a" : "#301820"
                border.color: studio.lastRequestSucceeded ? "#357d78" : "#934654"

                RowLayout {
                    id: modeRow
                    anchors.centerIn: parent
                    spacing: 8
                    Rectangle {
                        width: 8
                        height: 8
                        radius: 4
                        color: studio.lastRequestSucceeded ? "#75e1c1" : "#ff7180"
                    }
                    Label {
                        text: studio.solveIndicator + " · " + studio.lastInputLabel
                        color: studio.lastRequestSucceeded ? "#bde9dc" : "#ffc1c7"
                        font.pixelSize: 12
                    }
                }
            }

            Button {
                text: "恢复零位"
                onClicked: studio.resetToZero()
            }
        }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 14
        spacing: 12

        Rectangle {
            Layout.fillWidth: true
            implicitHeight: 38
            radius: 7
            color: studio.lastRequestSucceeded ? "#0e211f" : "#2a141a"
            border.color: studio.lastRequestSucceeded ? "#244a47" : "#7b3541"

            Label {
                anchors.fill: parent
                anchors.leftMargin: 13
                anchors.rightMargin: 13
                verticalAlignment: Text.AlignVCenter
                text: studio.statusText
                color: studio.lastRequestSucceeded ? "#8de0c7" : "#ff9aa4"
                font.pixelSize: 12
                elide: Text.ElideRight
            }
        }

        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 12

            MechanismView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.minimumWidth: 650
            }

            Rectangle {
                Layout.preferredWidth: 330
                Layout.fillHeight: true
                radius: 10
                color: "#0c1821"
                border.color: "#243b49"

                ScrollView {
                    anchors.fill: parent
                    anchors.margins: 14
                    clip: true

                    ColumnLayout {
                        width: parent.width
                        spacing: 12

                        Label {
                            text: "实时分析"
                            color: "#f0f7fa"
                            font.pixelSize: 17
                            font.weight: Font.DemiBold
                        }
                        Label {
                            text: "自动识别输入源，无需手动切换 IK/FK"
                            color: "#78929f"
                            font.pixelSize: 11
                        }

                        GridLayout {
                            Layout.fillWidth: true
                            columns: 2
                            columnSpacing: 8
                            rowSpacing: 8

                            Repeater {
                                model: [
                                    ["Pitch", studio.pitchDegrees, "°"],
                                    ["Roll", studio.rollDegrees, "°"],
                                    ["θ₁", studio.theta1Degrees, "°"],
                                    ["θ₂", studio.theta2Degrees, "°"]
                                ]
                                delegate: Rectangle {
                                    required property var modelData
                                    Layout.fillWidth: true
                                    implicitHeight: 60
                                    radius: 7
                                    color: "#08131a"
                                    border.color: "#203744"
                                    Column {
                                        anchors.fill: parent
                                        anchors.margins: 9
                                        spacing: 3
                                        Label { text: modelData[0]; color: "#78929f"; font.pixelSize: 10 }
                                        Label {
                                            text: window.numberText(modelData[1], 4) + modelData[2]
                                            color: "#edf7fb"
                                            font.family: "Consolas"
                                            font.pixelSize: 14
                                            font.weight: Font.DemiBold
                                        }
                                    }
                                }
                            }
                        }

                        Rectangle { Layout.fillWidth: true; height: 1; color: "#203744" }

                        Label { text: "闭环长度误差 / mm"; color: "#8da5b0"; font.pixelSize: 11 }
                        Label {
                            text: "[ " + window.numberText(studio.closureError1, 7)
                                  + ",  " + window.numberText(studio.closureError2, 7) + " ]"
                            color: "#d5e5eb"
                            font.family: "Consolas"
                            font.pixelSize: 12
                        }

                        Label { text: "J = ∂θ / ∂q"; color: "#8da5b0"; font.pixelSize: 11 }
                        Rectangle {
                            Layout.fillWidth: true
                            implicitHeight: 58
                            radius: 6
                            color: "#071119"
                            Label {
                                anchors.centerIn: parent
                                text: studio.jacobianAvailable
                                      ? "[ " + window.numberText(studio.jacobian11, 5)
                                        + "  " + window.numberText(studio.jacobian12, 5) + " ]\n"
                                        + "[ " + window.numberText(studio.jacobian21, 5)
                                        + "  " + window.numberText(studio.jacobian22, 5) + " ]"
                                      : "当前 Jacobian 奇异"
                                color: studio.jacobianAvailable ? "#cfe2e9" : "#ff9aa4"
                                font.family: "Consolas"
                                font.pixelSize: 12
                            }
                        }

                        GridLayout {
                            Layout.fillWidth: true
                            columns: 2
                            Label { text: "det(J)"; color: "#78929f"; font.pixelSize: 11 }
                            Label {
                                Layout.alignment: Qt.AlignRight
                                text: window.numberText(studio.jacobianDeterminant, 5)
                                color: "#d8e8ed"; font.family: "Consolas"; font.pixelSize: 12
                            }
                            Label { text: "cond(J)"; color: "#78929f"; font.pixelSize: 11 }
                            Label {
                                Layout.alignment: Qt.AlignRight
                                text: window.numberText(studio.jacobianCondition, 4)
                                color: "#d8e8ed"; font.family: "Consolas"; font.pixelSize: 12
                            }
                            Label { text: "FK 迭代次数"; color: "#78929f"; font.pixelSize: 11 }
                            Label {
                                Layout.alignment: Qt.AlignRight
                                text: studio.solveIndicator === "FK" ? String(studio.forwardIterations) : "—"
                                color: "#d8e8ed"; font.family: "Consolas"; font.pixelSize: 12
                            }
                        }

                        Item { Layout.fillHeight: true }
                    }
                }
            }
        }

        Rectangle {
            Layout.fillWidth: true
            implicitHeight: 252
            radius: 10
            color: "#0b171f"
            border.color: "#233b49"

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 12
                spacing: 8

                RowLayout {
                    Layout.fillWidth: true
                    Label {
                        text: "四变量统一控制"
                        color: "#edf7fb"
                        font.pixelSize: 15
                        font.weight: Font.DemiBold
                    }
                    Label {
                        text: "Pitch / Roll 自动调用 IK；θ₁ / θ₂ 自动调用 FK"
                        color: "#77909d"
                        font.pixelSize: 11
                    }
                    Item { Layout.fillWidth: true }
                    Label {
                        text: "内部单位 rad · 界面单位 °"
                        color: "#607985"
                        font.pixelSize: 10
                    }
                }

                GridLayout {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    columns: 2
                    columnSpacing: 10
                    rowSpacing: 8

                    AngleControl {
                        Layout.fillWidth: true
                        title: "平台俯仰"
                        symbol: "Pitch"
                        modelValue: studio.pitchDegrees
                        minimumValue: studio.pitchMinimumDegrees
                        maximumValue: studio.pitchMaximumDegrees
                        syncRevision: studio.revision
                        onUserEdited: function(value) { studio.submitPitch(value) }
                        onInvalidInput: studio.reportInputError("Pitch")
                    }
                    AngleControl {
                        Layout.fillWidth: true
                        title: "平台横滚"
                        symbol: "Roll"
                        modelValue: studio.rollDegrees
                        minimumValue: studio.rollMinimumDegrees
                        maximumValue: studio.rollMaximumDegrees
                        syncRevision: studio.revision
                        onUserEdited: function(value) { studio.submitRoll(value) }
                        onInvalidInput: studio.reportInputError("Roll")
                    }
                    AngleControl {
                        Layout.fillWidth: true
                        title: "驱动关节 1"
                        symbol: "θ₁"
                        modelValue: studio.theta1Degrees
                        minimumValue: studio.theta1MinimumDegrees
                        maximumValue: studio.theta1MaximumDegrees
                        syncRevision: studio.revision
                        onUserEdited: function(value) { studio.submitTheta1(value) }
                        onInvalidInput: studio.reportInputError("θ₁")
                    }
                    AngleControl {
                        Layout.fillWidth: true
                        title: "驱动关节 2"
                        symbol: "θ₂"
                        modelValue: studio.theta2Degrees
                        minimumValue: studio.theta2MinimumDegrees
                        maximumValue: studio.theta2MaximumDegrees
                        syncRevision: studio.revision
                        onUserEdited: function(value) { studio.submitTheta2(value) }
                        onInvalidInput: studio.reportInputError("θ₂")
                    }
                }
            }
        }
    }
}
