import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick3D
import QtQuick3D.Helpers

Rectangle {
    id: root
    color: "#08131a"
    radius: 10
    border.color: "#233b49"
    clip: true

    View3D {
        id: view
        anchors.fill: parent
        anchors.margins: 1

        environment: SceneEnvironment {
            backgroundMode: SceneEnvironment.Color
            clearColor: "#08131a"
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.High
        }

        Node {
            id: cameraOrigin

            PerspectiveCamera {
                id: camera
                z: 880
                clipNear: 1
                clipFar: 4000
                fieldOfView: 42
            }
        }

        OrbitCameraController {
            anchors.fill: parent
            origin: cameraOrigin
            camera: camera
            panEnabled: true
        }

        DirectionalLight {
            eulerRotation.x: -35
            eulerRotation.y: -35
            brightness: 1.1
            castsShadow: true
        }
        DirectionalLight {
            eulerRotation.x: 40
            eulerRotation.y: 150
            brightness: 0.55
        }

        // 求解器采用 z 轴向上；旋转场景，使求解器 z 轴映射为屏幕竖直方向。
        Node {
            id: mechanism
            eulerRotation.x: -90

            // 全局三轴坐标系。
            Rod3D { startPoint: Qt.vector3d(0, 0, 0); endPoint: Qt.vector3d(90, 0, 0); radius: 1.1; rodColor: "#ef6f6c" }
            Rod3D { startPoint: Qt.vector3d(0, 0, 0); endPoint: Qt.vector3d(0, 90, 0); radius: 1.1; rodColor: "#65d08c" }
            Rod3D { startPoint: Qt.vector3d(0, 0, 0); endPoint: Qt.vector3d(0, 0, 90); radius: 1.1; rodColor: "#6b9dff" }

            // 两条电机轴统一用蓝色显示。
            Rod3D { startPoint: studio.axis1Start; endPoint: studio.axis1End; radius: 1.5; rodColor: "#4f83ff"; metallic: true }
            Rod3D { startPoint: studio.axis2Start; endPoint: studio.axis2End; radius: 1.5; rodColor: "#4f83ff"; metallic: true }

            // 支链 1。
            Rod3D { startPoint: studio.m1; endPoint: studio.a1; radius: 3.8; rodColor: "#ffad5a"; metallic: true }
            Rod3D { startPoint: studio.a1; endPoint: studio.b1; radius: 2.8; rodColor: "#ffad5a" }

            // 支链 2。
            Rod3D { startPoint: studio.m2; endPoint: studio.a2; radius: 3.8; rodColor: "#55d6e7"; metallic: true }
            Rod3D { startPoint: studio.a2; endPoint: studio.b2; radius: 2.8; rodColor: "#55d6e7" }

            // 动平台骨架 O-B1-B2。
            Rod3D { startPoint: studio.origin; endPoint: studio.b1; radius: 2.2; rodColor: "#e2edf2" }
            Rod3D { startPoint: studio.origin; endPoint: studio.b2; radius: 2.2; rodColor: "#e2edf2" }
            Rod3D { startPoint: studio.b1; endPoint: studio.b2; radius: 2.6; rodColor: "#e2edf2"; metallic: true }

            Joint3D { position: studio.origin; radius: 8; jointColor: "#f1f8fb" }
            Joint3D { position: studio.m1; radius: 7; jointColor: "#4f83ff" }
            Joint3D { position: studio.a1; radius: 7; jointColor: "#ffad5a" }
            Joint3D { position: studio.b1; radius: 7; jointColor: "#ffad5a" }
            Joint3D { position: studio.m2; radius: 7; jointColor: "#4f83ff" }
            Joint3D { position: studio.a2; radius: 7; jointColor: "#55d6e7" }
            Joint3D { position: studio.b2; radius: 7; jointColor: "#55d6e7" }
        }
    }

    Rectangle {
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.margins: 12
        radius: 6
        color: "#b0061016"
        border.color: "#2a4655"
        width: viewHint.implicitWidth + 20
        height: viewHint.implicitHeight + 14

        Label {
            id: viewHint
            anchors.centerIn: parent
            text: "左键旋转 · 右键平移 · 滚轮缩放"
            color: "#91aab6"
            font.pixelSize: 11
        }
    }

    Rectangle {
        anchors.left: parent.left
        anchors.bottom: parent.bottom
        anchors.margins: 12
        radius: 6
        color: "#b0061016"
        border.color: "#2a4655"
        width: legendRow.implicitWidth + 20
        height: legendRow.implicitHeight + 14

        RowLayout {
            id: legendRow
            anchors.centerIn: parent
            spacing: 12

            Repeater {
                model: [
                    ["#ffad5a", "支链 1"],
                    ["#55d6e7", "支链 2"],
                    ["#4f83ff", "电机轴"],
                    ["#e2edf2", "平台 / 固定 O"]
                ]
                delegate: RowLayout {
                    required property var modelData
                    spacing: 4
                    Rectangle { width: 14; height: 3; radius: 1; color: modelData[0] }
                    Label { text: modelData[1]; color: "#91aab6"; font.pixelSize: 9 }
                }
            }
        }
    }
}
