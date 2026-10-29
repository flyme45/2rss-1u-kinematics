import QtQuick
import QtQuick3D

Model {
    property real radius: 7
    property color jointColor: "#f0f7fa"

    source: "#Sphere"
    scale: Qt.vector3d(radius / 50.0, radius / 50.0, radius / 50.0)
    materials: PrincipledMaterial {
        baseColor: jointColor
        metalness: 0.25
        roughness: 0.32
    }
}
