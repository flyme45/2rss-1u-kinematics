import QtQuick
import QtQuick3D

Node {
    id: root

    property vector3d startPoint: Qt.vector3d(0, 0, 0)
    property vector3d endPoint: Qt.vector3d(0, 100, 0)
    property real radius: 2.5
    property color rodColor: "#ffffff"
    property bool metallic: false

    function subtract(a, b) {
        return Qt.vector3d(a.x - b.x, a.y - b.y, a.z - b.z)
    }

    function midpoint(a, b) {
        return Qt.vector3d((a.x + b.x) * 0.5,
                           (a.y + b.y) * 0.5,
                           (a.z + b.z) * 0.5)
    }

    function magnitude(v) {
        return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
    }

    function fromY(direction) {
        const length = magnitude(direction)
        if (length < 1e-8)
            return Qt.quaternion(1, 0, 0, 0)
        const dx = direction.x / length
        const dy = direction.y / length
        const dz = direction.z / length
        if (dy < -0.999999)
            return Qt.quaternion(0, 1, 0, 0)
        const cx = dz
        const cy = 0
        const cz = -dx
        const w = 1 + dy
        const qLength = Math.sqrt(w * w + cx * cx + cy * cy + cz * cz)
        return Qt.quaternion(w / qLength, cx / qLength, cy / qLength, cz / qLength)
    }

    readonly property vector3d difference: subtract(endPoint, startPoint)
    readonly property real rodLength: magnitude(difference)

    position: midpoint(startPoint, endPoint)
    rotation: fromY(difference)
    visible: rodLength > 1e-6

    Model {
        source: "#Cylinder"
        scale: Qt.vector3d(root.radius / 50.0,
                           root.rodLength / 100.0,
                           root.radius / 50.0)
        materials: PrincipledMaterial {
            baseColor: root.rodColor
            metalness: root.metallic ? 0.55 : 0.05
            roughness: root.metallic ? 0.28 : 0.5
        }
    }
}
