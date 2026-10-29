import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: root

    property string title: "支链电机轴"
    property real azimuth: 0
    property real elevation: 0
    signal azimuthEdited(real value)
    signal elevationEdited(real value)

    implicitHeight: 82
    radius: 7
    color: "#0c1922"
    border.color: "#253d4b"

    onAzimuthChanged: if (!azimuthSlider.pressed) azimuthSlider.value = azimuth
    onElevationChanged: if (!elevationSlider.pressed) elevationSlider.value = elevation

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 8
        spacing: 3

        Label {
            text: root.title
            color: "#c9d9df"
            font.pixelSize: 11
            font.weight: Font.DemiBold
        }
        RowLayout {
            Layout.fillWidth: true
            spacing: 6
            Label { text: "方位"; color: "#78929f"; font.pixelSize: 9; Layout.preferredWidth: 28 }
            Slider {
                id: azimuthSlider
                Layout.fillWidth: true
                from: -180
                to: 180
                stepSize: 0.5
                value: root.azimuth
                live: true
                onMoved: root.azimuthEdited(value)
            }
            Label {
                text: Number(azimuthSlider.value).toFixed(1) + "°"
                color: "#a9bec8"; font.family: "Consolas"; font.pixelSize: 9
                Layout.preferredWidth: 48
                horizontalAlignment: Text.AlignRight
            }
        }
        RowLayout {
            Layout.fillWidth: true
            spacing: 6
            Label { text: "仰角"; color: "#78929f"; font.pixelSize: 9; Layout.preferredWidth: 28 }
            Slider {
                id: elevationSlider
                Layout.fillWidth: true
                from: -85
                to: 85
                stepSize: 0.5
                value: root.elevation
                live: true
                onMoved: root.elevationEdited(value)
            }
            Label {
                text: Number(elevationSlider.value).toFixed(1) + "°"
                color: "#a9bec8"; font.family: "Consolas"; font.pixelSize: 9
                Layout.preferredWidth: 48
                horizontalAlignment: Text.AlignRight
            }
        }
    }
}
