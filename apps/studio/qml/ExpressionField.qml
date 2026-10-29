import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "Expression.js" as Expression

Rectangle {
    id: root

    property alias text: field.text
    property bool invalid: false
    property string accessibleName: "数值或公式"
    signal textEdited(string value)

    implicitHeight: 34
    radius: 5
    color: "#08131a"
    border.color: invalid ? "#ff6f7d" : (field.activeFocus ? "#62d7d5" : "#304958")

    function step(direction) {
        const info = Expression.plainNumberInfo(field.text)
        if (!info)
            return
        const factor = Math.pow(10, info.decimals)
        const next = (Math.round(info.value * factor) + direction) / factor
        field.text = info.decimals ? next.toFixed(info.decimals) : String(next)
        root.invalid = false
        root.textEdited(field.text)
    }

    RowLayout {
        anchors.fill: parent
        spacing: 0

        TextField {
            id: field
            Layout.fillWidth: true
            Layout.fillHeight: true
            leftPadding: 8
            rightPadding: 5
            selectByMouse: true
            color: root.invalid ? "#ff9aa4" : "#edf7fb"
            font.family: "Consolas"
            font.pixelSize: 11
            inputMethodHints: Qt.ImhNoPredictiveText
            Accessible.name: root.accessibleName
            background: Item {}
            onTextEdited: {
                root.invalid = false
                root.textEdited(text)
            }
            Keys.onPressed: function(event) {
                if (event.key === Qt.Key_Up || event.key === Qt.Key_Down) {
                    if (Expression.plainNumberInfo(text)) {
                        root.step(event.key === Qt.Key_Up ? 1 : -1)
                        event.accepted = true
                    }
                }
            }
            ToolTip.visible: hovered
            ToolTip.text: Expression.plainNumberInfo(text)
                          ? "上下箭头改变最后一位"
                          : "公式输入请直接编辑"
        }

        Column {
            Layout.preferredWidth: 22
            Layout.fillHeight: true

            ToolButton {
                width: parent.width
                height: parent.height / 2
                padding: 0
                text: "▲"
                font.pixelSize: 8
                enabled: Boolean(Expression.plainNumberInfo(field.text))
                onClicked: root.step(1)
                background: Rectangle { color: parent.hovered ? "#1c3440" : "transparent" }
            }
            ToolButton {
                width: parent.width
                height: parent.height / 2
                padding: 0
                text: "▼"
                font.pixelSize: 8
                enabled: Boolean(Expression.plainNumberInfo(field.text))
                onClicked: root.step(-1)
                background: Rectangle { color: parent.hovered ? "#1c3440" : "transparent" }
            }
        }
    }
}
