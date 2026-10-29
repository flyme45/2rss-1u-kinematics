import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: root

    property string title: "Angle"
    property string symbol: "q"
    property real modelValue: 0
    property real minimumValue: -1
    property real maximumValue: 1
    property int syncRevision: 0
    property bool inputInvalid: false
    signal userEdited(real value)
    signal invalidInput()

    implicitHeight: 92
    radius: 8
    color: "#101c26"
    border.color: inputInvalid ? "#ff6f7d" : "#263b49"
    border.width: 1

    function display(value) {
        return Math.abs(value) < 0.00005 ? "0.0000" : Number(value).toFixed(4)
    }

    function synchronize(force) {
        if (!angleSlider.pressed)
            angleSlider.value = root.modelValue
        if (force && !angleField.activeFocus)
            root.inputInvalid = false
        // 用户尚未提交或输入非法内容时，不用自动刷新覆盖文本。
        if (!angleField.activeFocus && (!root.inputInvalid || force)) {
            angleField.text = root.display(root.modelValue)
            angleField.dirty = false
        }
    }

    onModelValueChanged: synchronize(false)
    onSyncRevisionChanged: synchronize(true)
    Component.onCompleted: synchronize(true)

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 5

        RowLayout {
            Layout.fillWidth: true

            Label {
                text: root.title
                color: "#edf6fa"
                font.pixelSize: 13
                font.weight: Font.DemiBold
            }
            Label {
                text: root.symbol
                color: "#62d7d5"
                font.pixelSize: 12
            }
            Item { Layout.fillWidth: true }
            Label {
                text: "[" + root.display(root.minimumValue) + ", "
                      + root.display(root.maximumValue) + "]°"
                color: "#6f8997"
                font.pixelSize: 10
            }
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 10

            Slider {
                id: angleSlider
                Layout.fillWidth: true
                from: root.minimumValue
                to: root.maximumValue
                stepSize: Math.max((to - from) / 2000.0, 0.0001)
                live: true

                onMoved: root.userEdited(value)
                onPressedChanged: {
                    if (!pressed)
                        value = root.modelValue
                }

                ToolTip.visible: pressed
                ToolTip.text: root.display(value) + "°"
            }

            TextField {
                id: angleField
                property bool dirty: false

                Layout.preferredWidth: 96
                horizontalAlignment: TextInput.AlignRight
                selectByMouse: true
                color: root.inputInvalid ? "#ff8b96" : "#eff8fb"
                placeholderText: "角度"
                inputMethodHints: Qt.ImhFormattedNumbersOnly
                validator: DoubleValidator {
                    bottom: root.minimumValue
                    top: root.maximumValue
                    decimals: 6
                    notation: DoubleValidator.StandardNotation
                }

                background: Rectangle {
                    radius: 5
                    color: "#08131a"
                    border.color: root.inputInvalid ? "#ff6f7d"
                                                    : (angleField.activeFocus ? "#62d7d5" : "#304958")
                }

                onTextEdited: {
                    dirty = true
                    root.inputInvalid = false
                }

                function commit() {
                    if (!dirty)
                        return
                    if (!acceptableInput || text.trim().length === 0) {
                        root.inputInvalid = true
                        root.invalidInput()
                        return
                    }
                    const candidate = Number(text)
                    if (!Number.isFinite(candidate)) {
                        root.inputInvalid = true
                        root.invalidInput()
                        return
                    }
                    dirty = false
                    root.inputInvalid = false
                    root.userEdited(candidate)
                }

                onAccepted: {
                    commit()
                    if (!root.inputInvalid)
                        focus = false
                }
                onEditingFinished: commit()
            }

            Label {
                text: "°"
                color: "#8da4af"
            }
        }
    }
}
