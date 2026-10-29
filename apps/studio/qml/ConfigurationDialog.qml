import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs as QuickDialogs
import QtQuick.Layouts
import "Expression.js" as Expression

Dialog {
    id: root

    property real availableWidth: 1200
    property real availableHeight: 800
    property bool newCopy: true
    property var originExpressions: ["0", "0", "0"]
    property string pendingExportJson: ""

    width: Math.min(1120, availableWidth - 36)
    height: Math.min(820, availableHeight - 36)
    x: Math.max(18, (availableWidth - width) / 2)
    y: Math.max(18, (availableHeight - height) / 2)
    modal: true
    focus: true
    padding: 0
    closePolicy: Popup.CloseOnEscape

    function setOrigin(component, value) {
        const next = originExpressions.slice()
        next[component] = value
        originExpressions = next
    }

    function loadDocument(json, imported) {
        try {
            const document = JSON.parse(json)
            if (!Array.isArray(document.branches) || document.branches.length !== 2)
                throw new Error("构型必须包含两条 RSS 支链")
            const expressions = Array.isArray(document.expressions)
                              && document.expressions.length === 2
                              ? document.expressions : [null, null]
            nameField.text = String(document.name || "导入构型")
            if (!imported && !studio.currentConfigurationIsUser)
                nameField.text += " 副本"
            const originSource = expressions[0]
                              && Array.isArray(expressions[0].platformOffset)
                              && expressions[0].platformOffset.length === 3
                              ? expressions[0].platformOffset
                              : document.branches[0].platformOffset.map(Expression.formatNumber)
            originExpressions = originSource.map(String)
            branchOne.loadBranch(document.branches[0], expressions[0])
            branchTwo.loadBranch(document.branches[1], expressions[1])
            newCopy = imported || !studio.currentConfigurationIsUser
            statusLabel.text = imported
                           ? "JSON 已载入。公共万向副中心 O 由两条支链共用，请检查后保存。"
                           : "可以直接输入 sin、cos、sqrt、pi/π 等公式；保存前会统一校验。"
            statusLabel.color = "#8de0c7"
        } catch (error) {
            statusLabel.text = "构型内容无法读取：" + error.message
            statusLabel.color = "#ff9aa4"
        }
    }

    function openCurrent() {
        loadDocument(studio.currentConfigurationJson, false)
        open()
        nameField.forceActiveFocus()
        nameField.selectAll()
    }

    function sharedOriginNumeric() {
        return [0, 1, 2].map(function(component) {
            try {
                return Expression.evaluate(originExpressions[component], false)
            } catch (error) {
                throw new Error("公共万向副中心 O 的 " + ["x", "y", "z"][component]
                                + " 分量公式错误：" + error.message)
            }
        })
    }

    function buildDocument() {
        const name = nameField.text.trim()
        if (!name)
            throw new Error("请输入构型名称")
        const origin = sharedOriginNumeric()
        const branches = [branchOne.numericBranch(origin), branchTwo.numericBranch(origin)]
        const expressions = [
            branchOne.expressionBranch(originExpressions),
            branchTwo.expressionBranch(originExpressions)
        ]
        return { version: 2, name: name, branches: branches, expressions: expressions }
    }

    function saveCurrent() {
        try {
            const document = buildDocument()
            if (studio.saveConfigurationJson(JSON.stringify(document, null, 2), !newCopy))
                close()
            else {
                statusLabel.text = studio.statusText
                statusLabel.color = "#ff9aa4"
            }
        } catch (error) {
            statusLabel.text = error.message
            statusLabel.color = "#ff9aa4"
        }
    }

    background: Rectangle {
        radius: 10
        color: "#0a161e"
        border.color: "#2a4655"
    }

    contentItem: ColumnLayout {
        spacing: 0

        Rectangle {
            Layout.fillWidth: true
            implicitHeight: 64
            color: "#0d1c25"
            border.color: "#243c49"

            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: 18
                anchors.rightMargin: 12
                spacing: 12
                ColumnLayout {
                    spacing: 1
                    Label { text: "CONFIGURATION"; color: "#62d7d5"; font.pixelSize: 9; font.bold: true; font.letterSpacing: 2 }
                    Label { text: "添加或编辑构型参数"; color: "#edf7fb"; font.pixelSize: 18; font.weight: Font.DemiBold }
                }
                Item { Layout.fillWidth: true }
                ToolButton { text: "×"; font.pixelSize: 20; onClicked: root.close() }
            }
        }

        ScrollView {
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true

            ColumnLayout {
                width: parent.width
                spacing: 12

                Rectangle {
                    Layout.fillWidth: true
                    implicitHeight: guideText.implicitHeight + 22
                    radius: 7
                    color: "#10242a"
                    border.color: "#285059"
                    Label {
                        id: guideText
                        anchors.fill: parent
                        anchors.margins: 11
                        text: "填写顺序：① 命名构型；② 输入两条支链的零位几何；③ 检查并保存。\n"
                              + "mᵢ 通常取电机输出轴与曲柄的交点；uᵢ 按右手定则给出。公共 O 只填写一次。"
                        color: "#b9d5d8"
                        font.pixelSize: 11
                        wrapMode: Text.WordWrap
                    }
                }

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 9
                    Label { text: "构型名称"; color: "#b8cbd3"; font.pixelSize: 11 }
                    TextField {
                        id: nameField
                        Layout.fillWidth: true
                        maximumLength: 60
                        placeholderText: "例如：样机标定参数 01"
                        selectByMouse: true
                    }
                    Button {
                        text: "新建副本"
                        onClicked: {
                            root.newCopy = true
                            if (!nameField.text.endsWith(" 副本"))
                                nameField.text += " 副本"
                            statusLabel.text = "保存后会新增到构型列表，不覆盖当前保存项。"
                            statusLabel.color = "#8de0c7"
                        }
                    }
                }

                Rectangle {
                    Layout.fillWidth: true
                    implicitHeight: expressionHelp.implicitHeight + 20
                    radius: 7
                    color: "#101e2b"
                    border.color: "#294154"
                    Label {
                        id: expressionHelp
                        anchors.fill: parent
                        anchors.margins: 10
                        text: "公式输入示例：-50*cos(15°)　50*sin(pi/12)　sqrt(2)/2\n"
                              + "支持 sin cos tan sqrt abs pi/π 和 + - * / ^。三角函数中的普通数字默认按度；含 pi/π 时按弧度。"
                        color: "#9fc1d4"
                        font.family: "Consolas"
                        font.pixelSize: 10
                        wrapMode: Text.WordWrap
                    }
                }

                ColumnLayout {
                    Layout.fillWidth: true
                    spacing: 4
                    RowLayout {
                        Layout.fillWidth: true
                        Label { text: "公共万向副中心 O / mm"; color: "#b7cbd3"; font.pixelSize: 11 }
                        Label { text: "两条支链共用"; color: "#62d7d5"; font.pixelSize: 9 }
                        Item { Layout.fillWidth: true }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 8
                        Repeater {
                            model: ["x", "y", "z"]
                            delegate: ColumnLayout {
                                required property string modelData
                                property int component: index
                                Layout.fillWidth: true
                                spacing: 2
                                Label { text: parent.modelData; color: "#718b98"; font.pixelSize: 9 }
                                ExpressionField {
                                    Layout.fillWidth: true
                                    accessibleName: "公共万向副中心 O，" + parent.modelData + " 分量"
                                    text: String(root.originExpressions[parent.component] || "0")
                                    onTextEdited: function(value) { root.setOrigin(parent.component, value) }
                                }
                            }
                        }
                    }
                }

                TabBar {
                    id: branchTabs
                    Layout.fillWidth: true
                    TabButton { text: "支链 1" }
                    TabButton { text: "支链 2" }
                }
                StackLayout {
                    Layout.fillWidth: true
                    currentIndex: branchTabs.currentIndex
                    BranchEditor { id: branchOne; branchNumber: 1 }
                    BranchEditor { id: branchTwo; branchNumber: 2 }
                }

                Label {
                    id: statusLabel
                    Layout.fillWidth: true
                    text: ""
                    color: "#8de0c7"
                    font.pixelSize: 11
                    wrapMode: Text.WordWrap
                }
            }
        }

        Rectangle {
            Layout.fillWidth: true
            implicitHeight: 60
            color: "#0d1c25"
            border.color: "#243c49"

            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: 12
                anchors.rightMargin: 12
                spacing: 8
                Button { text: "导入 JSON"; onClicked: importDialog.open() }
                Button {
                    text: "导出当前"
                    onClicked: {
                        try {
                            root.pendingExportJson = JSON.stringify(root.buildDocument(), null, 2)
                            exportDialog.open()
                        } catch (error) {
                            statusLabel.text = error.message
                            statusLabel.color = "#ff9aa4"
                        }
                    }
                }
                Button {
                    text: "删除保存项"
                    enabled: studio.currentConfigurationIsUser && !root.newCopy
                    onClicked: deleteConfirmation.open()
                }
                Item { Layout.fillWidth: true }
                Button { text: "取消"; onClicked: root.close() }
                Button {
                    text: root.newCopy ? "检查并新增" : "检查并保存"
                    highlighted: true
                    onClicked: root.saveCurrent()
                }
            }
        }
    }

    QuickDialogs.FileDialog {
        id: importDialog
        title: "导入 2-RSS-1U 构型"
        fileMode: QuickDialogs.FileDialog.OpenFile
        nameFilters: ["JSON 构型文件 (*.json)"]
        onAccepted: {
            const content = studio.loadConfigurationFile(selectedFile)
            if (content)
                root.loadDocument(content, true)
            else {
                statusLabel.text = studio.statusText
                statusLabel.color = "#ff9aa4"
            }
        }
    }

    QuickDialogs.FileDialog {
        id: exportDialog
        title: "导出 2-RSS-1U 构型"
        fileMode: QuickDialogs.FileDialog.SaveFile
        defaultSuffix: "json"
        nameFilters: ["JSON 构型文件 (*.json)"]
        onAccepted: {
            if (studio.writeConfigurationFile(selectedFile, root.pendingExportJson)) {
                statusLabel.text = studio.statusText
                statusLabel.color = "#8de0c7"
            } else {
                statusLabel.text = studio.statusText
                statusLabel.color = "#ff9aa4"
            }
        }
    }

    Dialog {
        id: deleteConfirmation
        title: "删除构型"
        modal: true
        standardButtons: Dialog.Yes | Dialog.No
        Label { text: "确定删除已保存构型“" + studio.currentConfigurationName + "”吗？" }
        onAccepted: {
            if (studio.deleteCurrentConfiguration())
                root.close()
        }
    }
}
