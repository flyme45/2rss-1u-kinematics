import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "Expression.js" as Expression

ColumnLayout {
    id: root

    property int branchNumber: 1
    property var expressions: ({})
    property var vectorFields: [
        { key: "axisPoint", label: "电机轴上的点 mᵢ / mm" },
        { key: "axis", label: "电机轴方向 uᵢ" },
        { key: "crankZero", label: "曲柄零位向量 aᵢ₀ / mm" },
        { key: "platformPoint", label: "平台球副向量 bᵢ₀ / mm" }
    ]
    property var scalarFields: [
        { key: "rodLength", label: "连杆长度 Lᵢ / mm", angle: false },
        { key: "motorOffsetDegrees", label: "电机零偏 / °", angle: true },
        { key: "motorLowerDegrees", label: "电机下限 / °", angle: true },
        { key: "motorUpperDegrees", label: "电机上限 / °", angle: true }
    ]

    spacing: 7

    function formatted(value) {
        return Expression.formatNumber(Number(value))
    }

    function loadBranch(branch, storedExpressions) {
        const angleScale = 180 / Math.PI
        const fallback = {
            axisPoint: branch.axisPoint.map(formatted),
            axis: branch.axis.map(formatted),
            crankZero: branch.crankZero.map(formatted),
            platformPoint: branch.platformPoint.map(formatted),
            rodLength: formatted(branch.rodLength),
            motorOffsetDegrees: formatted((branch.motorOffset || 0) * angleScale),
            motorLowerDegrees: formatted(branch.motorLower * angleScale),
            motorUpperDegrees: formatted(branch.motorUpper * angleScale)
        }
        if (storedExpressions) {
            vectorFields.forEach(function(field) {
                if (Array.isArray(storedExpressions[field.key])
                        && storedExpressions[field.key].length === 3)
                    fallback[field.key] = storedExpressions[field.key].map(String)
            })
            scalarFields.forEach(function(field) {
                if (storedExpressions[field.key] !== undefined)
                    fallback[field.key] = String(storedExpressions[field.key])
            })
        }
        expressions = fallback
    }

    function vectorText(field, component) {
        if (!expressions || !Array.isArray(expressions[field]))
            return ""
        const value = expressions[field][component]
        return value === undefined || value === null ? "" : String(value)
    }

    function scalarText(field) {
        return expressions && expressions[field] !== undefined
             ? String(expressions[field]) : ""
    }

    function setVectorText(field, component, value) {
        const next = JSON.parse(JSON.stringify(expressions || {}))
        if (!Array.isArray(next[field])) next[field] = ["", "", ""]
        next[field][component] = value
        expressions = next
    }

    function setScalarText(field, value) {
        const next = JSON.parse(JSON.stringify(expressions || {}))
        next[field] = value
        expressions = next
    }

    function evaluateField(field, text, angleField, componentName) {
        try {
            return Expression.evaluate(text, angleField)
        } catch (error) {
            const suffix = componentName ? "的 " + componentName + " 分量" : ""
            throw new Error("支链 " + branchNumber + "：" + field.label + suffix
                            + "公式错误：" + error.message)
        }
    }

    function numericBranch(sharedOrigin) {
        const result = {}
        vectorFields.forEach(function(field) {
            result[field.key] = [0, 1, 2].map(function(component) {
                return evaluateField(field, vectorText(field.key, component), false,
                                     ["x", "y", "z"][component])
            })
        })
        result.platformOffset = sharedOrigin.slice()
        scalarFields.forEach(function(field) {
            const value = evaluateField(field, scalarText(field.key), field.angle, "")
            if (field.key === "rodLength") result.rodLength = value
            else if (field.key === "motorOffsetDegrees") result.motorOffset = value * Math.PI / 180
            else if (field.key === "motorLowerDegrees") result.motorLower = value * Math.PI / 180
            else if (field.key === "motorUpperDegrees") result.motorUpper = value * Math.PI / 180
        })
        return result
    }

    function expressionBranch(sharedOriginText) {
        const result = JSON.parse(JSON.stringify(expressions || {}))
        result.platformOffset = sharedOriginText.slice()
        return result
    }

    Rectangle {
        Layout.fillWidth: true
        implicitHeight: 34
        radius: 6
        color: "#10222c"
        Label {
            anchors.fill: parent
            anchors.leftMargin: 10
            verticalAlignment: Text.AlignVCenter
            text: "支链 " + root.branchNumber
            color: root.branchNumber === 1 ? "#ffbd75" : "#71ddea"
            font.pixelSize: 13
            font.weight: Font.DemiBold
        }
    }

    RowLayout {
        Layout.fillWidth: true
        spacing: 6
        Label { Layout.preferredWidth: 178; text: "参数"; color: "#718b98"; font.pixelSize: 10 }
        Repeater {
            model: ["x", "y", "z"]
            delegate: Label {
                required property string modelData
                Layout.fillWidth: true
                text: modelData
                horizontalAlignment: Text.AlignHCenter
                color: "#718b98"
                font.pixelSize: 10
            }
        }
    }

    Repeater {
        model: root.vectorFields
        delegate: RowLayout {
            id: vectorRow
            required property var modelData
            Layout.fillWidth: true
            spacing: 6

            Label {
                Layout.preferredWidth: 178
                text: vectorRow.modelData.label
                color: "#b7c9d1"
                font.pixelSize: 11
                wrapMode: Text.WordWrap
            }
            Repeater {
                model: 3
                delegate: ExpressionField {
                    id: vectorInput
                    property int component: index
                    Layout.fillWidth: true
                    accessibleName: "支链 " + root.branchNumber + "，"
                                    + vectorRow.modelData.label + "，"
                                    + ["x", "y", "z"][component] + " 分量"
                    text: root.vectorText(vectorRow.modelData.key, component)
                    onTextEdited: function(value) {
                        root.setVectorText(vectorRow.modelData.key, component, value)
                    }
                }
            }
        }
    }

    GridLayout {
        Layout.fillWidth: true
        columns: 2
        columnSpacing: 10
        rowSpacing: 7

        Repeater {
            model: root.scalarFields
            delegate: ColumnLayout {
                id: scalarColumn
                required property var modelData
                Layout.fillWidth: true
                spacing: 3

                Label {
                    text: scalarColumn.modelData.label
                    color: "#9eb3bd"
                    font.pixelSize: 10
                }
                ExpressionField {
                    Layout.fillWidth: true
                    accessibleName: "支链 " + root.branchNumber + "，" + scalarColumn.modelData.label
                    text: root.scalarText(scalarColumn.modelData.key)
                    onTextEdited: function(value) {
                        root.setScalarText(scalarColumn.modelData.key, value)
                    }
                }
            }
        }
    }
}
