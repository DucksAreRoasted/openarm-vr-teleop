import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import OpenArm

ColumnLayout {
    id: root
    property bool fieldsEnabled: true
    property bool showSafety: false
    spacing: 10

    GridLayout {
        columns: 2
        columnSpacing: 16
        rowSpacing: 8
        Layout.fillWidth: true
        enabled: root.fieldsEnabled

        Text { text: "左臂 CAN"; color: Theme.muted }
        Text { text: "右臂 CAN"; color: Theme.muted }
        TextField {
            text: console.leftCan
            Layout.fillWidth: true
            onTextEdited: console.leftCan = text
        }
        TextField {
            text: console.rightCan
            Layout.fillWidth: true
            onTextEdited: console.rightCan = text
        }
        Text { text: "波特率"; color: Theme.muted }
        Text { text: "数据波特率"; color: Theme.muted }
        TextField {
            text: console.bitrate
            Layout.fillWidth: true
            onTextEdited: console.bitrate = text
        }
        TextField {
            text: console.dbitrate
            Layout.fillWidth: true
            onTextEdited: console.dbitrate = text
        }
    }

    CheckBox {
        visible: root.showSafety
        text: "确认 CAN 独占、工作区清空、急停可用"
        checked: console.safetyChecked
        enabled: root.fieldsEnabled
        Layout.fillWidth: true
        onToggled: console.safetyChecked = checked
    }
}
