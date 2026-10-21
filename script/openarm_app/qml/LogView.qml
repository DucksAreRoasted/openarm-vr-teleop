import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import OpenArm
import "components"

ColumnLayout {
    id: root
    spacing: 8

    RowLayout {
        Layout.fillWidth: true
        spacing: 8

        ComboBox {
            id: sourceBox
            model: ["全部来源", "控制器", "标定", "录制", "控制台"]
            onActivated: console.setLogSource(currentText)
        }
        TextField {
            placeholderText: "过滤日志内容…"
            Layout.fillWidth: true
            selectByMouse: true
            onTextEdited: console.setLogFilter(text)
        }
        CheckBox {
            text: "正则"
            onToggled: console.setLogRegex(checked)
        }
        ActionButton {
            text: console.logPaused ? "继续显示" : "暂停显示"
            onClicked: console.setLogPaused(!console.logPaused)
        }
        ActionButton {
            text: console.logUnread > 0 ? ("↓ 新日志 " + console.logUnread + " 条") : "↓ 最新"
            onClicked: {
                logList.stick = true
                console.followLog()
                logList.positionViewAtEnd()
            }
        }
        ActionButton { text: "清空"; onClicked: console.clearLog() }
    }

    Text {
        visible: console.logFilterError !== ""
        text: console.logFilterError
        color: "#b45309"
        font.pixelSize: 13
        Layout.fillWidth: true
    }

    RowLayout {
        Layout.fillWidth: true
        spacing: 8
        Text { text: "查找"; color: Theme.logMuted }
        TextField {
            id: findField
            placeholderText: "查找并高亮，Enter 下一处"
            Layout.fillWidth: true
            selectByMouse: true
            onTextEdited: console.setLogFind(text)
            onAccepted: console.findNextLog()
        }
        Repeater {
            model: ["DEBUG", "INFO", "WARN", "ERROR", "FATAL"]
            CheckBox {
                required property string modelData
                text: modelData
                Component.onCompleted: checked = console.logLevelEnabled(modelData)
                onToggled: console.setLogLevelEnabled(modelData, checked)
            }
        }
    }

    ListView {
        id: logList
        property bool stick: true
        Layout.fillWidth: true
        Layout.fillHeight: true
        clip: true
        model: console.logLines
        boundsBehavior: Flickable.StopAtBounds
        onMovementEnded: stick = atYEnd
        onCountChanged: if (stick) Qt.callLater(positionViewAtEnd)

        delegate: Rectangle {
            required property string display
            required property string level
            required property int index
            width: logList.width
            height: line.implicitHeight + 4
            color: level === "FATAL" ? "#c62828" : "transparent"

            Text {
                id: line
                width: parent.width - 8
                x: 4
                text: display
                color: {
                    if (level === "DEBUG") return "#829ab1"
                    if (level === "WARN") return "#b45309"
                    if (level === "ERROR") return "#b91c1c"
                    if (level === "FATAL") return "#ffffff"
                    return Theme.logInk
                }
                font.family: "JetBrains Mono, Cascadia Code, DejaVu Sans Mono"
                font.pixelSize: 13
                font.bold: level === "FATAL"
                wrapMode: Text.NoWrap
                elide: Text.ElideRight
            }
        }

        ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }
    }
}
