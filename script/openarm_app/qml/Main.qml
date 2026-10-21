import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import OpenArm
import "components"
import "pages"

ApplicationWindow {
    id: window
    width: 1280
    height: 900
    minimumWidth: 960
    minimumHeight: 640
    visible: true
    title: "OpenArm 遥操作"
    color: Theme.page
    font.pixelSize: 15
    font.family: "Noto Sans CJK SC"

    property int pageIndex: {
        if (console.page === "capture") return 1
        if (console.page === "gravity") return 2
        if (console.page === "data") return 3
        if (console.page === "replay") return 4
        if (console.page === "motors") return 5
        if (console.page === "teach") return 6
        return 0
    }

    Component.onCompleted: console.setViewportHeight(height)
    onHeightChanged: console.setViewportHeight(height)

    Shortcut {
        sequences: ["Ctrl+J"]
        onActivated: console.logExpanded = !console.logExpanded
    }
    Shortcut {
        sequences: ["Ctrl+F"]
        onActivated: {
            console.logExpanded = true
        }
    }

    onClosing: function(close) {
        if (!console.prepareClose())
            close.accepted = false
    }

    Connections {
        target: console
        function onCloseRequested() { window.close() }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 16
        spacing: 12

        RowLayout {
            Layout.fillWidth: true
            spacing: 12
            ColumnLayout {
                spacing: 2
                Text {
                    text: "OpenArm"
                    color: Theme.ink
                    font.pixelSize: 26
                    font.bold: true
                }
                Text {
                    text: "遥操作控制台"
                    color: Theme.muted
                    font.pixelSize: 13
                }
            }
            Item { Layout.fillWidth: true }
            Pill { text: console.modeLabel; fg: Theme.accent; bg: Theme.accentBg }
            Pill {
                text: console.recordLabel
                visible: console.page !== "capture"
                fg: console.recordState === "recording" ? "#b91c1c" : Theme.muted
                bg: console.recordState === "recording" ? "#fee2e2" : "#e8eef4"
            }
            Pill {
                text: console.statusText
                fg: {
                    if (console.statusTone === "error") return "#9b1c1c"
                    if (console.statusTone === "running") return Theme.success
                    if (console.statusTone === "busy") return "#92400e"
                    return Theme.ink
                }
                bg: {
                    if (console.statusTone === "error") return "#fee2e2"
                    if (console.statusTone === "running") return "#dcfce7"
                    if (console.statusTone === "busy") return "#fef3c7"
                    return "#e8eef4"
                }
            }
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 8
            ActionButton {
                text: "启动"
                tone: "primary"
                enabled: console.canStart
                implicitWidth: 120
                onClicked: console.start()
            }
            ActionButton {
                text: "停止并释放"
                tone: "danger"
                enabled: console.canStop
                implicitWidth: 140
                onClicked: console.stop()
            }
            Item { Layout.fillWidth: true }
            Text {
                text: console.questUrl
                color: Theme.muted
                font.pixelSize: 13
            }
        }

        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 12

            Rectangle {
                Layout.preferredWidth: 210
                Layout.fillHeight: true
                radius: Theme.radius
                color: Theme.card
                border.color: Theme.line

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 4
                    NavButton {
                        label: "遥操作"
                        selected: console.page === "teleop"
                        Layout.fillWidth: true
                        onClicked: console.setPage("teleop")
                    }
                    NavButton {
                        label: "拖动示教"
                        selected: console.page === "teach"
                        marked: console.mode === "zerog" && (console.running || console.recordState === "recording")
                        Layout.fillWidth: true
                        onClicked: console.setPage("teach")
                    }
                    NavButton {
                        label: "VR录制"
                        selected: console.page === "capture"
                        marked: console.recordState === "recording" || console.recordState === "pending"
                        Layout.fillWidth: true
                        onClicked: console.setPage("capture")
                    }
                    NavButton {
                        label: "重力"
                        selected: console.page === "gravity"
                        Layout.fillWidth: true
                        onClicked: console.setPage("gravity")
                    }
                    NavButton {
                        label: "遥操数据管理"
                        selected: console.page === "data"
                        Layout.fillWidth: true
                        onClicked: console.setPage("data")
                    }
                    NavButton {
                        label: "遥操数据回放"
                        selected: console.page === "replay"
                        marked: console.replay.running
                        Layout.fillWidth: true
                        onClicked: console.setPage("replay")
                    }
                    NavButton {
                        label: "电机参数"
                        selected: console.page === "motors"
                        Layout.fillWidth: true
                        onClicked: console.setPage("motors")
                    }
                    Item { Layout.fillHeight: true }
                }
            }

            StackLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                currentIndex: window.pageIndex
                TeleopPage {}
                CapturePage {}
                GravityPage {}
                DataPage {}
                ReplayPage {}
                MotorPage {}
                TeachPage {}
            }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 10
            radius: 4
            color: logDrag.pressed ? Theme.primary : "#d5dee8"

            MouseArea {
                id: logDrag
                anchors.fill: parent
                hoverEnabled: true
                cursorShape: Qt.SizeVerCursor
                property real origin: 0
                property real startY: 0
                onPressed: function(mouse) {
                    if (!console.logExpanded)
                        console.logExpanded = true
                    origin = console.logHeight
                    startY = mapToItem(null, mouse.x, mouse.y).y
                }
                onPositionChanged: function(mouse) {
                    if (!pressed)
                        return
                    var y = mapToItem(null, mouse.x, mouse.y).y
                    console.setLogHeight(origin - (y - startY))
                }
            }
        }

        Rectangle {
            id: logDock
            Layout.fillWidth: true
            Layout.preferredHeight: console.logExpanded ? Math.min(console.logHeight, Math.max(120, window.height * 0.6)) : 44
            radius: 12
            color: Theme.logBg
            border.color: Theme.line
            border.width: 1
            clip: true

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 8
                spacing: 6

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    RowLayout {
                        spacing: 8
                        Text {
                            text: (console.logExpanded ? "▾  " : "▸  ") + "实时日志"
                            color: console.logErrors > 0 && !console.logExpanded ? "#b91c1c" : Theme.logInk
                            font.bold: true
                            font.pixelSize: 14
                        }
                        Text {
                            text: "⚠ " + console.logWarnings + "    ✖ " + console.logErrors
                            color: "#b45309"
                            font.pixelSize: 13
                        }
                        TapHandler { onTapped: console.logExpanded = !console.logExpanded }
                    }
                    Item { Layout.fillWidth: true }
                    Text {
                        text: console.logDetached ? "已在独立窗口" : "独立窗口 ↗"
                        color: Theme.accent
                        font.pixelSize: 13
                        TapHandler { onTapped: console.setLogDetached(!console.logDetached) }
                    }
                }

                LogView {
                    visible: console.logExpanded && !console.logDetached
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                }
            }
        }
    }

    LogWindow { }
}
