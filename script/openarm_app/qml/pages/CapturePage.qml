import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import OpenArm
import "../components"

Flickable {
    contentWidth: width
    contentHeight: column.implicitHeight
    clip: true
    boundsBehavior: Flickable.StopAtBounds

    ColumnLayout {
        id: column
        width: parent.width
        spacing: 14

        Card {
            title: "数据采集"
            Layout.fillWidth: true

            RowLayout {
                Layout.fillWidth: true
                Text {
                    text: "开始 → 结束 → 标注"
                    color: Theme.ink
                    font.pixelSize: 14
                }
                Item { Layout.fillWidth: true }
                Pill {
                    text: console.recordLabel
                    fg: {
                        if (console.recordState === "recording") return "#b91c1c"
                        if (console.recordState === "pending") return "#92400e"
                        if (console.recordState === "ready") return Theme.success
                        return Theme.muted
                    }
                    bg: {
                        if (console.recordState === "recording") return "#fee2e2"
                        if (console.recordState === "pending") return "#fef3c7"
                        if (console.recordState === "ready") return "#dcfce7"
                        return "#e8eef4"
                    }
                }
            }

            RowLayout {
                Layout.fillWidth: true
                Text { text: "批次"; color: Theme.ink }
                TextField {
                    text: console.batchName
                    placeholderText: "为这批动作命名"
                    enabled: console.batchEditable
                    Layout.fillWidth: true
                    selectByMouse: true
                    onTextEdited: console.batchName = text
                }
            }

            RowLayout {
                Layout.fillWidth: true
                spacing: 8
                ActionButton {
                    text: "开始录制"
                    tone: "primary"
                    enabled: console.canRecordStart
                    Layout.fillWidth: true
                    onClicked: console.record("start")
                }
                ActionButton {
                    text: "结束本条"
                    enabled: console.canRecordStop
                    Layout.fillWidth: true
                    onClicked: console.record("stop")
                }
            }

            RowLayout {
                Layout.fillWidth: true
                spacing: 8
                ActionButton {
                    text: "标记成功"
                    tone: "success"
                    enabled: console.canLabel
                    Layout.fillWidth: true
                    onClicked: console.record("success")
                }
                ActionButton {
                    text: "标记失败"
                    tone: "danger"
                    enabled: console.canLabel
                    Layout.fillWidth: true
                    onClicked: console.record("fail")
                }
                ActionButton {
                    text: console.redoLabel
                    enabled: console.canRedo
                    Layout.fillWidth: true
                    onClicked: console.redo()
                }
            }

            Text {
                text: console.recordHint
                color: Theme.muted
                font.pixelSize: 14
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
            }

            RowLayout {
                Layout.fillWidth: true
                ActionButton {
                    text: "去数据页"
                    tone: "accent"
                    onClicked: console.setPage("data")
                }
                ActionButton {
                    text: "动作回放…"
                    enabled: console.canStart
                    onClicked: console.openReplay()
                }
                Item { Layout.fillWidth: true }
            }
        }

        Card {
            title: "手柄"
            Layout.fillWidth: true

            GridLayout {
                columns: 2
                columnSpacing: 24
                rowSpacing: 6
                Layout.fillWidth: true

                Text { text: "左摇杆"; color: Theme.accent; font.bold: true }
                Text { text: "右摇杆 · 停录后"; color: Theme.accent; font.bold: true }
                Text { text: "短按  开始 / 结束录制"; color: Theme.ink; wrapMode: Text.WordWrap; Layout.fillWidth: true }
                Text { text: "短按  标记成功"; color: Theme.ink; wrapMode: Text.WordWrap; Layout.fillWidth: true }
                Text { text: "按住 1.5 秒  丢弃当前条"; color: Theme.ink; wrapMode: Text.WordWrap; Layout.fillWidth: true }
                Text { text: "按住 1.5 秒  标记失败"; color: Theme.ink; wrapMode: Text.WordWrap; Layout.fillWidth: true }
            }

            Text {
                text: "按住时约每 0.3 秒轻振一下，满 1.5 秒即执行。丢弃后 10 秒内可撤销。A/B、X/Y 控制夹爪。"
                color: Theme.muted
                font.pixelSize: 13
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
            }
        }
    }
}
