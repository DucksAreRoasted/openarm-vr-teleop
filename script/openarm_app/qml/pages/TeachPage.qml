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

        Text {
            text: "平时零重力托住双臂，可以掰动。前往起手姿势后会停在左右镜像的末帧；点开始录制才松开，然后拖动。记下的是编码器实测关节。"
            color: Theme.muted
            font.pixelSize: 14
            wrapMode: Text.WordWrap
            Layout.fillWidth: true
        }

        Card {
            title: "拖动"
            Layout.fillWidth: true

            CanFields {
                fieldsEnabled: console.canFieldsEnabled
                showSafety: true
                Layout.fillWidth: true
            }

            RowLayout {
                Layout.fillWidth: true
                spacing: 8
                ActionButton {
                    text: console.mode === "zerog" && console.running ? "拖动中" : "启动拖动示教"
                    tone: "primary"
                    enabled: !(console.mode === "zerog" && console.running) && console.canStart
                    onClicked: console.startDragTeach()
                }
                ActionButton {
                    text: "前往起手姿势"
                    tone: "accent"
                    enabled: console.mode === "zerog" && console.running && console.canRecordStart
                    onClicked: console.goTeachHome()
                }
            }
            RowLayout {
                Layout.fillWidth: true
                Text {
                    text: console.mode === "zerog" && console.running
                          ? console.statusText
                          : "勾选急停确认后启动。其他模式正在运行时请先停止并释放。"
                    color: Theme.muted
                    font.pixelSize: 13
                    wrapMode: Text.WordWrap
                    Layout.fillWidth: true
                }
            }
        }

        Card {
            title: "记录轨迹"
            Layout.fillWidth: true

            RowLayout {
                Layout.fillWidth: true
                Text { text: "批次"; color: Theme.ink }
                TextField {
                    text: console.batchName
                    placeholderText: "为这批拖动示教命名"
                    enabled: console.canRecordStart || !console.running
                    selectByMouse: true
                    Layout.fillWidth: true
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
                    text: "结束"
                    enabled: console.canRecordStop
                    Layout.fillWidth: true
                    onClicked: console.record("stop")
                }
            }

            Text {
                text: console.recordHint
                color: Theme.ink
                font.pixelSize: 14
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
            }

            ActionButton {
                text: "去回放"
                tone: "accent"
                enabled: console.canStart
                onClicked: console.openReplay()
            }
        }
    }
}
