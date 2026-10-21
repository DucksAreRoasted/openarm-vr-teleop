import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import OpenArm
import "../components"

Flickable {
    id: page
    objectName: "replayPage"
    contentWidth: width
    contentHeight: column.implicitHeight
    clip: true
    boundsBehavior: Flickable.StopAtBounds

    ColumnLayout {
        id: column
        width: parent.width
        spacing: 14

        Text {
            text: "选择动作，先到达起手位置。到位后停在左右镜像的起手末帧；开始回放时才走到这条录像的第一帧。"
            color: Theme.muted
            font.pixelSize: 14
            wrapMode: Text.WordWrap
            Layout.fillWidth: true
        }

        Card {
            title: "选择动作"
            Layout.fillWidth: true

            RowLayout {
                Layout.fillWidth: true
                spacing: 8
                TextField {
                    text: console.replay.source
                    placeholderText: "片段目录或双臂 CSV"
                    enabled: console.replay.canEditSetup
                    selectByMouse: true
                    Layout.fillWidth: true
                    onTextEdited: console.replay.setSource(text)
                }
                ActionButton {
                    text: "选片段目录"
                    enabled: console.replay.canEditSetup
                    onClicked: console.replay.chooseFolder()
                }
                ActionButton {
                    text: "选 CSV"
                    enabled: console.replay.canEditSetup
                    onClicked: console.replay.chooseFile()
                }
            }
        }

        Card {
            title: "回放设置"
            Layout.fillWidth: true

            GridLayout {
                columns: 2
                columnSpacing: 16
                rowSpacing: 8
                Layout.fillWidth: true

                Text { text: "运行环境"; color: Theme.muted }
                ComboBox {
                    enabled: console.replay.canEditSetup
                    model: ["仿真预览（按录制时间）", "真机回放"]
                    currentIndex: console.replay.modeIndex
                    Layout.fillWidth: true
                    onActivated: console.replay.setModeIndex(currentIndex)
                }
                Text { text: "播放速度"; color: Theme.muted }
                ComboBox {
                    enabled: console.replay.canEditSetup
                    model: ["0.25 倍", "0.5 倍", "1 倍"]
                    currentIndex: console.replay.speedIndex
                    Layout.fillWidth: true
                    onActivated: console.replay.setSpeedIndex(currentIndex)
                }
                Text { text: "录制来源"; color: Theme.muted }
                ComboBox {
                    enabled: console.replay.canEditSetup
                    model: ["源数据：真机夹爪角度", "源数据：仿真夹爪位移"]
                    currentIndex: console.replay.unitsIndex
                    Layout.fillWidth: true
                    onActivated: console.replay.setUnitsIndex(currentIndex)
                }
                Text { text: "播放次数"; color: Theme.muted }
                RowLayout {
                    enabled: console.replay.canEditRepeat
                    SpinBox {
                        from: 0
                        to: 10000
                        value: console.replay.repeatCount
                        editable: true
                        onValueModified: console.replay.setRepeatCount(value)
                    }
                    CheckBox {
                        text: "持续循环"
                        checked: console.replay.forever
                        onToggled: console.replay.setForever(checked)
                    }
                }
            }

            Text {
                text: "仿真预览直接从首帧循环。真机每遍之间保持使能并慢速返回首帧，返回路径需要确认没有障碍。"
                color: Theme.muted
                font.pixelSize: 13
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
            }

            CheckBox {
                text: console.replay.safetyLabel
                checked: console.replay.safetyChecked
                enabled: console.replay.canEditSetup
                Layout.fillWidth: true
                onToggled: console.replay.setSafetyChecked(checked)
            }
        }

        Card {
            title: "播放"
            Layout.fillWidth: true

            Flow {
                Layout.fillWidth: true
                spacing: 8
                ActionButton {
                    text: console.replay.prepareLabel
                    tone: "accent"
                    enabled: console.replay.canPrepare
                    onClicked: console.replay.prepare()
                }
                ActionButton {
                    text: "开始回放"
                    tone: "primary"
                    enabled: console.replay.canStart
                    onClicked: console.replay.startPlayback()
                }
                ActionButton {
                    text: console.replay.pauseLabel
                    enabled: console.replay.canPause
                    onClicked: console.replay.pausePlayback()
                }
                ActionButton {
                    text: "结束循环"
                    enabled: console.replay.canFinish
                    onClicked: console.replay.finishLoop()
                }
                ActionButton {
                    text: "停止并释放"
                    tone: "danger"
                    enabled: console.replay.canStop
                    onClicked: console.replay.stopPlayback()
                }
            }

            Slider {
                id: bar
                Layout.fillWidth: true
                from: 0
                to: 1000
                enabled: console.replay.canSeek
                onMoved: console.replay.previewSeek(value)
                onPressedChanged: {
                    if (!pressed)
                        console.replay.seekTo(value)
                }
            }

            Connections {
                target: console.replay
                function onChanged() {
                    if (!bar.pressed)
                        bar.value = console.replay.progress
                }
            }

            Text {
                text: console.replay.progressText
                color: Theme.muted
                font.pixelSize: 13
            }
            Text {
                text: console.replay.statusText
                color: Theme.ink
                font.pixelSize: 14
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
            }

            CheckBox {
                id: logToggle
                text: "显示运行日志"
            }
            TextArea {
                visible: logToggle.checked
                text: console.replay.logText
                readOnly: true
                wrapMode: TextEdit.NoWrap
                selectByMouse: true
                font.family: "JetBrains Mono, DejaVu Sans Mono"
                font.pixelSize: 12
                color: Theme.logInk
                Layout.fillWidth: true
                Layout.preferredHeight: 160
            }
        }
    }
}
