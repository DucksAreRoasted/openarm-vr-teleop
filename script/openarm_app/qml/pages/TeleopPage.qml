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
            title: "模式"
            Layout.fillWidth: true

            Text {
                text: "仿真只动模型。真机、MoveIt 实机和零重力会连接机械臂。"
                color: Theme.muted
                font.pixelSize: 13
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
            }

            RowLayout {
                Layout.fillWidth: true
                spacing: 8
                Repeater {
                    model: [
                        { id: "sim", label: "仿真遥操" },
                        { id: "real", label: "真机遥操" },
                        { id: "moveit", label: "MoveIt" },
                        { id: "zerog", label: "零重力" }
                    ]
                    ActionButton {
                        required property var modelData
                        text: modelData.label
                        tone: console.mode === modelData.id ? "primary" : "normal"
                        enabled: !console.running
                        Layout.fillWidth: true
                        onClicked: console.setMode(modelData.id)
                    }
                }
            }
        }

        Card {
            title: "设备"
            Layout.fillWidth: true

            Text {
                text: console.showConnection
                      ? "四个模式共用这一套臂和相机。真机、MoveIt 和零重力才需要 CAN。"
                      : "仿真只用相机。切到真机、MoveIt 或零重力后，在这里填写 CAN。"
                color: Theme.muted
                font.pixelSize: 13
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
            }

            CanFields {
                visible: console.showConnection
                fieldsEnabled: console.canFieldsEnabled
                showSafety: true
                Layout.fillWidth: true
            }

            CameraFields { Layout.fillWidth: true }
        }

        Card {
            title: "MoveIt 软硬"
            visible: console.showImpedance
            Layout.fillWidth: true

            Text {
                text: "启动前选择起始软硬。真机运行中可切换软硬或软急停。停止并释放才会失能下落。"
                color: Theme.muted
                font.pixelSize: 13
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
            }

            RowLayout {
                Layout.fillWidth: true
                enabled: console.impedanceProfileEnabled
                Text { text: "起始软硬"; color: Theme.ink }
                ComboBox {
                    Layout.fillWidth: true
                    model: [
                        { id: "P0", label: "正常刚度" },
                        { id: "P1", label: "中等刚度" },
                        { id: "P2", label: "偏低刚度" },
                        { id: "P3", label: "接触柔顺" }
                    ]
                    textRole: "label"
                    valueRole: "id"
                    currentIndex: {
                        if (console.impedanceProfile === "P1") return 1
                        if (console.impedanceProfile === "P2") return 2
                        if (console.impedanceProfile === "P3") return 3
                        return 0
                    }
                    onActivated: console.impedanceProfile = currentValue
                }
            }

            Flow {
                Layout.fillWidth: true
                spacing: 8
                enabled: console.impedanceLive
                ActionButton { text: "正常刚度"; onClicked: console.setImpedance("P0") }
                ActionButton { text: "中等刚度"; onClicked: console.setImpedance("P1") }
                ActionButton { text: "偏低刚度"; onClicked: console.setImpedance("P2") }
                ActionButton { text: "接触柔顺"; onClicked: console.setImpedance("P3") }
                ActionButton { text: "手拖"; onClicked: console.setImpedance("drag") }
                ActionButton { text: "软急停"; tone: "danger"; onClicked: console.softStop() }
            }
        }

    }
}
