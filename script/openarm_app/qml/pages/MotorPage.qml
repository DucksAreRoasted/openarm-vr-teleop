import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import OpenArm
import "../components"

ScrollView {
    id: motorPage
    clip: true
    contentWidth: availableWidth

    ColumnLayout {
        width: motorPage.availableWidth
        spacing: 12

        Text {
            text: "查询双臂 EEPROM，不使能、不动臂。未读到的值显示为 —。"
            color: Theme.muted
            font.pixelSize: 14
            wrapMode: Text.WordWrap
            Layout.fillWidth: true
        }

        Card {
            title: "连接"
            Layout.fillWidth: true

            CanFields {
                fieldsEnabled: console.canMaintain
                Layout.fillWidth: true
            }

            RowLayout {
                Layout.fillWidth: true
                ActionButton {
                    text: "读取电机参数"
                    tone: "primary"
                    enabled: console.canMaintain
                    onClicked: console.readMotors()
                }
                Text {
                    text: console.motorMessage
                    color: Theme.ink
                    font.pixelSize: 14
                    wrapMode: Text.WordWrap
                    Layout.fillWidth: true
                }
            }
            Text {
                visible: console.motorReport !== ""
                text: console.motorReport
                color: Theme.muted
                font.pixelSize: 12
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
            }
        }

        Card {
            title: "实时关节角度"
            Layout.fillWidth: true
            Text {
                text: "运行遥操作、仿真或拖动示教 / 零重力后自动显示实测值（约 30 Hz）；停止后 0.5 秒显示超时。MoveIt 暂未接入。"
                color: Theme.muted
                font.pixelSize: 12
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
            }
            Repeater {
                model: console.jointAngles.rows
                ColumnLayout {
                    required property var modelData
                    Layout.fillWidth: true
                    Text { text: modelData.side + " · " + modelData.status; color: Theme.ink; font.bold: true }
                    Flow {
                        Layout.fillWidth: true
                        spacing: 12
                        Repeater {
                            model: modelData.values
                            Text {
                                required property int index
                                required property string modelData
                                text: "J" + (index + 1) + "  " + modelData
                                color: Theme.ink
                                font.pixelSize: 12
                            }
                        }
                    }
                }
            }
        }

        Card {
            title: "关节参数"
            Layout.fillWidth: true
            Layout.preferredHeight: 360

            RowLayout {
                Layout.fillWidth: true
                ActionButton {
                    text: "左臂"
                    tone: console.motorSide === "left" ? "primary" : "normal"
                    onClicked: console.setMotorSide("left")
                }
                ActionButton {
                    text: "右臂"
                    tone: console.motorSide === "right" ? "primary" : "normal"
                    onClicked: console.setMotorSide("right")
                }
                Item { Layout.fillWidth: true }
            }

            Flickable {
                id: tableScroll
                Layout.fillWidth: true
                Layout.fillHeight: true
                contentWidth: table.implicitWidth
                contentHeight: height
                clip: true
                boundsBehavior: Flickable.StopAtBounds

                Column {
                    id: table
                    spacing: 0

                    Row {
                        spacing: 8
                        Repeater {
                            model: [
                                { label: "关节", w: 52 },
                                { label: "型号", w: 88 },
                                { label: "模式", w: 88 },
                                { label: "KT", w: 88 },
                                { label: "减速比", w: 88 },
                                { label: "极对数", w: 88 },
                                { label: "相电阻", w: 88 },
                                { label: "相电感", w: 88 },
                                { label: "磁链", w: 88 },
                                { label: "惯量", w: 88 },
                                { label: "阻尼", w: 88 },
                                { label: "位置上限", w: 88 },
                                { label: "速度上限", w: 88 },
                                { label: "力矩上限", w: 88 }
                            ]
                            Text {
                                required property var modelData
                                width: modelData.w
                                text: modelData.label
                                color: Theme.muted
                                font.pixelSize: 12
                                font.bold: true
                            }
                        }
                    }

                    ListView {
                        width: table.width
                        height: Math.max(120, tableScroll.height - 28)
                        clip: true
                        model: console.motors
                        spacing: 2

                        delegate: Rectangle {
                            required property string joint
                            required property string typeName
                            required property string modeName
                            required property string kt
                            required property string gear
                            required property string poles
                            required property string resistance
                            required property string inductance
                            required property string flux
                            required property string inertia
                            required property string damp
                            required property string pmax
                            required property string vmax
                            required property string tmax
                            width: table.width
                            height: 32
                            radius: 6
                            color: "#f8fafc"

                            Row {
                                anchors.fill: parent
                                anchors.leftMargin: 6
                                anchors.rightMargin: 6
                                spacing: 8
                                Text { width: 52; text: joint; color: Theme.ink; font.bold: true; anchors.verticalCenter: parent.verticalCenter }
                                Text { width: 88; text: typeName; color: Theme.ink; anchors.verticalCenter: parent.verticalCenter }
                                Text { width: 88; text: modeName; color: Theme.accent; anchors.verticalCenter: parent.verticalCenter }
                                Text { width: 88; text: kt; color: Theme.ink; anchors.verticalCenter: parent.verticalCenter }
                                Text { width: 88; text: gear; color: Theme.ink; anchors.verticalCenter: parent.verticalCenter }
                                Text { width: 88; text: poles; color: Theme.ink; anchors.verticalCenter: parent.verticalCenter }
                                Text { width: 88; text: resistance; color: Theme.ink; anchors.verticalCenter: parent.verticalCenter }
                                Text { width: 88; text: inductance; color: Theme.ink; anchors.verticalCenter: parent.verticalCenter }
                                Text { width: 88; text: flux; color: Theme.ink; anchors.verticalCenter: parent.verticalCenter }
                                Text { width: 88; text: inertia; color: Theme.ink; anchors.verticalCenter: parent.verticalCenter }
                                Text { width: 88; text: damp; color: Theme.ink; anchors.verticalCenter: parent.verticalCenter }
                                Text { width: 88; text: pmax; color: Theme.ink; anchors.verticalCenter: parent.verticalCenter }
                                Text { width: 88; text: vmax; color: Theme.ink; anchors.verticalCenter: parent.verticalCenter }
                                Text { width: 88; text: tmax; color: Theme.ink; anchors.verticalCenter: parent.verticalCenter }
                            }
                        }
                    }
                }
            }
        }
    }
}
