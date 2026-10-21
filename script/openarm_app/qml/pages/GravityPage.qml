import QtQuick
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
            text: "每一步都直接覆盖当前配置，不另存备份。写入后下次启动生效。"
            color: Theme.muted
            font.pixelSize: 14
            wrapMode: Text.WordWrap
            Layout.fillWidth: true
        }

        Card {
            Layout.fillWidth: true
            RowLayout {
                Layout.fillWidth: true
                spacing: 14
                Rectangle {
                    width: 32
                    height: 32
                    radius: 16
                    color: Theme.accentBg
                    Text { anchors.centerIn: parent; text: "1"; color: Theme.accent; font.bold: true }
                }
                ColumnLayout {
                    Layout.fillWidth: true
                    spacing: 4
                    Text { text: "使用 URDF 初值"; color: Theme.ink; font.pixelSize: 16; font.bold: true }
                    Text {
                        text: "不动臂。把质量、质心和关节修正恢复为模型基准，前馈增益设为 0.95。夹爪、相机和摩擦保留。"
                        color: Theme.muted
                        font.pixelSize: 13
                        wrapMode: Text.WordWrap
                        Layout.fillWidth: true
                    }
                }
                ActionButton {
                    text: "写入初值"
                    enabled: console.canMaintain
                    onClicked: console.seedGravity()
                }
            }
        }

        Card {
            Layout.fillWidth: true
            RowLayout {
                Layout.fillWidth: true
                spacing: 14
                Rectangle {
                    width: 32
                    height: 32
                    radius: 16
                    color: "#e0f2fe"
                    Text { anchors.centerIn: parent; text: "2"; color: Theme.primary; font.bold: true }
                }
                ColumnLayout {
                    Layout.fillWidth: true
                    spacing: 4
                    Text { text: "自动微调"; color: Theme.ink; font.pixelSize: 16; font.bold: true }
                    Text {
                        text: "机械臂会运动，大约 10–15 分钟。腕、肘、肩从两侧接近后软持采数，只小幅调整关节倍率。需要勾选安全确认。"
                        color: Theme.muted
                        font.pixelSize: 13
                        wrapMode: Text.WordWrap
                        Layout.fillWidth: true
                    }
                }
                ActionButton {
                    text: "开始微调"
                    tone: "primary"
                    enabled: console.canMaintain
                    onClicked: console.autoCalib()
                }
            }
        }

        Card {
            Layout.fillWidth: true
            RowLayout {
                Layout.fillWidth: true
                spacing: 14
                Rectangle {
                    width: 32
                    height: 32
                    radius: 16
                    color: "#ecfdf5"
                    Text { anchors.centerIn: parent; text: "3"; color: Theme.success; font.bold: true }
                }
                ColumnLayout {
                    Layout.fillWidth: true
                    spacing: 4
                    Text { text: "惯量标定"; color: Theme.ink; font.pixelSize: 16; font.bold: true }
                    Text {
                        text: "不动臂。用最近一次自动微调的两侧日志辨识质量与质心。先看误差预览，确认后才写入。"
                        color: Theme.muted
                        font.pixelSize: 13
                        wrapMode: Text.WordWrap
                        Layout.fillWidth: true
                    }
                }
                ActionButton {
                    text: "辨识惯量"
                    enabled: console.canMaintain
                    onClicked: console.identifyGravity()
                }
            }
        }

        Card {
            Layout.fillWidth: true
            RowLayout {
                Layout.fillWidth: true
                spacing: 14
                Rectangle {
                    width: 32
                    height: 32
                    radius: 16
                    color: "#fef3c7"
                    Text { anchors.centerIn: parent; text: "4"; color: "#b45309"; font.bold: true }
                }
                ColumnLayout {
                    Layout.fillWidth: true
                    spacing: 4
                    Text { text: "摩擦辨识"; color: Theme.ink; font.pixelSize: 16; font.bold: true }
                    Text {
                        text: "机械臂会运动，大约 8–10 分钟。逐个关节按三档速度正反向转动，拟合库仑与黏性摩擦。通过的关节打 85 折并把换向陡度写成 8；超过上限的保留原值。确认后写入配置。重力补偿目前不下发这些摩擦力矩。需要勾选安全确认。"
                        color: Theme.muted
                        font.pixelSize: 13
                        wrapMode: Text.WordWrap
                        Layout.fillWidth: true
                    }
                }
                ActionButton {
                    text: "辨识摩擦"
                    tone: "primary"
                    enabled: console.canMaintain
                    onClicked: console.identifyFriction()
                }
            }
        }

    }
}
