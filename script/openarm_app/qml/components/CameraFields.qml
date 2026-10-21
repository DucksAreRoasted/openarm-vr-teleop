import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import OpenArm

ColumnLayout {
    spacing: 10

    GridLayout {
        columns: 2
        columnSpacing: 12
        rowSpacing: 8
        Layout.fillWidth: true

        Text { text: "头部"; color: Theme.ink }
        ComboBox {
            model: console.cameraOptions
            Layout.fillWidth: true
            currentIndex: Math.max(0, find(console.headCamera))
            onActivated: console.headCamera = currentText
        }
        Text { text: "左腕"; color: Theme.ink }
        ComboBox {
            model: console.cameraOptions
            Layout.fillWidth: true
            currentIndex: Math.max(0, find(console.wristLeftCamera))
            onActivated: console.wristLeftCamera = currentText
        }
        Text { text: "右腕"; color: Theme.ink }
        ComboBox {
            model: console.cameraOptions
            Layout.fillWidth: true
            currentIndex: Math.max(0, find(console.wristRightCamera))
            onActivated: console.wristRightCamera = currentText
        }
    }

    RowLayout {
        Layout.fillWidth: true
        ActionButton {
            text: "刷新 USB"
            onClicked: console.refreshCameras()
        }
        ActionButton {
            text: "打开相机预览"
            tone: "accent"
            onClicked: console.openCameras()
        }
        Item { Layout.fillWidth: true }
    }

    Text {
        text: console.cameraStatus
        color: Theme.muted
        font.pixelSize: 13
        wrapMode: Text.WordWrap
        Layout.fillWidth: true
    }
}
