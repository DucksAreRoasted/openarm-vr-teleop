import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import OpenArm

Window {
    id: floatLog
    width: 1100
    height: 720
    minimumWidth: 760
    minimumHeight: 420
    title: "OpenArm · 实时日志"
    color: Theme.logBg
    visible: console.logDetached

    onClosing: function(close) {
        close.accepted = false
        console.setLogDetached(false)
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 12
        spacing: 8

        RowLayout {
            Layout.fillWidth: true
            Text {
                text: "实时日志"
                color: Theme.logInk
                font.pixelSize: 16
                font.bold: true
            }
            Item { Layout.fillWidth: true }
            Button {
                text: "收回面板"
                onClicked: console.setLogDetached(false)
            }
        }

        LogView { Layout.fillWidth: true; Layout.fillHeight: true }
    }
}
