import QtQuick
import OpenArm

Rectangle {
    id: root
    property string text: ""
    property color fg: Theme.muted
    property color bg: "#e8eef4"
    implicitWidth: label.implicitWidth + 22
    implicitHeight: 28
    radius: 14
    color: bg

    Text {
        id: label
        anchors.centerIn: parent
        text: root.text
        color: root.fg
        font.pixelSize: 13
        font.bold: true
    }

    Behavior on color { ColorAnimation { duration: 150 } }
}
