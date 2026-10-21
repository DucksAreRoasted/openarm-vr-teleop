import QtQuick
import OpenArm

Rectangle {
    id: root
    property string label: ""
    property bool selected: false
    property bool marked: false
    signal clicked
    implicitHeight: 46
    radius: 10
    color: "transparent"

    Rectangle {
        anchors.fill: parent
        radius: 10
        color: root.selected ? "#e7f6f8" : "#eef3f8"
        opacity: root.selected || mouse.containsMouse ? 1 : 0
        Behavior on opacity { NumberAnimation { duration: 120 } }
    }

    Rectangle {
        width: 3
        radius: 2
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        anchors.topMargin: 10
        anchors.bottomMargin: 10
        color: Theme.primary
        visible: root.selected
    }

    Text {
        anchors.left: parent.left
        anchors.leftMargin: 16
        anchors.verticalCenter: parent.verticalCenter
        text: root.label
        color: root.selected ? Theme.primary : (mouse.containsMouse ? Theme.primary : Theme.ink)
        font.pixelSize: 15
        font.bold: root.selected
    }

    Rectangle {
        visible: root.marked
        width: 8
        height: 8
        radius: 4
        color: "#b91c1c"
        anchors.right: parent.right
        anchors.rightMargin: 14
        anchors.verticalCenter: parent.verticalCenter
    }

    MouseArea {
        id: mouse
        anchors.fill: parent
        hoverEnabled: true
        cursorShape: Qt.PointingHandCursor
        onClicked: root.clicked()
    }
}
