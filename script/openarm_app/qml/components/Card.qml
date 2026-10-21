import QtQuick
import QtQuick.Layouts
import OpenArm

Item {
    id: root
    property string title: ""
    default property alias body: column.data
    implicitHeight: column.implicitHeight + 40

    Rectangle {
        anchors.fill: card
        anchors.topMargin: 4
        radius: Theme.radius
        color: "#140f172a"
    }

    Rectangle {
        id: card
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 4
        radius: Theme.radius
        color: Theme.card
        border.color: Theme.line

        ColumnLayout {
            id: column
            anchors.fill: parent
            anchors.margins: 18
            spacing: 12

            Text {
                visible: root.title !== ""
                text: root.title
                color: "#243b53"
                font.pixelSize: 16
                font.bold: true
                Layout.fillWidth: true
            }
        }
    }
}
