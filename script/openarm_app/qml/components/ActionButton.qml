import QtQuick
import QtQuick.Controls
import OpenArm

Button {
    id: control
    property string tone: "normal"
    font.pixelSize: 15
    font.bold: tone === "primary"
    implicitHeight: 42
    leftPadding: 16
    rightPadding: 16
    hoverEnabled: true

    background: Rectangle {
        radius: Theme.controlRadius
        border.width: 1
        color: {
            if (!control.enabled)
                return "#f0f4f8"
            if (control.down)
                return control.tone === "primary" ? Theme.primaryPress : "#d5dee8"
            if (control.hovered) {
                if (control.tone === "primary")
                    return Theme.primaryHover
                if (control.tone === "danger")
                    return "#ffe3e3"
                if (control.tone === "success")
                    return "#d1fae5"
                return "#d9e2ec"
            }
            if (control.tone === "primary")
                return Theme.primary
            if (control.tone === "danger")
                return Theme.dangerBg
            if (control.tone === "success")
                return Theme.successBg
            if (control.tone === "accent")
                return Theme.accentBg
            return Theme.soft
        }
        border.color: {
            if (!control.enabled)
                return "#e0e6ed"
            if (control.tone === "primary")
                return control.background.color
            if (control.tone === "danger")
                return Theme.dangerLine
            if (control.tone === "success")
                return Theme.successLine
            if (control.tone === "accent")
                return Theme.accentLine
            return "#bcccdc"
        }
        Behavior on color { ColorAnimation { duration: 120 } }
    }

    contentItem: Text {
        text: control.text
        font: control.font
        color: {
            if (!control.enabled)
                return "#9aa8b5"
            if (control.tone === "primary")
                return "white"
            if (control.tone === "danger")
                return Theme.danger
            if (control.tone === "success")
                return Theme.success
            if (control.tone === "accent")
                return Theme.accent
            return Theme.ink
        }
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        elide: Text.ElideRight
    }
}
