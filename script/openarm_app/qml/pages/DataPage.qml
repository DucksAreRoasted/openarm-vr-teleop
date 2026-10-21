import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import OpenArm
import "../components"

Item {
    ColumnLayout {
        anchors.fill: parent
        spacing: 10

        RowLayout {
            Layout.fillWidth: true
            Text {
                text: console.catalog.statusText
                color: Theme.muted
                font.pixelSize: 13
                elide: Text.ElideRight
                Layout.fillWidth: true
            }
            ActionButton { text: "刷新"; onClicked: console.catalog.refresh() }
        }

        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 12

            Card {
                title: "批次"
                Layout.preferredWidth: 320
                Layout.fillHeight: true

                ListView {
                    id: batches
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    clip: true
                    model: console.catalog.batches
                    spacing: 4
                    boundsBehavior: Flickable.StopAtBounds

                    delegate: Rectangle {
                        required property string batchName
                        required property string summary
                        required property int index
                        width: batches.width
                        height: textCol.implicitHeight + 16
                        radius: 8
                        color: batchName === console.catalog.selectedBatch ? "#e7f6f8" : "#f8fafc"
                        border.color: batchName === console.catalog.selectedBatch ? Theme.primary : Theme.line

                        Column {
                            id: textCol
                            anchors.left: parent.left
                            anchors.right: parent.right
                            anchors.verticalCenter: parent.verticalCenter
                            anchors.margins: 10
                            spacing: 2
                            Text {
                                width: parent.width
                                text: batchName
                                color: Theme.ink
                                font.bold: true
                                elide: Text.ElideRight
                            }
                            Text {
                                width: parent.width
                                text: summary
                                color: Theme.muted
                                font.pixelSize: 12
                                wrapMode: Text.WordWrap
                            }
                        }

                        TapHandler { onTapped: console.catalog.selectBatch(batchName) }
                    }
                }

                RowLayout {
                    Layout.fillWidth: true
                    ActionButton { text: "打开目录"; enabled: console.catalog.canOpen; onClicked: console.catalog.openFolder() }
                    ActionButton { text: "导出"; enabled: console.catalog.canExport; onClicked: console.catalog.exportSelected() }
                    ActionButton {
                        text: "删整批"
                        tone: "danger"
                        enabled: console.catalog.canDeleteBatch
                        onClicked: console.catalog.deleteBatch()
                    }
                }
            }

            Card {
                title: "片段"
                Layout.fillWidth: true
                Layout.fillHeight: true

                RowLayout {
                    Layout.fillWidth: true
                    Text { text: "筛选"; color: Theme.muted }
                    ComboBox {
                        model: ["全部", "成功", "失败", "未标注"]
                        currentIndex: ["全部", "成功", "失败", "未标注"].indexOf(console.catalog.episodeFilter)
                        onActivated: console.catalog.setEpisodeFilter(currentText)
                    }
                    Item { Layout.fillWidth: true }
                }

                ListView {
                    id: episodes
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    clip: true
                    model: console.catalog.episodes
                    spacing: 4
                    boundsBehavior: Flickable.StopAtBounds

                    delegate: Rectangle {
                        required property string episodeId
                        required property string outcomeLabel
                        required property string detail
                        width: episodes.width
                        height: 52
                        radius: 8
                        color: episodeId === console.catalog.selectedEpisode ? "#e7f6f8" : "#f8fafc"
                        border.color: episodeId === console.catalog.selectedEpisode ? Theme.primary : Theme.line

                        RowLayout {
                            anchors.fill: parent
                            anchors.margins: 10
                            Text {
                                text: episodeId
                                color: Theme.ink
                                font.bold: true
                                font.pixelSize: 16
                            }
                            Text { text: outcomeLabel; color: Theme.accent; font.bold: true }
                            Text {
                                text: detail
                                color: Theme.muted
                                font.pixelSize: 12
                                Layout.fillWidth: true
                                elide: Text.ElideRight
                            }
                        }

                        TapHandler { onTapped: console.catalog.selectEpisode(episodeId) }
                    }
                }

                Flow {
                    Layout.fillWidth: true
                    spacing: 8
                    ActionButton {
                        text: "改成功"
                        tone: "success"
                        enabled: console.catalog.canEditEpisode
                        onClicked: console.catalog.setOutcome("success")
                    }
                    ActionButton {
                        text: "改失败"
                        tone: "danger"
                        enabled: console.catalog.canEditEpisode
                        onClicked: console.catalog.setOutcome("failure")
                    }
                    ActionButton {
                        text: "重采此条"
                        enabled: console.catalog.canRerecord
                        onClicked: console.catalog.rerecord()
                    }
                    ActionButton {
                        text: "平滑此条…"
                        enabled: console.catalog.canReplay
                        onClicked: console.catalog.smoothSelected()
                    }
                    ActionButton {
                        text: "回放此条"
                        enabled: console.catalog.canReplay
                        onClicked: console.catalog.replaySelected()
                    }
                    ActionButton {
                        text: "删片段"
                        tone: "danger"
                        enabled: console.catalog.canEditEpisode
                        onClicked: console.catalog.deleteEpisode()
                    }
                }
            }
        }
    }
}
