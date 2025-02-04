import QtQuick 2.2
import Box2D 2.0
import "../shared"

PhysicsItem {
    id: wall

    fixtures: Box {
        width: wall.width
        height: wall.height
        friction: 1
        density: 1
    }

    Rectangle {
        color: 'gray'
        anchors.fill: parent
    }
}
