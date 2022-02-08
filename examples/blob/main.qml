import QtQuick 2.2
import Box2D 2.0
import QtQuick.Controls 2.2
import "../shared"

Rectangle {
    id: screen

    width: 800
    height: 600

    Component {
        id: cvjComponent
        ConstantVolumeJoint {}
    }

    World { id: physicsWorld }

    Component {
        id: ball
        Rectangle {
            id: rectangle
            radius: width / 2
            width: 16
            height: 16
            smooth: true
            color: "blue"
            property CircleBody body: circleBody

            CircleBody {
                id: circleBody
                target: rectangle
                world: physicsWorld
                bodyType: Body.Dynamic
                radius: 8
            }
        }
    }

    Component.onCompleted: {
        const blobX = 420;
        const blobY = 320;
        const links = 44;
        const radius = 100;
        let bodies = [];

        function map( v1, min1, max1, min2, max2 ) { return ( min2 + (( max2 - min2 )* ( v1 - min1 )) / ( max1 - min1 )); };
        for ( let i = 0; i < links; i++ ) {
            const theta = map( i, 0, links, 0, Math.PI * 2.0 );
            const x = blobX + radius * Math.sin( theta );
            const y = blobY + radius * Math.cos( theta );
            const body = ball.createObject( screen, { x: x, y: y } );

            body.body.fixedRotation = true;
            bodies.push( body.body );
        }

        cvjComponent.createObject( screen, { bodies: bodies, frequencyHz: 10, dampingRatio: 50 } )
    }

    PhysicsItem {
        id: ground
        height: 40
        anchors {
            left: parent.left
            right: parent.right
            bottom: parent.bottom
        }
        fixtures: Box {
            width: ground.width
            height: ground.height
            friction: 1
            density: 1
        }
        Rectangle {
            anchors.fill: parent
            color: "#DEDEDE"
        }
    }

    Wall {
        id: topWall
        height: 40
        anchors {
            left: parent.left
            right: parent.right
            top: parent.top
        }
    }

    Wall {
        id: leftWall
        width: 40
        anchors {
            left: parent.left
            top: parent.top
            bottom: parent.bottom
            bottomMargin: 40
        }
    }

    Wall {
        id: rightWall
        width: 40
        anchors {
            right: parent.right
            top: parent.top
            bottom: parent.bottom
            bottomMargin: 40
        }
    }

    Rectangle {
        id: debugButton
        x: 50
        y: 50
        width: 120
        height: 30
        Text {
            text: "Debug view: " + (debugDraw.visible ? "on" : "off")
            anchors.centerIn: parent
        }
        color: "#DEDEDE"
        border.color: "#999"
        radius: 5
        MouseArea {
            anchors.fill: parent
            onClicked: debugDraw.visible = !debugDraw.visible;
        }
    }

    DebugDraw {
        id: debugDraw
        world: physicsWorld
        opacity: 1
        visible: false
    }
}
