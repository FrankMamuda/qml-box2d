/*
 * includes
 */
#include "box2constantvolumejoint.h"

/**
 * @brief Box2DConstantVolumeJoint::Box2DConstantVolumeJoint
 * @param parent
 */
Box2DConstantVolumeJoint::Box2DConstantVolumeJoint( QObject *parent )
    : Box2DJoint( ConstantVolumeJoint, parent ) {}

/**
 * @brief Box2DConstantVolumeJoint::createJoint
 * @return
 */
b2Joint *Box2DConstantVolumeJoint::createJoint() {
    b2ConstantVolumeJointDef jointDef;

    std::vector<b2Body*> b2Bodies;
    for ( const QVariant &v : this->bodies())
        b2Bodies.push_back( v.value<Box2DBody*>()->body());

    initializeJointDef( jointDef );
    jointDef.Initialize( b2Bodies );
    jointDef.dampingRatio = this->dampingRatio();
    jointDef.frequencyHz = this->frequencyHz();

    return this->world()->world().CreateJoint( &jointDef );
}
