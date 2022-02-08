#pragma once

#include <vector>
#include <Box2D/Dynamics/Joints/b2Joint.h>
#include <Box2D/Dynamics/Joints/b2DistanceJoint.h>
#include <Box2D/Dynamics/b2TimeStep.h>

/**
 * @brief The ConstantVolumeJointDef struct
 */
struct b2ConstantVolumeJointDef : public b2JointDef {
    std::vector<b2Body*> bodies;
    float frequencyHz;
    float dampingRatio;

    /**
     * @brief ConstantVolumeJointDef
     */
    explicit b2ConstantVolumeJointDef() {
        this->type = e_constantVolumeJoint;
        this->collideConnected = false;
        this->frequencyHz = 2.0f;
        this->dampingRatio = 0.5f;
    }

    /**
     * @brief Initialize
     * @param bodies
     */
    void Initialize( const std::vector<b2Body*> &bodies ) {
        this->bodies = bodies;

        if ( this->bodies.size() <= 2 )
            return;

        this->bodyA = this->bodies[0];
        this->bodyB = this->bodies[1];
    }
};

/**
 * @brief The b2ConstantVolumeJoint class
 */
class b2ConstantVolumeJoint : public b2Joint {
public:
    std::vector<b2Body*> bodies;
    std::vector<b2DistanceJoint*> distanceJoints;

    /**
     * @brief Inflate
     * @param factor
     */
    void Inflate( float factor ) { this->targetVolume *= factor; }

    /**
     * @brief ConstrainEdges
     * @return
     */
    bool ConstrainEdges( b2Position *positions );

    /**
     * @brief SolvePositionConstraints
     * @return
     */
    bool SolvePositionConstraints( const b2SolverData &step ) override {
        return this->ConstrainEdges( step.positions );
    }

    void InitVelocityConstraints( const b2SolverData &data ) override;
    void SolveVelocityConstraints( const b2SolverData & ) override;

    /**
     * @brief GetAnchorA
     * @return
     */
    virtual b2Vec2 GetAnchorA() const override { return b2Vec2(); }

    /**
     * @brief GetAnchorB
     * @return
     */
    virtual b2Vec2 GetAnchorB() const override { return b2Vec2(); }

    /**
     * @brief GetReactionForce
     * @param inv_dt
     * @return
     */
    virtual b2Vec2 GetReactionForce( float32 ) const override { return b2Vec2(); }

    /**
     * @brief GetReactionTorque
     * @return
     */
    virtual float32 GetReactionTorque( float32 ) const override { return 0; }

protected:
    friend class b2Joint;

    explicit b2ConstantVolumeJoint( const b2ConstantVolumeJointDef *def );
    ~b2ConstantVolumeJoint() override;

    /**
     * @brief count
     * @return
     */
    size_t count() const { return this->bodies.size(); }
    float *targetLengths = nullptr;
    float targetVolume = 0.0;
    float m_impulse = 0.0;

    b2World *world = nullptr;
    b2Vec2 *normals = nullptr;
    b2TimeStep m_step;
};
