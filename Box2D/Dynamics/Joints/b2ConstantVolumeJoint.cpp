/*
 * includes
 */
#include <Box2D/Dynamics/Joints/b2ConstantVolumeJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>
#include <Box2D/Dynamics/b2World.h>

/**
 * @brief ConstantVolumeJoint::ConstantVolumeJoint
 * @param def
 */
b2ConstantVolumeJoint::b2ConstantVolumeJoint( const b2ConstantVolumeJointDef *def ) : b2Joint( def ) {
    if ( def->bodies.size() <= 2 )
        return;

    this->world = def->bodyA->GetWorld();
    this->bodies = def->bodies;
    this->targetLengths = new float[this->count()];
    this->m_bodyA = this->bodies[0];
    this->m_bodyB = this->bodies[1];

    for ( size_t i = 0; i < this->count(); i++ ) {
        const size_t next = ( i == this->count() - 1 ) ? 0 : i + 1;
        this->targetLengths[i] = ( this->bodies[i]->GetWorldCenter() - this->bodies[next]->GetWorldCenter()).Length();
    }

    auto getBodyArea = [ this ]() {
        float area = 0.0f;
        for ( size_t i = 0; i < this->count(); i++ ) {
            const size_t next = ( i == this->count() - 1 ) ? 0 : i + 1;
            area += this->bodies[i]->GetWorldCenter().x * this->bodies[next]->GetWorldCenter().y -
                    this->bodies[next]->GetWorldCenter().x * this->bodies[i]->GetWorldCenter().y;
        }
        area *= .5f;
        return area;
    };

    this->targetVolume = getBodyArea();
    for ( size_t i = 0; i < this->count(); i++ ) {
        const size_t next = ( i == this->count() - 1 ) ? 0 : i + 1;

        b2DistanceJointDef *djd( new b2DistanceJointDef());
        djd->frequencyHz = def->frequencyHz;
        djd->dampingRatio = def->dampingRatio;
        djd->collideConnected = def->collideConnected;
        djd->Initialize( this->bodies[i], this->bodies[next], this->bodies[i]->GetWorldCenter(), this->bodies[next]->GetWorldCenter());

        this->distanceJoints.push_back( reinterpret_cast<b2DistanceJoint*>( this->world->CreateJoint( djd )));
    }

    this->normals = reinterpret_cast<b2Vec2*>( b2Alloc( static_cast<int32>( this->count() * sizeof( b2Vec2 ))));
}

/**
 * @brief b2ConstantVolumeJoint::~b2ConstantVolumeJoint
 */
b2ConstantVolumeJoint::~b2ConstantVolumeJoint() {
    delete [] this->normals;
    delete [] this->targetLengths;

    for ( auto* obj : this->distanceJoints )
        this->world->DestroyJoint( obj );

    this->distanceJoints.clear();
}

/**
 * @brief b2ConstantVolumeJoint::ConstrainEdges
 * @return
 */
bool b2ConstantVolumeJoint::ConstrainEdges( b2Position *positions ) {
    float perimeter = 0.0f;

    for ( size_t i = 0; i < this->count(); i++ ) {
        const size_t next = ( i == this->count() - 1 ) ? 0 : i + 1;
        const float dx = positions[this->bodies[next]->m_islandIndex].c.x - positions[this->bodies[i]->m_islandIndex].c.x;
        const float dy = positions[this->bodies[next]->m_islandIndex].c.y - positions[this->bodies[i]->m_islandIndex].c.y;
        float dist = b2Sqrt( dx * dx + dy * dy );
        if ( dist < b2_epsilon )
            dist = 1.0f;

        this->normals[i].x = dy / dist;
        this->normals[i].y = -dx / dist;
        perimeter += dist;
    }

    auto getSolverArea = [ this ]( b2Position *positions ) {
        float area = 0.0f;
        for ( size_t i = 0; i < this->count(); i++ ) {
            const size_t next = ( i == this->count() - 1 ) ? 0 : i + 1;
            area += positions[this->bodies[i]->m_islandIndex].c.x * positions[this->bodies[next]->m_islandIndex].c.y -
                    positions[this->bodies[next]->m_islandIndex].c.x * positions[this->bodies[i]->m_islandIndex].c.y;
        }
        area *= .5f;
        return area;
    };

    const float deltaArea = this->targetVolume - getSolverArea( positions );
    const float toExtrude = 0.5f * deltaArea / perimeter;
    bool done = true;

    for ( size_t i = 0; i < this->count(); i++ ) {
        const size_t next = ( i == this->count() - 1 ) ? 0 : i + 1;
        b2Vec2 delta( toExtrude * ( normals[i].x + normals[next].x ), toExtrude * ( normals[i].y + normals[next].y ));
        float normSqrd = delta.LengthSquared();

        if ( normSqrd > b2_maxLinearCorrection * b2_maxLinearCorrection )
            delta *= ( b2_maxLinearCorrection / b2Sqrt( normSqrd ));

        if ( normSqrd > b2_linearSlop * b2_linearSlop )
            done = false;

        positions[this->bodies[next]->m_islandIndex].c.x += delta.x;
        positions[this->bodies[next]->m_islandIndex].c.y += delta.y;
    }

    return done;
}

/**
 * @brief b2ConstantVolumeJoint::InitVelocityConstraints
 * @param data
 */
void b2ConstantVolumeJoint::InitVelocityConstraints( const b2SolverData &data ) {
    const b2TimeStep step = data.step;
    this->m_step = step;

    b2Velocity *velocities = data.velocities;
    b2Position *positions = data.positions;
    b2Vec2 *d = reinterpret_cast<b2Vec2*>( b2Alloc( static_cast<int32>( this->count() * sizeof( b2Vec2 ))));

    for ( size_t i = 0; i < this->count(); i++ ) {
        const size_t prev = ( i == 0 ) ? this->count() - 1 : i - 1;
        const size_t next = ( i == this->count() - 1 ) ? 0 : i + 1;

        const b2Vec2 &v = positions[this->bodies[next]->m_islandIndex].c;
        d[i].Set( v.x, v.y );
        d[i] -= ( positions[this->bodies[prev]->m_islandIndex].c );
    }

    if ( step.warmStarting ) {
        this->m_impulse *= step.dtRatio;
        for ( size_t i = 0; i < this->count(); i++ ) {
            velocities[this->bodies[i]->m_islandIndex].v.x += this->bodies[i]->m_invMass * d[i].y * .5f * this->m_impulse;
            velocities[this->bodies[i]->m_islandIndex].v.y += this->bodies[i]->m_invMass * -d[i].x * .5f * this->m_impulse;
        }
    } else {
        this->m_impulse = 0.0f;
    }

    b2Free( d );
}

/**
 * @brief b2ConstantVolumeJoint::SolveVelocityConstraints
 */
void b2ConstantVolumeJoint::SolveVelocityConstraints( const b2SolverData &data ) {
    float crossMassSum = 0.0f;
    float dotMassSum = 0.0f;

    b2Velocity *velocities = data.velocities;
    b2Position *positions = data.positions;
    b2Vec2 *d = reinterpret_cast<b2Vec2*>( b2Alloc( static_cast<int32>( this->count() * sizeof( b2Vec2 ))));

    for ( size_t i = 0; i < this->count(); i++ ) {
        const size_t prev = ( i == 0 ) ? this->count() - 1 : i - 1;
        const size_t next = ( i == this->count() - 1 ) ? 0 : i + 1;

        const b2Vec2 &v = positions[this->bodies[next]->m_islandIndex].c;
        d[i].Set( v.x, v.y );
        d[i] -= ( positions[this->bodies[prev]->m_islandIndex].c );

        dotMassSum += ( d[i].LengthSquared()) / this->bodies[i]->GetMass();
        crossMassSum += b2Cross( velocities[this->bodies[i]->m_islandIndex].v, d[i]);
    }

    const float lambda = -2.0f * crossMassSum / dotMassSum;
    this->m_impulse += lambda;

    for ( size_t i = 0; i < this->count(); i++ ) {
        velocities[this->bodies[i]->m_islandIndex].v.x += this->bodies[i]->m_invMass * d[i].y * .5f * lambda;
        velocities[this->bodies[i]->m_islandIndex].v.y += this->bodies[i]->m_invMass * -d[i].x * .5f * lambda;
    }

    b2Free( d );
}
