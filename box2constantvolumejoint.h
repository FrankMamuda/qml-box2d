#pragma once

/*
 * includes
 */
#include "box2djoint.h"
#include <Box2D.h>
#include <Dynamics/Joints/b2ConstantVolumeJoint.h>

class Box2DConstantVolumeJoint : public Box2DJoint {
    Q_OBJECT
    Q_PROPERTY(QVariantList bodies READ bodies WRITE setBodies NOTIFY bodiesChanged)
    Q_PROPERTY(float frequencyHz READ frequencyHz WRITE setFrequencyHz NOTIFY frequencyHzChanged)
    Q_PROPERTY(float dampingRatio READ dampingRatio WRITE setDampingRatio NOTIFY dampingRatioChanged)

public:
    explicit Box2DConstantVolumeJoint( QObject *parent = nullptr );

    /**
     * @brief bodies
     * @return
     */
    [[nodiscard]] QVariantList bodies() const { return this->m_bodies; }

    /**
     * @brief setBodies
     * @param bodies
     */
    void setBodies( const QVariantList &bodies ) {
        if ( bodies == this->m_bodies )
            return;

        this->m_bodies = bodies;

        if ( this->bodies().count() >= 2 ) {
            this->setBodyA( this->bodies().at( 0 ).value<Box2DBody*>());
            this->setBodyB( this->bodies().at( 1 ).value<Box2DBody*>());
        }

        //this->createJoint();
        emit this->bodiesChanged();
    }

    /**
     * @brief dampingRatio
     * @return
     */
    [[nodiscard]] float dampingRatio() const { return this->m_dampingRatio; }

    /**
     * @brief setDampingRatio
     * @param dampingRatio
     */
    void setDampingRatio( float dampingRatio ) {
        if ( qFuzzyCompare( this->m_dampingRatio, dampingRatio ))
            return;

        this->m_dampingRatio = dampingRatio;
        if ( this->constantVolumeJoint()) {
            for ( b2DistanceJoint *dj : this->constantVolumeJoint()->distanceJoints )
                dj->SetDampingRatio( m_dampingRatio );
        }

        emit this->dampingRatioChanged();
    }

    /**
     * @brief frequencyHz
     * @return
     */
    [[nodiscard]] float frequencyHz() const { return this->m_frequencyHz; }

    /**
     * @brief setFrequencyHz
     * @param frequencyHz
     */
    void setFrequencyHz( float frequencyHz ) {
        if ( qFuzzyCompare( this->m_frequencyHz, frequencyHz ))
            return;

        this->m_frequencyHz = frequencyHz;
        if ( this->constantVolumeJoint()) {
            for ( b2DistanceJoint *dj : this->constantVolumeJoint()->distanceJoints )
                dj->SetFrequency( frequencyHz );
        }

        emit this->frequencyHzChanged();
    }

    /**
     * @brief constantVolumeJoint
     * @return
     */
    [[nodiscard]] b2ConstantVolumeJoint *constantVolumeJoint() const { return static_cast<b2ConstantVolumeJoint*>( this->joint()); }

signals:
    void bodiesChanged();
    void frequencyHzChanged();
    void dampingRatioChanged();

protected:
    b2Joint *createJoint() override;

private:
    QVariantList m_bodies;
    float m_frequencyHz = 0.0;
    float m_dampingRatio = 0.0;
};
