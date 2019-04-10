/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "VelocityConstraint.h"

namespace internal {
namespace kinematics {

    VelocityConstraint::VelocityConstraint(const std::string& frameName, VelocityConstraintType type)
        : m_type(type)
        , m_frameName(frameName)
        , m_linearVelocityWeight(1.0)
        , m_angularVelocityWeight(1.0)
    {}

    VelocityConstraint VelocityConstraint::linearVelocityConstraint(const std::string& frameName,
                                                                    const iDynTree::Vector3& linearVelocity,
                                                                    const double linearVelocityWeight)
    {
        VelocityConstraint velocityConstraint(frameName, VelocityConstraintTypeLinearVelocity);
        velocityConstraint.setLinearVelocity(linearVelocity);
        velocityConstraint.setLinearVelocityWeight(linearVelocityWeight);
        return velocityConstraint;
    }

    VelocityConstraint VelocityConstraint::angularVelocityConstraint(const std::string& frameName,
                                                                     const iDynTree::Vector3& angularVelocity,
                                                                     const double angularVelocityWeight)
    {
        VelocityConstraint velocityConstraint(frameName, VelocityConstraintTypeAngularVelocity);
        velocityConstraint.setLinearVelocity(angularVelocity);
        velocityConstraint.setLinearVelocityWeight(angularVelocityWeight);
        return velocityConstraint;
    }

    VelocityConstraint VelocityConstraint::TwistConstraint(const std::string& frameName,
                                                           const iDynTree::Vector3& linearVelocity,
                                                           const iDynTree::Vector3& angularVelocity,
                                                           const double linearVelocityWeight,
                                                           const double angularVelocityWeight)
    {
        iDynTree::Twist twist(linearVelocity, angularVelocity);
        return TwistConstraint(frameName, twist, linearVelocityWeight, angularVelocityWeight);
    }

    VelocityConstraint VelocityConstraint::TwistConstraint(const std::string& frameName,
                                                           const iDynTree::Twist& twist,
                                                           const double linearVelocityWeight,
                                                           const double angularVelocityWeight)
    {
        VelocityConstraint velocityConstraint(frameName, VelocityConstraintTypeTwist);
        velocityConstraint.setTwist(twist);
        velocityConstraint.setLinearVelocityWeight(linearVelocityWeight);
        velocityConstraint.setLinearVelocityWeight(angularVelocityWeight);
        return velocityConstraint;
    }

    VelocityConstraint::VelocityConstraintType VelocityConstraint::getType() const
    {
        return m_type;
    }

    const std::string& VelocityConstraint::getFrameName() const
    {
        return m_frameName;
    }

    bool VelocityConstraint::hasLinearVelocityConstraint() const
    {
        return (m_type == VelocityConstraintTypeLinearVelocity) || (m_type == VelocityConstraintTypeTwist);
    }

    bool VelocityConstraint::hasAngularVelocityConstraint() const
    {
        return (m_type == VelocityConstraintTypeAngularVelocity) || (m_type == VelocityConstraintTypeTwist);
    }

    const iDynTree::Vector3& VelocityConstraint::getLinearVelocity() const
    {
        return m_twist.getLinearVec3();
    }

    void VelocityConstraint::setLinearVelocity(const iDynTree::Vector3& newLinearVelocity)
    {
        m_twist.setLinearVec3(newLinearVelocity);
    }

    const iDynTree::Vector3& VelocityConstraint::getAngularVelocity() const
    {
        return m_twist.getAngularVec3();
    }

    void VelocityConstraint::setAngularVelocity(const iDynTree::Vector3& newAngularVelocity)
    {
        m_twist.setAngularVec3(newAngularVelocity);
    }

    const iDynTree::Twist& VelocityConstraint::getTwist() const
    {
        return m_twist;
    }

    void VelocityConstraint::setTwist(const iDynTree::Twist& newTwist)
    {
        m_twist = newTwist;
    }

    const double VelocityConstraint::getLinearVelocityWeight() const
    {
        return m_linearVelocityWeight;
    }

    void VelocityConstraint::setLinearVelocityWeight(const double newLinearVelocityWeight)
    {
        m_linearVelocityWeight = newLinearVelocityWeight;
    }

    const double VelocityConstraint::getAngularVelocityWeight() const
    {
        return m_angularVelocityWeight;
    }

    void VelocityConstraint::setAngularVelocityWeight(const double newAngularVelocityWeight)
    {
        m_angularVelocityWeight = newAngularVelocityWeight;
    }

}
}
