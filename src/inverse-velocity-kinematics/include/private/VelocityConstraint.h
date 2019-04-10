/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_INTERNAL_VELOCITY_CONSTRAINT_H
#define IDYNTREE_INTERNAL_VELOCITY_CONSTRAINT_H

#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/InverseVelocityKinematics.h>

namespace internal {
    namespace kinematics {
        class VelocityConstraint;
    }
}

namespace iDynTree {
    class Twist; //TODO instead of using Vector3 for linear and angular velocity, we may prefere to use LinearVector3T and AngularVector3T
}

class internal::kinematics::VelocityConstraint {
public:

    enum VelocityConstraintType {
        VelocityConstraintTypeLinearVelocity,
        VelocityConstraintTypeAngularVelocity,
        VelocityConstraintTypeTwist,
        };

private:
    VelocityConstraint(const std::string& frameName, VelocityConstraintType type);

    VelocityConstraintType m_type;
    iDynTree::Twist m_twist;
    std::string m_frameName;
    double m_linearVelocityWeight;
    double m_angularVelocityWeight;

public:
    static VelocityConstraint linearVelocityConstraint(const std::string& frameName, const iDynTree::Vector3& linearVelocity, const double linearVelocityWeight=1.0);
    static VelocityConstraint angularVelocityConstraint(const std::string& frameName, const iDynTree::Vector3& angularVelocity, const double angularVelocityWeight=1.0);
    static VelocityConstraint TwistConstraint(const std::string& frameName,
                                              const iDynTree::Vector3& linearVelocity,
                                              const iDynTree::Vector3& angularVelocity,
                                              const double linearVelocityWeight=1.0,
                                              const double angularVelocityWeight=1.0);
    static VelocityConstraint TwistConstraint(const std::string& frameName,
                                              const iDynTree::Twist& twist,
                                              const double linearVelocityWeight=1.0,
                                              const double angularVelocityWeight=1.0);

    VelocityConstraint::VelocityConstraintType getType() const;
    const std::string& getFrameName() const;

    bool hasLinearVelocityConstraint() const;
    bool hasAngularVelocityConstraint() const;

    const iDynTree::Vector3& getLinearVelocity() const;
    void setLinearVelocity(const iDynTree::Vector3& newLinearVelocity);

    const iDynTree::Vector3& getAngularVelocity() const;
    void setAngularVelocity(const iDynTree::Vector3& newAngularVelocity);

    const iDynTree::Twist& getTwist() const;
    void setTwist(const iDynTree::Twist& newTwist);

    const double getLinearVelocityWeight() const;
    void setLinearVelocityWeight(const double newLinearVelocityWeight);

    const double getAngularVelocityWeight() const;
    void setAngularVelocityWeight(const double newAngularVelocityWeight);
};

#endif /* end of include guard: IDYNTREE_INTERNAL_VELOCITY_CONSTRAINT_H */
