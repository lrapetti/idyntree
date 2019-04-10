/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_INTERNAL_INVERSEVELOCITYKINEMATICSDATA_H
#define IDYNTREE_INTERNAL_INVERSEVELOCITYKINEMATICSDATA_H

#include "InverseVelocityKinematicsSolver.h"
#include <iDynTree/InverseVelocityKinematics.h>

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Twist.h>

#include <map>

namespace internal {
namespace kinematics {

	class InverseVelocityKinematicsData;
	class VelocityConstraint;
	typedef std::map<int, internal::kinematics::VelocityConstraint> VelocityMap; //ordered map. Order is important

	class InverseVelocityKinematicsSolver;
}
}

class internal::kinematics::InverseVelocityKinematicsData {
    //Declare as friend the IVKSOLVER class so as it can access the private data
    friend class InverseVelocityKinematicsSolver;
    // and also inverseVelocityKineamtics
    friend class iDynTree::InverseVelocityKinematics;

    //forbid copy
    InverseVelocityKinematicsData(const InverseVelocityKinematicsData&);
    InverseVelocityKinematicsData& operator=(const InverseVelocityKinematicsData&);

public:
    iDynTree::Model m_model;
    iDynTree::KinDynComputations m_dynamics;
    size_t m_dofs;

    struct {
            iDynTree::VectorDynSize jointsConfiguration;
            iDynTree::Transform basePose;
            iDynTree::VectorDynSize jointsVelocity;
            iDynTree::Twist baseTwist;
            iDynTree::Vector3 worldGravity;
        } m_state;

    iDynTree::InverseVelocityKinematicsResolutionMode m_resolutionMode;

    VelocityMap m_velocityTargets;

    size_t m_numberOfTargetVariables;
    double m_regularizationWeight;

    iDynTree::Twist m_baseVelocityResult;
    iDynTree::VectorDynSize m_jointVelocityResult;

    bool m_problemInitialized;

    std::shared_ptr<internal::kinematics::InverseVelocityKinematicsSolver> m_solver;

    /*!
     * Default constructor
     */
    InverseVelocityKinematicsData();

    void updateConfiguration(); // add from PIMP

    void prepareForSolver(); //NEW

    void computeProblemSizeAndResizeBuffers(); // add from PIMP //TODO move things in solver

    bool setModel(const iDynTree::Model& model); //add from IVK

    void clearProblem(); //add from IVK

    //TODO add constraints
    // addFrameVelocityConstraint
    // addFrameLinearVelocityConstraint
    // addFrameAngularVelocityConstraint

    bool addTarget(const internal::kinematics::VelocityConstraint& frameVelocityTarget); //add from PIMP

    VelocityMap::iterator getTargetRefIfItExists(const std::string targetFrameName); //add from PIMP

    void updateLinearVelocityTarget(VelocityMap::iterator target, iDynTree::Vector3 newLinearVelocity, double newLinearVelocityWeight); //add from PIMP

    void updateAngularVelocityTarget(VelocityMap::iterator target, iDynTree::Vector3 newLAngularVelocity, double newAngularVelocityWeight); //add from PIMP

    bool setJointConfiguration(const std::string& jointName, const double jointConfiguration); //add from IVK
    bool setJointsConfiguration(const iDynTree::VectorDynSize& jointsConfiguration); //add from IVK
    bool setBasePose(const iDynTree::Transform& baseTransform); //add from IVK
    bool setBasePose(const iDynTree::Vector3& basePosition, const iDynTree::Rotation& baseRotation); //add from IVK
    bool setConfiguration(const iDynTree::Transform& baseTransform, const iDynTree::VectorDynSize& jointsConfiguration); //add from IVK
    bool setConfiguration(const iDynTree::Vector3& basePosition, const iDynTree::Rotation& baseRotation, const iDynTree::VectorDynSize& jointsConfiguration); //add from IVK

    void setResolutionMode(enum iDynTree::InverseVelocityKinematicsResolutionMode resolutionMode); //add from IVK
    void setRegularization(double regularizationWeight); //add from IVK

    enum iDynTree::InverseVelocityKinematicsResolutionMode resolutionMode(); //NEW
    double regularizationWeight(); //NEW

    void computeTargetSize(); //from PIMPL, //TODO maybe to be moved into the solver

    /*!
     * Access the Kinematics and Dynamics object used by the solver
     *
     * @return reference to the kinematics and dynamics object
     */
    iDynTree::KinDynComputations& dynamics(); //NEW

    bool solveProblem(); //NEW //TODO to be completed with the solver

};

#endif /* end of include guard: IDYNTREE_INTERNAL_INVERSEVELOCITYKINEMATICSDATA_H */
