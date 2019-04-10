/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "InverseVelocityKinematicsData.h"
#include "InverseVelocityKinematicsSolver.h"
#include "VelocityConstraint.h"

#include <iDynTree/Core/EigenHelpers.h>

namespace internal {
namespace kinematics {

    InverseVelocityKinematicsData::InverseVelocityKinematicsData(const InverseVelocityKinematicsData&)
    {}
    InverseVelocityKinematicsData& InverseVelocityKinematicsData::operator=(const InverseVelocityKinematicsData&) { return *this; }

    InverseVelocityKinematicsData::InverseVelocityKinematicsData()
        : m_dofs(0)
        , m_resolutionMode(iDynTree::InverseVelocityKinematicsSolveAsPseudoinverse)
        , m_numberOfTargetVariables(0)
        , m_regularizationWeight(1E-8)
        , m_problemInitialized(false)
        , m_solver(new internal::kinematics::InverseVelocityKinematicsSolver(*this))
    {
        //These variables are touched only once.
        m_state.worldGravity.zero();
    }

    void InverseVelocityKinematicsData::updateConfiguration()
    {
        m_dynamics.setRobotState(m_state.basePose,
                                 m_state.jointsConfiguration,
                                 m_state.baseTwist,
                                 m_state.jointsVelocity,
                                 m_state.worldGravity);
    }

    void InverseVelocityKinematicsData::prepareForSolver()
    {
        //TODO here we can call the solvere prepare matrices and vectors
        //TODO prepare initial condition
        //TODO check limits
    }

    void InverseVelocityKinematicsData::computeProblemSizeAndResizeBuffers()
    {
        computeTargetSize();

        m_solver->initializeInternalData();

        m_problemInitialized = true;
    }

    bool InverseVelocityKinematicsData::setModel(const iDynTree::Model& model)
    {
        m_dofs = model.getNrOfDOFs();
        m_model = model;

        bool result = m_dynamics.loadRobotModel(model);
        if (!result || !m_dynamics.isValid()) {
            std::cerr << "[ERROR] Error loading robot model" << std::endl;
            return false;
        }

        clearProblem();

        updateConfiguration();

        return true;
    }

    void InverseVelocityKinematicsData::clearProblem()
    {
        m_state.jointsConfiguration.resize(m_dofs);
        m_state.jointsConfiguration.zero();

        m_state.jointsVelocity.resize(m_dofs);
        m_state.jointsVelocity.zero();

        m_state.basePose.setPosition(iDynTree::Position(0, 0, 0));
        m_state.basePose.setRotation(iDynTree::Rotation::Identity());

        m_baseVelocityResult.zero();

        m_jointVelocityResult.resize(m_dofs);
        m_jointVelocityResult.zero();

        m_state.baseTwist.zero();

        m_velocityTargets.clear();

        m_problemInitialized = false;
    }

    bool InverseVelocityKinematicsData::addTarget(const internal::kinematics::VelocityConstraint& frameVelocityTarget)
    {
        int frameIndex = m_dynamics.getFrameIndex(frameVelocityTarget.getFrameName());
        if (frameIndex < 0)
            return false;

        std::pair<VelocityMap::iterator, bool> result = m_velocityTargets.insert(VelocityMap::value_type(frameIndex, frameVelocityTarget));

        m_problemInitialized = false;

        return result.second;
    }

    VelocityMap::iterator InverseVelocityKinematicsData::getTargetRefIfItExists(const std::string targetFrameName)
    {
        // The error for this check is already printed in getFrameIndex
        int frameIndex = m_dynamics.getFrameIndex(targetFrameName);
        if (frameIndex == iDynTree::FRAME_INVALID_INDEX)
            return m_velocityTargets.end();

        // Find the target (if this fails, it will return m_targets.end()
        return m_velocityTargets.find(frameIndex);
    }

    void InverseVelocityKinematicsData::updateLinearVelocityTarget(VelocityMap::iterator target, iDynTree::Vector3 newLinearVelocity, double newLinearVelocityWeight)
    {
        assert(target != velocityTargets.end());
        target->second.setLinearVelocity(newLinearVelocity);
        target->second.setLinearVelocityWeight(newLinearVelocityWeight);
    }

    void InverseVelocityKinematicsData::updateAngularVelocityTarget(VelocityMap::iterator target, iDynTree::Vector3 newLAngularVelocity, double newAngularVelocityWeight)
    {
        assert(target != velocityTargets.end());
        target->second.setAngularVelocity(newLAngularVelocity);
        target->second.setAngularVelocityWeight(newAngularVelocityWeight);
    }

    bool InverseVelocityKinematicsData::setJointConfiguration(const std::string& jointName, const double jointConfiguration)
    {
        iDynTree::JointIndex jointIndex = m_dynamics.model().getJointIndex(jointName);
        if (jointIndex == iDynTree::JOINT_INVALID_INDEX) return false;
        m_state.jointsConfiguration(jointIndex) = jointConfiguration;
        updateConfiguration();
        return true;
    }

    bool InverseVelocityKinematicsData::setJointsConfiguration(const iDynTree::VectorDynSize& jointsConfiguration)
    {
        assert(pImpl->state.jointsConfiguration.size() == jointsConfiguration.size());
        m_state.jointsConfiguration = jointsConfiguration;
        updateConfiguration();
        return true;
    }

    bool InverseVelocityKinematicsData::setBasePose(const iDynTree::Transform& baseTransform)
    {
        m_state.basePose = baseTransform;
        updateConfiguration();
        return true;
    }

    bool InverseVelocityKinematicsData::setBasePose(const iDynTree::Vector3& basePosition, const iDynTree::Rotation& baseRotation)
    {
        iDynTree::Position _basePosition;
        iDynTree::toEigen(_basePosition) = iDynTree::toEigen(basePosition);
        m_state.basePose.setPosition(_basePosition);
        m_state.basePose.setRotation(baseRotation);
        updateConfiguration();
        return true;
    }

    bool InverseVelocityKinematicsData::setConfiguration(const iDynTree::Transform& baseTransform, const iDynTree::VectorDynSize& jointsConfiguration)
    {
        if (!(setJointsConfiguration(jointsConfiguration) && setBasePose(baseTransform)))
            return false;
        updateConfiguration();
        return true;
    }

    bool InverseVelocityKinematicsData::setConfiguration(const iDynTree::Vector3& basePosition, const iDynTree::Rotation& baseRotation, const iDynTree::VectorDynSize& jointsConfiguration)
    {
        if (!(setJointsConfiguration(jointsConfiguration) && setBasePose(basePosition, baseRotation)))
            return false;
        updateConfiguration();
        return true;
    }

    void InverseVelocityKinematicsData::setResolutionMode(iDynTree::InverseVelocityKinematicsResolutionMode resolutionMode)
    {
        m_resolutionMode = resolutionMode;
    }

    void InverseVelocityKinematicsData::setRegularization(double regularizationWeight)
    {
        m_regularizationWeight = regularizationWeight;
    }

    enum iDynTree::InverseVelocityKinematicsResolutionMode InverseVelocityKinematicsData::resolutionMode()
    {
        return m_resolutionMode;
    }

    double InverseVelocityKinematicsData::regularizationWeight()
    {
        return m_regularizationWeight;
    }

    bool InverseVelocityKinematicsData::solveProblem()
    {
        //TODO add if solver is null create it

        if (!m_problemInitialized) {
            computeProblemSizeAndResizeBuffers();
        }

        prepareForSolver();

        //TODO add solver solving and check if it is solved
        if (m_solver->solve())
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    iDynTree::KinDynComputations& InverseVelocityKinematicsData::dynamics()
    {
        return m_dynamics;
    }

    void InverseVelocityKinematicsData::computeTargetSize()
    {
        m_numberOfTargetVariables = 0;
        for (VelocityMap::const_iterator target = m_velocityTargets.begin();
                     target != m_velocityTargets.end(); ++target) {
            if (target->second.getType() == VelocityConstraint::VelocityConstraintTypeTwist)
            {
                m_numberOfTargetVariables += 6;
            }
            else
            {
                m_numberOfTargetVariables += 3;
            }
        }
    }
}

}
