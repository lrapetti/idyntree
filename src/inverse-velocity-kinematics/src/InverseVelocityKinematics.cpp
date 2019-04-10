/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/InverseVelocityKinematics.h>
#include "InverseVelocityKinematicsData.h"
#include "VelocityConstraint.h"

#include <iDynTree/ModelIO/ModelLoader.h>

#include <cassert>

// TODO: directly access the raw data, thus removing the methods in IKData class
#define IVK_PIMPL(x) static_cast<internal::kinematics::InverseVelocityKinematicsData*>((x))

namespace iDynTree {

    InverseVelocityKinematics::InverseVelocityKinematics()
    : m_pimpl(0)
    {
        m_pimpl = new internal::kinematics::InverseVelocityKinematicsData();
    }

    InverseVelocityKinematics::~InverseVelocityKinematics()
    {
        if (m_pimpl)
        {
            delete IVK_PIMPL(m_pimpl);
            m_pimpl = 0;
        }
    }

    bool InverseVelocityKinematics::loadModelFromFile(const std::string & filename,
                                                      const std::string & filetype)
    {
        ModelLoader loader;
        if (!loader.loadModelFromFile(filename) || !loader.isValid()) {
            std::cerr << "[ERROR] iDynTree::InverseDynamics : Failed to load model from URDF file " << filename << std::endl;
            return false;
        }

        return setModel(loader.model());
    }

    bool InverseVelocityKinematics::setModel(const iDynTree::Model &model)
    {
        assert(m_pimpl);
        return IVK_PIMPL(m_pimpl)->setModel(model);
    }

    void InverseVelocityKinematics::clearProblem()
    {
        assert(m_pimpl);
        IVK_PIMPL(m_pimpl)->clearProblem();
    }

    bool InverseVelocityKinematics::setFloatingBaseOnFrameNamed(const std::string &floatingBaseFrameName)
    {
        assert(m_pimpl);
        return IVK_PIMPL(m_pimpl)->dynamics().setFloatingBase(floatingBaseFrameName);
    }

    bool InverseVelocityKinematics::setJointConfiguration(const std::string& jointName, const double jointConfiguration)
    {
        assert(m_pimpl);
        return IVK_PIMPL(m_pimpl)->setJointConfiguration(jointName, jointConfiguration);
    }

    bool InverseVelocityKinematics::setJointsConfiguration(const iDynTree::VectorDynSize& jointsConfiguration)
    {
        assert(m_pimpl);
        return IVK_PIMPL(m_pimpl)->setJointsConfiguration(jointsConfiguration);
    }

    bool InverseVelocityKinematics::setBasePose(const iDynTree::Transform& baseTransform)
    {
        assert(m_pimpl);
        return IVK_PIMPL(m_pimpl)->setBasePose(baseTransform);
    }

    bool InverseVelocityKinematics::setBasePose(const iDynTree::Position& basePosition, const iDynTree::Rotation& baseRotation)
    {
        assert(m_pimpl);
        return IVK_PIMPL(m_pimpl)->setBasePose(basePosition, baseRotation);
    }

    bool InverseVelocityKinematics::setConfiguration(const iDynTree::Transform& baseTransform, const iDynTree::VectorDynSize& jointsConfiguration)
    {
        assert(m_pimpl);
        return IVK_PIMPL(m_pimpl)->setConfiguration(baseTransform, jointsConfiguration);
    }

    bool InverseVelocityKinematics::setConfiguration(const iDynTree::Position& basePosition, const iDynTree::Rotation& baseRotation, const iDynTree::VectorDynSize& jointsConfiguration)
    {
        assert(m_pimpl);
        return IVK_PIMPL(m_pimpl)->setConfiguration(basePosition, baseRotation, jointsConfiguration);
    }

    void InverseVelocityKinematics::setResolutionMode(enum InverseVelocityKinematicsResolutionMode resolutionMode)
    {
        assert(m_pimpl);
        IVK_PIMPL(m_pimpl)->setResolutionMode(resolutionMode);
    }

    enum InverseVelocityKinematicsResolutionMode InverseVelocityKinematics::resolutionMode()
    {
        assert(m_pimpl);
        return IVK_PIMPL(m_pimpl)->resolutionMode();
    }

    void InverseVelocityKinematics::setRegularization(double regularizationWeight)
    {
        assert(m_pimpl);
        IVK_PIMPL(m_pimpl)->setRegularization(regularizationWeight);
    }

    double InverseVelocityKinematics::regularizationWeight()
    {
        assert(m_pimpl);
        return IVK_PIMPL(m_pimpl)->regularizationWeight();
    }

    bool InverseVelocityKinematics::addTarget(const std::string& linkName,
                                              const iDynTree::Vector3& linearVelocity,
                                              const iDynTree::Vector3& angularVelocity,
                                              const double linearWeight,
                                              const double angularWeight)
    {
        assert(m_pimpl);
        return IVK_PIMPL(m_pimpl)->addTarget(linkName,
                                             linearVelocity,
                                             angularVelocity,
                                             linearWeight,
                                             angularWeight);
    }

    bool InverseVelocityKinematics::addTarget(const std::string& linkName, const iDynTree::Twist& twist, const double linearWeight, const double angularWeight);

    bool InverseVelocityKinematics::addLinearVelocityTarget(const std::string& linkName, const iDynTree::Vector3& linearVelocity, const double linearWeight);

    bool InverseVelocityKinematics::addLinearVelocityTarget(const std::string& linkName, const iDynTree::Twist& twist, const double linearWeight);

    bool InverseVelocityKinematics::addAngularVelocityTarget(const std::string& linkName, const iDynTree::Vector3&  angularVelocity, const double angularWeight);

    bool InverseVelocityKinematics::addAngularVelocityTarget(const std::string& linkName, const iDynTree::Twist&  twist, const double angularWeight);
}
