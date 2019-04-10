/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */


#ifndef IDYNTREE_INVERSEVELOCITYKINEMATICS_H
#define IDYNTREE_INVERSEVELOCITYKINEMATICS_H

#include <string>
#include <vector>

namespace iDynTree {
    class VectorDynSize;
    class Twist;
    class Transform;
    class Position;
    class Rotation;
    class Model;
    class Vector3;
}

namespace iDynTree {
    class InverseVelocityKinematics;

    enum  InverseVelocityKinematicsResolutionMode {
        InverseVelocityKinematicsSolveAsPseudoinverse
    };
}

class iDynTree::InverseVelocityKinematics
{

public:
    /*!
     * Default constructor
     */
    InverseVelocityKinematics();

    /*!
     * Destructor
     */
    ~InverseVelocityKinematics();

    /*!
     * @brief Loads the kinematic model from an external file.
     *
     * @param[in] urdfFile path to the urdf file describing the model
     * @param[in] filetype (optional) explicit definition of the type of the loaded file. Only "urdf" is supported at the moment.
     * @return true if successful. False otherwise
     */
    bool loadModelFromFile(const std::string & filename,
                           const std::string & filetype="urdf");

    /*!
     * @brief set the kinematic model to be used in the optimization

     *
     * @note you may want to simplify the model by calling
     * loadReducedModelFromFullModel method contained in the ModelLoader class.
     *
     * @param model the kinematic model to be used in the optimization
     * @return true if successful. False otherwise
     */
    bool setModel(const iDynTree::Model &model);

    //TODO joint velocity limits handling
    //setJointLimits()
    //getJointLimits()

    void clearProblem();

    bool setFloatingBaseOnFrameNamed(const std::string &floatingBaseFrameName);



    bool setJointConfiguration(const std::string& jointName, const double jointConfiguration);

    bool setJointsConfiguration(const iDynTree::VectorDynSize& jointsConfiguration);

    bool setBasePose(const iDynTree::Transform& baseTransform);

    bool setBasePose(const iDynTree::Position& basePosition, const iDynTree::Rotation& baseRotation);

    bool setConfiguration(const iDynTree::Transform& baseTransform, const iDynTree::VectorDynSize& jointsConfiguration);

    bool setConfiguration(const iDynTree::Position& basePosition, const iDynTree::Rotation& baseRotation, const iDynTree::VectorDynSize& jointsConfiguration);



    void setResolutionMode(enum InverseVelocityKinematicsResolutionMode resolutionMode);

    enum InverseVelocityKinematicsResolutionMode resolutionMode();

    void setRegularization(double regularizationWeight);

    double regularizationWeight();



    bool addTarget(const std::string& linkName, const iDynTree::Vector3& linearVelocity, const iDynTree::Vector3& angularVelocity, const double linearWeight=1.0, const double angularWeight=1.0);

    bool addTarget(const std::string& linkName, const iDynTree::Twist& twist, const double linearWeight=1.0, const double angularWeight=1.0);

    bool addLinearVelocityTarget(const std::string& linkName, const iDynTree::Vector3& linearVelocity, const double linearWeight=1.0);

    bool addLinearVelocityTarget(const std::string& linkName, const iDynTree::Twist& twist, const double linearWeight=1.0);

    bool addAngularVelocityTarget(const std::string& linkName, const iDynTree::Vector3&  angularVelocity, const double angularWeight=1.0);

    bool addAngularVelocityTarget(const std::string& linkName, const iDynTree::Twist&  twist, const double angularWeight=1.0);



    //TODO at the moment only target are implemented, no constraints
    // addFrameVelocityConstraint
    // addFrameLinearVelocityConstraint
    // addFrameAngularVelocityConstraint



    bool updateTarget(const std::string& linkName, const iDynTree::Vector3& linearVelocity, const iDynTree::Vector3&  angularVelocity, const double linearWeight=1.0, const double angularWeight=1.0);

    bool updateTarget(const std::string& linkName, const iDynTree::Twist& twist, const double linearWeight=1.0, const double angularWeight=1.0);

    bool updateLinearVelocityTarget(const std::string& linkName, const iDynTree::Vector3& linearVelocity, const double linearWeight=1.0);

    bool updateAngularVelocityTarget(const std::string& linkName, const iDynTree::Vector3&  angularVelocity, const double angularWeight=1.0);



    bool getVelocitySolution(iDynTree::Twist& baseVelocity, iDynTree::VectorDynSize& jointsVelocity);

    bool getJointsVelocitySolution(iDynTree::VectorDynSize& jointsVelocity);

    bool getBaseVelocitySolution(iDynTree::Twist& baseVelocity);

    bool getBaseVelocitySolution(iDynTree::Vector3& linearVelocity, iDynTree::Vector3& angularVelocity);

    bool solve();

    void clearProbelm();



    const Model & model() const;

private:
    void* m_pimpl; /*!< private implementation */
};

#endif /* end of include guard: IDYNTREE_INVERSEVELOCITYKINEMATICS_H */
