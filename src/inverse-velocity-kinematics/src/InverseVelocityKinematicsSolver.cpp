/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "InverseVelocityKinematicsSolver.h"
#include "InverseVelocityKinematicsData.h"
#include "VelocityConstraint.h"

#include <iDynTree/Core/EigenHelpers.h>

namespace internal {
namespace kinematics {

    InverseVelocityKinematicsSolver::InverseVelocityKinematicsSolver(InverseVelocityKinematicsData& data)
    : m_data(data)
    {}

    void InverseVelocityKinematicsSolver::initializeInternalData()
    {
        //prepare buffers
        m_fullJacobianBuffer.resize(m_data.m_numberOfTargetVariables, 6 + m_data.m_dofs);
        m_fullJacobianBuffer.zero();
        m_fullVelocityBuffer.resize(m_data.m_numberOfTargetVariables);
        m_fullVelocityBuffer.zero();
        m_weightVectorBuffer.resize(m_data.m_numberOfTargetVariables);
        m_weightVectorBuffer.zero();

        //TODO need smarter way to create identity matrix in iDynTree?
        m_regularizationMatrixBuffer.resize(m_data.m_numberOfTargetVariables, m_data.m_numberOfTargetVariables);
        m_regularizationMatrixBuffer.zero();
        for (int i=0; i<m_data.m_numberOfTargetVariables; i++)
            m_regularizationMatrixBuffer.setVal(i, i, m_data.m_regularizationWeight);
    }

    bool InverseVelocityKinematicsSolver::solve()
    {
        prepareForOptimization();

        iDynTree::VectorDynSize nu;
        solveWeightedPseudoInverse(m_fullJacobianBuffer, m_fullVelocityBuffer, nu, m_weightVectorBuffer, m_regularizationMatrixBuffer);

        m_data.m_baseVelocityResult.setVal(0, nu.getVal(0));
        m_data.m_baseVelocityResult.setVal(1, nu.getVal(1));
        m_data.m_baseVelocityResult.setVal(2, nu.getVal(2));
        m_data.m_baseVelocityResult.setVal(3, nu.getVal(3));
        m_data.m_baseVelocityResult.setVal(4, nu.getVal(4));
        m_data.m_baseVelocityResult.setVal(5, nu.getVal(5));

        for (int k=0; k<m_data.m_dofs; k++)
        {
            m_data.m_jointVelocityResult.setVal(k, nu.getVal(k + 6));
        }

        return true;
    }

    void InverseVelocityKinematicsSolver::prepareForOptimization()
    {
        prepareFullVelocityVector();
        prepareFullJacobianMatrix();
        prepareWeightVector();
    }

    void InverseVelocityKinematicsSolver::prepareFullVelocityVector()
    {
        unsigned int vectorIndex = 0;

        //TODO this should be done by filling the sub-blocks trough Eigen maps
        for (VelocityMap::const_iterator target = m_data.m_velocityTargets.begin();
                     target != m_data.m_velocityTargets.end(); ++target) {
            if (target->second.getType() == VelocityConstraint::VelocityConstraintTypeTwist)
            {
                m_fullVelocityBuffer.setVal(vectorIndex    , target->second.getLinearVelocity().getVal(0));
                m_fullVelocityBuffer.setVal(vectorIndex + 1, target->second.getLinearVelocity().getVal(1));
                m_fullVelocityBuffer.setVal(vectorIndex + 2, target->second.getLinearVelocity().getVal(2));
                m_fullVelocityBuffer.setVal(vectorIndex + 3, target->second.getAngularVelocity().getVal(0));
                m_fullVelocityBuffer.setVal(vectorIndex + 4, target->second.getAngularVelocity().getVal(1));
                m_fullVelocityBuffer.setVal(vectorIndex + 5, target->second.getAngularVelocity().getVal(2));
                vectorIndex += 6;
            }
            else if (target->second.getType() == VelocityConstraint::VelocityConstraintTypeLinearVelocity)
            {
                m_fullVelocityBuffer.setVal(vectorIndex    , target->second.getLinearVelocity().getVal(0));
                m_fullVelocityBuffer.setVal(vectorIndex + 1, target->second.getLinearVelocity().getVal(1));
                m_fullVelocityBuffer.setVal(vectorIndex + 2, target->second.getLinearVelocity().getVal(2));
                vectorIndex += 3;
            }
            else if (target->second.getType() == VelocityConstraint::VelocityConstraintTypeAngularVelocity)
            {
                m_fullVelocityBuffer.setVal(vectorIndex    , target->second.getAngularVelocity().getVal(0));
                m_fullVelocityBuffer.setVal(vectorIndex + 1, target->second.getAngularVelocity().getVal(1));
                m_fullVelocityBuffer.setVal(vectorIndex + 2, target->second.getAngularVelocity().getVal(2));
                vectorIndex += 3;
            }
        }
    }

    void InverseVelocityKinematicsSolver::prepareFullJacobianMatrix()
    {
        unsigned int rowIndex = 0;

        iDynTree::iDynTreeEigenMatrixMap fullJacobian = iDynTree::toEigen(m_fullJacobianBuffer);
        iDynTree::MatrixDynSize frameJacobian(6, 6 + m_data.m_dofs);

        for (VelocityMap::const_iterator target = m_data.m_velocityTargets.begin();
                     target != m_data.m_velocityTargets.end(); ++target) {

            m_data.m_dynamics.getFrameFreeFloatingJacobian(target->second.getFrameName(), frameJacobian);

            if (target->second.getType() == VelocityConstraint::VelocityConstraintTypeTwist)
            {
                fullJacobian.block(rowIndex, 0, 6, 6 + m_data.m_dofs) = iDynTree::toEigen(frameJacobian);
                rowIndex += 6;
            }
            else if (target->second.getType() == VelocityConstraint::VelocityConstraintTypeLinearVelocity)
            {
                fullJacobian.block(rowIndex, 0, 3, 6 + m_data.m_dofs) = iDynTree::toEigen(frameJacobian).topRows(3);
                rowIndex += 3;
            }
            else if (target->second.getType() == VelocityConstraint::VelocityConstraintTypeAngularVelocity)
            {
                fullJacobian.block(rowIndex, 0, 3, 6 + m_data.m_dofs) = iDynTree::toEigen(frameJacobian).bottomRows(3);
                rowIndex += 3;
            }
        }
    }

    void InverseVelocityKinematicsSolver::prepareWeightVector()
    {
        unsigned int vectorIndex = 0;

        //TODO this should be done by filling the sub-blocks trough Eigen maps
        for (VelocityMap::const_iterator target = m_data.m_velocityTargets.begin();
                     target != m_data.m_velocityTargets.end(); ++target) {
            if (target->second.getType() == VelocityConstraint::VelocityConstraintTypeTwist)
            {
                m_weightVectorBuffer.setVal(vectorIndex    , target->second.getLinearVelocityWeight());
                m_weightVectorBuffer.setVal(vectorIndex + 1, target->second.getLinearVelocityWeight());
                m_weightVectorBuffer.setVal(vectorIndex + 2, target->second.getLinearVelocityWeight());
                m_weightVectorBuffer.setVal(vectorIndex + 3, target->second.getAngularVelocityWeight());
                m_weightVectorBuffer.setVal(vectorIndex + 4, target->second.getAngularVelocityWeight());
                m_weightVectorBuffer.setVal(vectorIndex + 5, target->second.getAngularVelocityWeight());
                vectorIndex += 6;
            }
            else if (target->second.getType() == VelocityConstraint::VelocityConstraintTypeLinearVelocity)
            {
                m_weightVectorBuffer.setVal(vectorIndex    , target->second.getLinearVelocityWeight());
                m_weightVectorBuffer.setVal(vectorIndex + 1, target->second.getLinearVelocityWeight());
                m_weightVectorBuffer.setVal(vectorIndex + 2, target->second.getLinearVelocityWeight());
                vectorIndex += 3;
            }
            else if (target->second.getType() == VelocityConstraint::VelocityConstraintTypeAngularVelocity)
            {
                m_weightVectorBuffer.setVal(vectorIndex    , target->second.getAngularVelocityWeight());
                m_weightVectorBuffer.setVal(vectorIndex + 1, target->second.getAngularVelocityWeight());
                m_weightVectorBuffer.setVal(vectorIndex + 2, target->second.getAngularVelocityWeight());
                vectorIndex += 3;
            }
        }
    }

    bool InverseVelocityKinematicsSolver::solveWeightedPseudoInverse(iDynTree::MatrixDynSize matrix, iDynTree::VectorDynSize inputVector, iDynTree::VectorDynSize& outputVector, iDynTree::VectorDynSize weightVector, iDynTree::MatrixDynSize regularizationMatrix)
    {
        if (inputVector.size() != matrix.rows() || inputVector.size() != regularizationMatrix.rows() || inputVector.size() != regularizationMatrix.cols())
            return false;

        outputVector.resize(matrix.cols());

        Eigen::DiagonalMatrix<double,Eigen::Dynamic> weightInverse(weightVector.size());
        weightInverse = Eigen::DiagonalMatrix<double,Eigen::Dynamic>(iDynTree::toEigen(weightVector)).inverse();

        iDynTree::toEigen(outputVector) = (iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix() * iDynTree::toEigen(matrix) + iDynTree::toEigen(regularizationMatrix)).inverse() * iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix() * iDynTree::toEigen(inputVector);

        return true;
    }

}
}
