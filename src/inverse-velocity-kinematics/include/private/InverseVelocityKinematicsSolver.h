/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_INTERNAL_INVERSEKINEMATICSSOLVER_H
#define IDYNTREE_INTERNAL_INVERSEKINEMATICSSOLVER_H

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>

namespace internal {
namespace kinematics {
	class InverseVelocityKinematicsSolver;
	class InverseVelocityKinematicsData;
	class VelocityConstraint;
}
}

class internal::kinematics::InverseVelocityKinematicsSolver {

    InverseVelocityKinematicsData& m_data;

    iDynTree::MatrixDynSize m_fullJacobianBuffer;
    iDynTree::VectorDynSize m_fullVelocityBuffer;
    iDynTree::VectorDynSize m_weightVectorBuffer;
    iDynTree::MatrixDynSize m_regularizationMatrixBuffer;

    void prepareForOptimization();

    void prepareFullVelocityVector();

    void prepareFullJacobianMatrix();

    void prepareWeightVector();

    bool solveWeightedPseudoInverse(iDynTree::MatrixDynSize matrix, iDynTree::VectorDynSize inputVector, iDynTree::VectorDynSize& outputVector, iDynTree::VectorDynSize weightVector, iDynTree::MatrixDynSize regularizationMatrix);

public:
    /*! Constructor
     * @param data reference to the InverseVelocityKinematicsData object
     */
    InverseVelocityKinematicsSolver(InverseVelocityKinematicsData& data);

    void initializeInternalData();

    bool solve();
};

#endif /* end of include guard: IDYNTREE_INTERNAL_INVERSEKINEMATICSSOLVER_H */
