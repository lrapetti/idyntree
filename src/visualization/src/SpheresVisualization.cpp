/*
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "SpheresVisualization.h"

#include "IrrlichtUtils.h"

#include <cassert>
#include <string>
#include <cmath>

using namespace iDynTree;



void SpheresVisualization::drawSphere(size_t sphereIndex)
{
    // Delete existing node
    if (m_spheres[sphereIndex].visualizationNode)
    {
        m_spheres[sphereIndex].visualizationNode->remove();
        m_spheres[sphereIndex].visualizationNode = nullptr;
    }

    float arrowHeight = static_cast<float>(std::abs(m_spheres[sphereIndex].modulus) * m_heightScale);
    float arrowWidth = static_cast<float>(m_radiusOffset +
                                          m_radiusMultiplier * std::abs(m_spheres[sphereIndex].modulus) * m_heightScale);


    auto arrowPosition = idyntree2irr_pos(m_spheres[sphereIndex].origin);

    iDynTree::Direction sphereDirection = (m_spheres[sphereIndex].modulus < 0) ?
                m_spheres[sphereIndex].direction.reverse() :
                m_spheres[sphereIndex].direction;

    iDynTree::Direction yDirection(0.0, 1.0, 0.0); //Arrows in irrlicht are pointing in the y direction by default

    iDynTree::Direction rotationAxis;

    iDynTree::toEigen(rotationAxis) = iDynTree::toEigen(yDirection).cross(iDynTree::toEigen(sphereDirection));
    rotationAxis.Normalize();

    double rotationAngle = std::acos(iDynTree::toEigen(sphereDirection).dot(iDynTree::toEigen(yDirection)));

    iDynTree::Rotation arrowRotationMatrix = iDynTree::Rotation::RotAxis(rotationAxis, rotationAngle);

    irr::core::vector3df arrowRotation = idyntree2irr_rot(arrowRotationMatrix);

    irr::scene::ISceneNode * frameNode = m_smgr->addEmptySceneNode();

    irr::scene::IMesh* arrowMesh = m_smgr->getGeometryCreator()->createArrowMesh(4, 8, arrowHeight, 0.9f * arrowHeight,
                                                                                 arrowWidth, 2.0f * arrowWidth);

    m_spheres[sphereIndex].visualizationNode = m_smgr->addMeshSceneNode(arrowMesh,frameNode);
    m_spheres[sphereIndex].visualizationNode->setPosition(arrowPosition);
    m_spheres[sphereIndex].visualizationNode->setRotation(arrowRotation);
    irr::video::SMaterial arrowColor;
    arrowColor.AmbientColor = idyntree2irrlicht(m_spheres[sphereIndex].color).toSColor();
    arrowColor.DiffuseColor = arrowColor.AmbientColor;
    m_spheres[sphereIndex].visualizationNode->getMaterial(0) = arrowColor;
    m_spheres[sphereIndex].visualizationNode->getMaterial(1) = arrowColor;


    arrowMesh->drop();
    arrowMesh = nullptr;
}

void SpheresVisualization::drawAll()
{
    for (size_t i = 0; i < m_spheres.size(); ++i) {
        drawSphere(i);
    }
}

SpheresVisualization::SpheresVisualization()
    : m_smgr(nullptr)
    , m_radiusOffset(0.01)
    , m_radiusMultiplier(0.0)
    , m_heightScale(1.0)
{

}

void SpheresVisualization::init(irr::scene::ISceneManager *smgr)
{
    assert(smgr);
    m_smgr = smgr;
    m_smgr->grab();
}

void SpheresVisualization::close()
{
    for (auto& sphere: m_spheres) {
        if (sphere.visualizationNode) {
            sphere.visualizationNode->removeAll();
            sphere.visualizationNode = nullptr;
        }
    }
    m_spheres.resize(0);

    if (m_smgr)
    {
        m_smgr->drop();
        m_smgr = nullptr;
    }
}

SpheresVisualization::~SpheresVisualization()
{
    close();
}

size_t SpheresVisualization::addSphere(const Position &origin, const Direction &direction, double modulus)
{
    SpheresProperties newSphere;
    newSphere.origin = origin;
    newSphere.direction = direction;
    newSphere.modulus = modulus;
    newSphere.color = m_spheresDefaultColor;

    m_spheres.push_back(newSphere);

    drawSphere(m_spheres.size()-1);

    return m_spheres.size() - 1;
}

size_t SpheresVisualization::addSphere(const Position &origin, const Sphere3 &components)
{
    return addSphere(origin, Direction(components(0), components(1), components(2)), toEigen(components).norm());
}

size_t SpheresVisualization::getNrOfSpheres() const
{
    return m_spheres.size();
}

bool SpheresVisualization::getSphere(size_t sphereIndex, Position &currentOrigin, Direction &currentDirection, double &currentModulus) const
{
    if (sphereIndex >= m_spheres.size()) {
        reportError("SpheresVisualization","getSphere","sphereIndex out of bounds.");
        return false;
    }

    currentDirection = m_spheres[sphereIndex].direction;
    currentOrigin = m_spheres[sphereIndex].origin;
    currentModulus = m_spheres[sphereIndex].modulus;

    return true;
}

bool SpheresVisualization::getSphere(size_t sphereIndex, Position &currentOrigin, Vector3 &components) const
{
    if (sphereIndex >= m_spheres.size()) {
        reportError("SpheresVisualization","getSphere","sphereIndex out of bounds.");
        return false;
    }

    currentOrigin = m_spheres[sphereIndex].origin;
    toEigen(components) = m_spheres[sphereIndex].modulus * toEigen(m_spheres[sphereIndex].direction);

    return true;
}

bool SpheresVisualization::updateSphere(size_t sphereIndex, const Position &origin, const Direction &direction, double modulus)
{
    if (sphereIndex >= m_spheres.size()) {
        reportError("SpheresVisualization","updateSphere","sphereIndex out of bounds.");
        return false;
    }

    m_spheres[sphereIndex].origin = origin;
    m_spheres[sphereIndex].direction = direction;
    m_spheres[sphereIndex].modulus = modulus;

    drawSphere(sphereIndex);

    return true;
}

bool SpheresVisualization::updateSphere(size_t sphereIndex, const Position &origin, const Vector3 &components)
{
    return updateSphere(sphereIndex, origin, Direction(components(0), components(1), components(2)), toEigen(components).norm());
}

void SpheresVisualization::setSpheresDefaultColor(const ColorViz &sphereColor)
{
    m_spheresDefaultColor = sphereColor;
}

void SpheresVisualization::setSpheresColor(const ColorViz &sphereColor)
{
    for(auto & sphere : m_spheres) {
        sphere.color = sphereColor;
    }

    drawAll();
}

bool SpheresVisualization::setSphereColor(size_t sphereIndex, const ColorViz &sphereColor)
{
    if (sphereIndex >= m_spheres.size()) {
        reportError("SpheresVisualization","setSphereColor","sphereIndex out of bounds.");
        return false;
    }

    m_spheres[sphereIndex].color = sphereColor;

    drawSphere(sphereIndex);

    return true;
}

bool SpheresVisualization::setSpheresAspect(double zeroModulusRadius, double modulusMultiplier, double heightScale)
{
    if (zeroModulusRadius < 0) {
        reportError("SpheresVisualization","setSpheresAspect","zeroModulusRadius is supposed to be non negative.");
        return false;
    }

    if (modulusMultiplier < 0) {
        reportError("SpheresVisualization","setSpheresAspect","modulusMultiplier is supposed to be non negative.");
        return false;
    }

    if (heightScale < 0) {
        reportError("SpheresVisualization","setSpheresAspect","heightScale is supposed to be non negative.");
        return false;
    }

    m_radiusOffset = zeroModulusRadius;
    m_radiusMultiplier = modulusMultiplier;
    m_heightScale = heightScale;

    drawAll();

    return true;
}
