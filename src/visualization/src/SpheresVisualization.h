/*
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
#ifndef IDYNTREE_SPHERES_VISUALIZATION_H
#define IDYNTREE_SPHERES_VISUALIZATION_H

#include <iDynTree/Visualizer.h>

#include <vector>
#include <irrlicht.h>

namespace iDynTree {

    class SpheresVisualization : public ISpheresVisualization {

        typedef struct {
            Position center;
            double radius;
            ColorViz color = ColorViz(1.0, 0.0, 0.0, 1.0);
            irr::scene::ISceneNode * visualizationNode = nullptr;
        } SpheresProperties;

        ColorViz m_spheresDefaultColor{ColorViz(1.0, 0.0, 0.0, 1.0)};

        std::vector<SpheresProperties> m_spheres;

        irr::scene::ISceneManager* m_smgr;

        double m_polyCount,

        void drawSphere(size_t sphereIndex);

        void drawAll();

    public:

        SpheresVisualization();

        SpheresVisualization(const SpheresVisualization& other) = delete;

        SpheresVisualization& operator=(const SpheresVisualization& other) = delete;

        void init(irr::scene::ISceneManager* smgr);

        void close();

        virtual ~SpheresVisualization() override;

        virtual size_t addSphere(const Position & center, const double radius) override;

        virtual size_t getNrOfSpheres() const override;

        virtual bool getSphere(size_t sphereIndex, Position & currentCenter,
                               double & currentRadius) const override;

        virtual bool updateSphere(size_t sphereIndex, const Position & center, const double radius) override;

        virtual bool setSphereColor(size_t sphereIndex, const ColorViz & sphereColor) override;

        virtual void setSpheresDefaultColor(const ColorViz &sphereColor) override;

        virtual void setSpheresColor(const ColorViz &sphereColor) override;

        virtual bool setSpheresPolygonCount(const double polyCount) override;
        
    };

}

#endif // IDYNTREE_SPHERES_VISUALIZATION_H
