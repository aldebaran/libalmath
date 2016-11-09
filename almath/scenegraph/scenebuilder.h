/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */

#ifndef LIB_ALMATH_SCENEGRAPH_SCENEBUILDER_H
#define LIB_ALMATH_SCENEGRAPH_SCENEBUILDER_H

#include <Eigen/Geometry>
#include <almath/api.h>

namespace AL {
class Mesh;

namespace Math {
struct Transform;
class Sphere;
class RoundedRectangle;
class Pill;
class Shape3D;
}
// Interface for a simple Scene builder.
//
// The scene is indeed simple:
//
// * all meshes have the same material
// * each node binds a single mesh, and a mesh is bound by a single node
// * there is no node hierarchy
// * there is no texture support (yet?)
//
//
// Possible future developments:
//
// * add texture support
// * add support for external meshes
// * add support for debugging markers (arrows, frames, lines, points)
// * add ids (and maybe scoped ids) to nodes so that one can target them later
//   (maybe add an id for individual shapes color)
// * add animation support: let one
//    * move
//    * change color
//    * toggle visibility
//   of stuff at specific time
//
// One can imagine several implementations of this interface:
//
// * build the scene in a collada file
// * animate a collada scene by dumping collada animations
//   (though we lack a nice portable collada animation viewer, maybe blender
//    would fit the bill for specifically created collada files?)
// * build  the scene in a node of the Ogre scene graph (we do not need to
//   build the whole Ogre scene)
// * animate an ogre scene
// * idem with renderwidget?
// * write the animation in a ros bag
class ALMATH_API SceneBuilder {
 public:
  class Color {
   public:
    Color() : r(0.5f), g(0.5f), b(0.5f), a(0.5f) {}
    float r, g, b, a;
  };

  // a class for configuration which can be used by the SceneBuilder
  // implementations
  class Config {
   public:
    Config() : nbArcVertices(2) {}
    Color color;
    // the number of intermediate vertices on the are of a rounded box
    int nbArcVertices;
  };

  SceneBuilder(const Config &_config);
  SceneBuilder(const SceneBuilder &other);

  virtual ~SceneBuilder();

  const Config &getConfig() const;

  void add(const Mesh &mesh, const Eigen::Affine3f &tf);
  void add(const Mesh &mesh, const Math::Transform &tf);
  void add(const Math::Sphere &shape, const Math::Transform &tf);
  void add(const Math::RoundedRectangle &shape, const Math::Transform &tf);
  void add(const Math::Pill &shape, const Math::Transform &tf);

  void add(const Math::Shape3D &shape, const Math::Transform &tf);

 protected:
  virtual void xAddMesh(const Mesh &mesh, const Eigen::Affine3f &tf) = 0;

 private:
  Config _config;
};
}

#endif
