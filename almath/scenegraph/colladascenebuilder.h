/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */

#ifndef LIB_ALMATH_SCENEGRAPH_COLLADASCENEBUILDER_H
#define LIB_ALMATH_SCENEGRAPH_COLLADASCENEBUILDER_H

#include <almath/scenegraph/scenebuilder.h>
#include <iosfwd>

namespace AL {

// pimpl
class ColladaSceneBuilderP;

// Build the scene in memory in the collada format then write it in
// a collada file.
//
// Note that this class will create a full collada file.
// Maybe a version which opens an existing collada file and
// adds nodes to it would be more useful.
// However, one should then care about id collisions.
class ColladaSceneBuilder : public SceneBuilder {
 public:
  ColladaSceneBuilder(const Config &config = Config());
  ~ColladaSceneBuilder();

  void xAddMesh(const Mesh &mesh, const Eigen::Affine3f &tf);
  friend std::ostream &operator<<(std::ostream &os,
                                  const ColladaSceneBuilder &builder);
  ColladaSceneBuilder(const ColladaSceneBuilder &other);
  ColladaSceneBuilder &operator=(const ColladaSceneBuilder &other);

 private:
  ColladaSceneBuilderP *_p;
};
}

#endif
