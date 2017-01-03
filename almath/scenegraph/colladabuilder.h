/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */

#ifndef LIB_ALMATH_SCENEGRAPH_COLLADABUILDER_H
#define LIB_ALMATH_SCENEGRAPH_COLLADABUILDER_H

#include <almath/api.h>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <boost/property_tree/ptree_fwd.hpp>

namespace AL {

class Mesh;

// A low-level class for building a COLLADA tree as a
// boost::property_tree::ptree
//
// This class is for internal use, one is expected to use higher-level
// ColladaSceneBuilder instead.
//
// Possible future developments:
//  * maybe create an object for "id/target/url"
//  * let addGeometryMesh() support meshes with texture coordinates.
class ALMATH_API ColladaBuilder {
 public:
  typedef boost::property_tree::ptree ptree;
  ptree &addEffectColor(ptree &parent, const std::string &id, float red,
                        float green, float blue, float alpha);
  ptree &addMaterial(ptree &parent, const std::string &id,
                     const std::string &effect_id);
  void addMaterialEffectColor(ptree &root, float r, float g, float b, float a,
                              const ::std::string &id);
  ptree createNode(const std::string &id, const Eigen::Affine3f &tf);
  ptree &addNode(ptree &parent, const std::string &id,
                 const Eigen::Affine3f &tf);
  ptree &addVisualScene(ptree &parent, const std::string &id);
  ptree &addInstanceVisualScene(ptree &parent, const std::string &scene_id);
  ptree &addSourceXYZ(ptree &parent, const std::vector<float> &data,
                      const std::string &id);

  ptree &addInput(ptree &parent, const std::string &semantic,
                  const std::string &source_id, int offset);
  // add a geometry element containing a mesh. The mesh should not contain
  // texture coordinates.
  ptree &addGeometryMesh(ptree &parent, const std::string &id,
                         const Mesh &mesh);
  ptree &addInstanceGeometry(ptree &parent, const std::string &geometry_id,
                             const std::string &material_id);

  ptree createDoc();
};
}
#endif
