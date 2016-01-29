/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */

#include <almath/scenegraph/colladascenebuilder.h>
#include <almath/scenegraph/mesh.h>
#include <almath/scenegraph/colladabuilder.h>
#include <sstream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/format.hpp>
#include <boost/version.hpp>

namespace {

static const std::string mymaterial = "mymaterial";
static const std::string myscene = "myscene";
}
namespace AL {

class ColladaSceneBuilderP {
 public:
  ColladaSceneBuilderP(const SceneBuilder::Config &config);
  ColladaSceneBuilderP(const ColladaSceneBuilderP &other);
  // counter to associate geometries and nodes with unique ids
  int id;
  // a ref to the config owned by the SceneBuilder
  const SceneBuilder::Config &config;
  ColladaBuilder builder;
  // the root xml element
  ColladaBuilder::ptree xml;
  // element where mesh data should be added
  ColladaBuilder::ptree &library_geometries;
  ColladaBuilder::ptree &visual_scene;
};

ColladaSceneBuilderP::ColladaSceneBuilderP(const SceneBuilder::Config &config)
    : id(0),
      config(config),
      xml(builder.createDoc()),
      library_geometries(xml.get_child("COLLADA.library_geometries")),
      visual_scene(builder.addVisualScene(
          xml.get_child("COLLADA.library_visual_scenes"), myscene)) {
  builder.addMaterialEffectColor(xml, config.color.r, config.color.g,
                                 config.color.b, config.color.a, mymaterial);
  builder.addInstanceVisualScene(xml.get_child("COLLADA.scene"), myscene);
}

ColladaSceneBuilderP::ColladaSceneBuilderP(const ColladaSceneBuilderP &other)
    : id(other.id),
      config(other.config),
      builder(other.builder),
      xml(other.xml),
      library_geometries(xml.get_child("COLLADA.library_geometries")),
      visual_scene(xml.get_child(
          "COLLADA.library_visual_scenes.visual_scene"))  // we know there is a
                                                          // single scene
{}

ColladaSceneBuilder::ColladaSceneBuilder(const SceneBuilder::Config &config)
    : SceneBuilder(config), _p(new ColladaSceneBuilderP(this->getConfig())) {}

ColladaSceneBuilder::ColladaSceneBuilder(const ColladaSceneBuilder &other)
    : SceneBuilder(other.getConfig()),
      _p(new ColladaSceneBuilderP(*other._p)) {}

ColladaSceneBuilder &ColladaSceneBuilder::operator=(
    const ColladaSceneBuilder &other) {
  if (&other != this) {
    SceneBuilder::operator=(other);
    ColladaSceneBuilderP *tmp = new ColladaSceneBuilderP(*other._p);
    delete _p;
    _p = tmp;
  }
  return *this;
}

ColladaSceneBuilder::~ColladaSceneBuilder() { delete _p; }

void ColladaSceneBuilder::xAddMesh(const Mesh &mesh,
                                   const Eigen::Affine3f &tf) {
  std::string geometry_id = boost::str(boost::format("mesh%1%") % _p->id++);
  ColladaBuilder &b = _p->builder;
  b.addGeometryMesh(_p->library_geometries, geometry_id, mesh);
  ColladaBuilder::ptree &node = b.addNode(_p->visual_scene, "", tf);
  b.addInstanceGeometry(node, geometry_id, mymaterial);
}

std::ostream &operator<<(std::ostream &os, const ColladaSceneBuilder &builder) {
#if BOOST_VERSION >= 105600
  boost::property_tree::xml_writer_settings<std::string> settings(' ', 4);
#else
  boost::property_tree::xml_writer_settings<char> settings(' ', 4);
#endif
  write_xml(os, builder._p->xml, settings);
  return os;
}
}
