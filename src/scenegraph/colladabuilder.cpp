/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */

#include <almath/scenegraph/colladabuilder.h>
#include <almath/scenegraph/mesh.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/format.hpp>

namespace {
template <typename T>
std::string toString(const std::vector<T> &data) {
  std::ostringstream ss;
  std::copy(data.begin(), data.end(), std::ostream_iterator<float>(ss, " "));
  return ss.str();
}
}

namespace AL {

ColladaBuilder::ptree &ColladaBuilder::addEffectColor(ptree &parent,
                                                      const std::string &id,
                                                      float red, float green,
                                                      float blue, float alpha) {
  ptree effect;
  effect.put("<xmlattr>.id", id);
  effect.put("profile_COMMON.technique.phong.diffuse.color",
             boost::str(boost::format("%1% %2% %3% %4%") % red % green % blue %
                        alpha));
  return parent.add_child("effect", effect);
}

ColladaBuilder::ptree &ColladaBuilder::addMaterial(
    ptree &parent, const std::string &id, const std::string &effect_id) {
  ptree material;
  material.put("<xmlattr>.id", id);
  material.put("instance_effect.<xmlattr>.url", "#" + effect_id);
  return parent.add_child("material", material);
}
void ColladaBuilder::addMaterialEffectColor(ptree &root, float r, float g,
                                            float b, float a,
                                            const ::std::string &id) {
  addEffectColor(root.get_child("COLLADA.library_effects"), id + "-fx", r, g, b,
                 a);
  addMaterial(root.get_child("COLLADA.library_materials"), id, id + "-fx");
}

ColladaBuilder::ptree ColladaBuilder::createNode(const std::string &id,
                                                 const Eigen::Affine3f &tf) {
  ptree node;
  if (!id.empty()) {
    node.put("<xmlattr>.id", id);
    // node.put("node.<xmlattr>.name", id);
  }
  Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ",
                      "", "", "", "");
  std::ostringstream elems_row_major;
  elems_row_major << tf.matrix().format(fmt);
  node.put("matrix", elems_row_major.str());
  // node.put("matrix.<xmlattr>.sid", "matrix");
  return node;
}

ColladaBuilder::ptree &ColladaBuilder::addVisualScene(ptree &parent,
                                                      const std::string &id) {
  ptree visual_scene;
  visual_scene.put("<xmlattr>.id", id);
  return parent.add_child("visual_scene", visual_scene);
}

ColladaBuilder::ptree &ColladaBuilder::addInstanceVisualScene(
    ptree &parent, const std::string &scene_id) {
  ptree instance_visual_scene;
  instance_visual_scene.put("<xmlattr>.url", "#" + scene_id);
  return parent.add_child("instance_visual_scene", instance_visual_scene);
}

ColladaBuilder::ptree &ColladaBuilder::addNode(ptree &parent,
                                               const std::string &id,
                                               const Eigen::Affine3f &tf) {
  return parent.add_child("node", createNode(id, tf));
}

ColladaBuilder::ptree &ColladaBuilder::addSourceXYZ(
    ptree &parent, const std::vector<float> &data, const std::string &id) {
  ptree source;
  source.put("<xmlattr>.id", id);
  {
    ptree float_array(toString(data));
    float_array.put("<xmlattr>.count", data.size());
    float_array.put("<xmlattr>.id", id + "_array");
    source.add_child("float_array", float_array);
  }
  {
    ptree technique_common;
    ptree accessor;
    accessor.put("<xmlattr>.count", data.size() / 3);
    accessor.put("<xmlattr>.source", "#" + id + "_array");
    accessor.put("<xmlattr>.stride", 3);
    const char names[] = {'X', 'Y', 'Z'};
    for (const char *name = names; name != names + 3; ++name) {
      ptree param;
      param.put("<xmlattr>.name", *name);
      param.put("<xmlattr>.type", "float");
      accessor.add_child("param", param);
    }
    technique_common.add_child("accessor", accessor);
    source.add_child("technique_common", technique_common);
  }
  return parent.add_child("source", source);
}

ColladaBuilder::ptree &ColladaBuilder::addInput(ptree &parent,
                                                const std::string &semantic,
                                                const std::string &source_id,
                                                int offset) {
  ptree input;
  input.put("<xmlattr>.semantic", semantic);
  input.put("<xmlattr>.source", "#" + source_id);
  input.put("<xmlattr>.offset", offset);
  return parent.add_child("input", input);
}

ColladaBuilder::ptree &ColladaBuilder::addGeometryMesh(ptree &parent,
                                                       const std::string &id,
                                                       const Mesh &m) {
  assert(!m.withTexCoords());
  ptree geometry;
  geometry.put("<xmlattr>.id", id);
  {
    ptree mesh;
    addSourceXYZ(mesh, m.positions(), id + "_positions");
    addSourceXYZ(mesh, m.normals(), id + "_normals");
    {
      ptree vertices;
      vertices.put("<xmlattr>.id", id + "_vertices");
      addInput(vertices, "POSITION", id + "_positions", 0);
      mesh.add_child("vertices", vertices);
    }
    {
      ptree polylist;
      polylist.put("<xmlattr>.count", m.polygonVerticesCounts().size());
      // the symbolic name of the mesh material is "material"
      polylist.put("<xmlattr>.material", "material");
      addInput(polylist, "VERTEX", id + "_vertices", m.positionOffset);
      addInput(polylist, "NORMAL", id + "_normals", m.normalOffset);
      {
        ptree elem(toString(m.polygonVerticesCounts()));
        polylist.add_child("vcount", elem);
      }
      {
        ptree elem(toString(m.vertices()));
        polylist.add_child("p", elem);
      }
      mesh.add_child("polylist", polylist);
    }

    geometry.add_child("mesh", mesh);
  }

  return parent.add_child("geometry", geometry);
}

ColladaBuilder::ptree &ColladaBuilder::addInstanceGeometry(
    ptree &parent, const std::string &geometry_id,
    const std::string &material_id) {
  ptree instance_geometry;
  instance_geometry.put("<xmlattr>.url", "#" + geometry_id);
  instance_geometry.put(
      "bind_material.technique_common.instance_material.<xmlattr>.symbol",
      "material");
  instance_geometry.put(
      "bind_material.technique_common.instance_material.<xmlattr>.target",
      "#" + material_id);
  return parent.add_child("instance_geometry", instance_geometry);
}

ColladaBuilder::ptree ColladaBuilder::createDoc() {
  ptree root;
  ptree collada;
  collada.put("<xmlattr>.xmlns",
              "http://www.collada.org/2005/11/COLLADASchema");
  collada.put("<xmlattr>.version", "1.4.1");

  {
    ptree asset;
    asset.put("created", "2010-10-09T17:00:00");
    asset.put("modified", "2010-10-09T17:00:00");
    {
      ptree unit;
      unit.put("<xmlattr>.meter", "1");
      unit.put("<xmlattr>.name", "meter");
      asset.add_child("unit", unit);
    }
    collada.add_child("asset", asset);
  }
  // create all the libraries.
  // we should not do that since there can be zero or many of each of them.
  // Note that they can have an id too.
  collada.put("library_effects", "");
  collada.put("library_materials", "");
  collada.put("library_geometries", "");
  collada.put("library_nodes", "");
  collada.put("library_visual_scenes", "");
  collada.put("scene", "");
  root.add_child("COLLADA", collada);
  return root;
}
}
