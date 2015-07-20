/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */

#include <almath/scenegraph/scenebuilder.h>
#include <almath/scenegraph/mesh.h>
#include <almath/scenegraph/meshfactory.h>
#include <almath/geometrics/shapes3d.h>
#include <almath/types/altransform.h>

namespace AL {

class SceneBuilderShape3DVisitor : public Math::NotImplementedShape3DVisitor {
 public:
  SceneBuilderShape3DVisitor(SceneBuilder &builder, const Math::Transform &tf)
      : builder(builder), tf(tf) {}
  void visit(const Math::Pill &pShape) const { builder.add(pShape, tf); }
  void visit(const Math::Sphere &pShape) const { builder.add(pShape, tf); }
  void visit(const Math::RoundedRectangle &pShape) const {
    builder.add(pShape, tf);
  }

 private:
  SceneBuilder &builder;
  const Math::Transform &tf;
};

SceneBuilder::SceneBuilder(const Config &config) : _config(config) {}

SceneBuilder::SceneBuilder(const SceneBuilder &other)
    : _config(other._config) {}

SceneBuilder::~SceneBuilder() {}

const SceneBuilder::Config &SceneBuilder::getConfig() const { return _config; }
void SceneBuilder::add(const Mesh &mesh, const Eigen::Affine3f &tf) {
  if (mesh.withTexCoords()) {
    throw std::invalid_argument(
        "SceneBuilder does not support meshes with texture coordinates");
  }
  xAddMesh(mesh, tf);
}

void SceneBuilder::add(const Mesh &mesh, const Math::Transform &tf) {
  Eigen::Map<const Eigen::Matrix<float, 3, 4, Eigen::RowMajor> > m(&tf.r1_c1);
  xAddMesh(mesh, Eigen::Affine3f(m));
}

void SceneBuilder::add(const Math::Sphere &shape, const Math::Transform &tf) {
  add(createRoundedBoxMesh(0.f, 0.f, 0.f, shape.getRadius(),
                           _config.nbArcVertices),
      tf);
}

void SceneBuilder::add(const Math::RoundedRectangle &shape,
                       const Math::Transform &tf) {
  add(createRoundedBoxMesh(shape.getHalfExtentX(), shape.getHalfExtentY(), 0.f,
                           shape.getRadius(), _config.nbArcVertices),
      tf);
}

void SceneBuilder::add(const Math::Pill &shape, const Math::Transform &tf) {
  add(createRoundedBoxMesh(0.f, 0.f, shape.getHalfExtent(), shape.getRadius(),
                           _config.nbArcVertices),
      tf);
}

void SceneBuilder::add(const Math::Shape3D &shape, const Math::Transform &tf) {
  SceneBuilderShape3DVisitor vis(*this, tf);
  shape.accept(vis);
}
}
