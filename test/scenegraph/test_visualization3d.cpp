/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */

#include <almath/scenegraph/mesh.h>
#include <almath/scenegraph/scenebuilder.h>
#include <almath/scenegraph/meshfactory.h>
#include <almath/scenegraph/colladascenebuilder.h>
#include <almath/geometrics/shapes3d.h>
#include <gtest/gtest.h>
#include <almath/types/altransform.h>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/scoped_ptr.hpp>




using namespace AL;

boost::filesystem::path mktmpdir(const std::string &prefix) {
  using namespace boost::filesystem;
  path p = temp_directory_path() / unique_path(prefix + "%%%%-%%%%-%%%%-%%%%");
  create_directory(p);
  return p;
}


TEST(Mesh, init) {
  Mesh m;
  ASSERT_EQ(0u, m.polygonsNb());
  ASSERT_EQ(0u, m.positionsNb());
  ASSERT_EQ(0u, m.normalsNb());
  ASSERT_EQ(0u, m.texCoordsNb());
}

TEST(Mesh, double_begin) {
  Mesh m;
  m.begin(Mesh::POLYGON);
  // double begin
  EXPECT_ANY_THROW(m.begin(Mesh::POLYGON));
}

TEST(Mesh, no_begin) {
  Mesh m;
  EXPECT_ANY_THROW(m.vertex(0.f, 0.f, 0.f));
  ASSERT_EQ(0u, m.positionsNb());
  EXPECT_ANY_THROW(m.end());
}

TEST(Mesh, no_normal) {
  Mesh m;
  m.begin(Mesh::POLYGON);
  EXPECT_ANY_THROW(m.vertex(0.f, 0.f, 0.f));
  m.end();
}

TEST(Mesh, no_texcoord_ok) {
  Mesh m;
  EXPECT_EQ(2u, m.verticesStride());
  m.normal(0.f, 0.f, 1.f);
  m.begin(Mesh::POLYGON);
  m.vertex(0.f, 0.f, 0.f);
  m.end();
}

TEST(Mesh, no_texcoord_ko) {
  Mesh m(true);
  EXPECT_EQ(3u, m.verticesStride());
  m.normal(0.f, 0.f, 1.f);
  m.begin(Mesh::POLYGON);
  EXPECT_ANY_THROW(m.vertex(0.f, 0.f, 0.f));
  m.end();
}

TEST(Mesh, accessors) {
  Mesh m;
  m.position(0.f, 1.f, 2.f);
  m.position(3.f, 4.f, 5.f);
  EXPECT_EQ(0.f, *m.positionPtrAt(0));
  EXPECT_EQ(1.f, *(m.positionPtrAt(0) + 1));
  EXPECT_EQ(2.f, *(m.positionPtrAt(0) + 2));
  EXPECT_EQ(3.f, *m.positionPtrAt(1));
  EXPECT_EQ(4.f, *(m.positionPtrAt(1) + 1));
  EXPECT_EQ(5.f, *(m.positionPtrAt(1) + 2));

  m.normal(0.f, 0.f, 1.f);
  m.normal(0.f, 1.f, 0.f);
  EXPECT_EQ(0.f, *m.normalPtrAt(0));
  EXPECT_EQ(0.f, *(m.normalPtrAt(0) + 1));
  EXPECT_EQ(1.f, *(m.normalPtrAt(0) + 2));
  EXPECT_EQ(0.f, *m.normalPtrAt(1));
  EXPECT_EQ(1.f, *(m.normalPtrAt(1) + 1));
  EXPECT_EQ(0.f, *(m.normalPtrAt(1) + 2));

  m.texCoord(0.f, 1.f);
  m.texCoord(2.f, 3.f);
  EXPECT_EQ(0.f, *m.texCoordPtrAt(0));
  EXPECT_EQ(1.f, *(m.texCoordPtrAt(0) + 1));
  EXPECT_EQ(2.f, *m.texCoordPtrAt(1));
  EXPECT_EQ(3.f, *(m.texCoordPtrAt(1) + 1));

  m.begin(Mesh::POLYGON);
  m.vertex(0);
  m.vertex(1);
  m.vertex(0);
  m.end();
  m.begin(Mesh::POLYGON);
  m.vertex(0);
  m.vertex(1);
  m.vertex(0);
  m.vertex(1);
  m.end();

  EXPECT_EQ(0u, *m.vertexPtrAt(0));
  EXPECT_EQ(1u, *(m.vertexPtrAt(0) + 1));
  EXPECT_EQ(1u, *m.vertexPtrAt(1));

  EXPECT_EQ(1u, *(m.vertexPtrAt(1) + 1));
  EXPECT_EQ(0u, *m.vertexPtrAt(2));
  EXPECT_EQ(1u, *(m.vertexPtrAt(2) + 1));

  EXPECT_EQ(3, m.polygonVerticesCountAt(0));
  EXPECT_EQ(4, m.polygonVerticesCountAt(1));
}

TEST(Mesh, vertex) {
  Mesh m;
  size_t n = m.polygonsNb();

  size_t p = m.position(0.f, 0.f, 0.f);
  ASSERT_EQ(0u, p);
  ASSERT_EQ(1u, m.positionsNb());
  m.normal(0.f, 0.f, 1.f);
  m.texCoord(0.f, 0.f);

  m.begin(Mesh::POLYGON);
  m.end();
  // empty polygon is a no-op
  ASSERT_EQ(n, m.polygonsNb());

  m.begin(Mesh::POLYGON);
  m.vertex(p);
  m.vertex(p);
  m.vertex(p);
  m.end();
  ASSERT_EQ(n + 1u, m.polygonsNb());
  ASSERT_EQ(3u, m.polygonVerticesCountAt(n));

  m.begin(Mesh::POLYGON);
  m.vertex(p);
  m.vertex(p);
  m.vertex(p);
  m.vertex(p);
  m.end();
  ASSERT_EQ(n + 2u, m.polygonsNb());
  ASSERT_EQ(4u, m.polygonVerticesCountAt(n + 1u));

  m.begin(Mesh::TRIANGLES);
  m.end();
  // empty triangles set is a no-op
  ASSERT_EQ(n + 2u, m.polygonsNb());

  m.begin(Mesh::TRIANGLES);
  m.vertex(p);
  m.vertex(p);
  m.vertex(p);
  // the triangle is not added before the end() call
  ASSERT_EQ(n + 2u, m.polygonsNb());
  m.vertex(p);
  m.vertex(p);
  // unfinished triangle
  EXPECT_ANY_THROW(m.end());
  // the first triangle was not added before the failed end() call
  ASSERT_EQ(n + 2u, m.polygonsNb());
  m.vertex(p);
  m.end();
  ASSERT_EQ(n + 4u, m.polygonsNb());
  ASSERT_EQ(3u, m.polygonVerticesCountAt(n + 2u));
  ASSERT_EQ(3u, m.polygonVerticesCountAt(n + 3u));

  m.begin(Mesh::QUADS);
  m.end();
  // empty quads set is a no-op
  ASSERT_EQ(n + 4, m.polygonsNb());

  m.begin(Mesh::QUADS);
  m.vertex(p);
  m.vertex(p);
  m.vertex(p);
  m.vertex(p);
  ASSERT_EQ(n + 4u, m.polygonsNb());
  m.vertex(p);
  m.vertex(p);
  m.vertex(p);
  // unfinished quad
  EXPECT_ANY_THROW(m.end());
  // the first triangle was not added before the failed end() call
  ASSERT_EQ(n + 4u, m.polygonsNb());
  m.vertex(p);
  m.end();
  ASSERT_EQ(n + 6u, m.polygonsNb());
  ASSERT_EQ(4u, m.polygonVerticesCountAt(n + 4u));
  ASSERT_EQ(4u, m.polygonVerticesCountAt(n + 5u));
}

class ColladaSceneBuilderTest : public ::testing::Test {
 public:
  static boost::filesystem::path tmpdir;
  static void SetUpTestCase() {
    // tmpdir = "/tmp"; return;
    tmpdir = mktmpdir("test_visualization3d");
  }
  static void writeScene(const ColladaSceneBuilder &scene,
                         const std::string &filename) {
    boost::filesystem::path path = tmpdir / filename;
    std::cout << "writing collada scene to: " << path << "\n";
    boost::filesystem::ofstream os(path.c_str());
    os << scene;
  }
};
boost::filesystem::path ColladaSceneBuilderTest::tmpdir;
#define WRITE_SCENE(scene) writeScene(scene, #scene ".dae")
TEST_F(ColladaSceneBuilderTest, rule_of_three) {
  SceneBuilder::Config config0, config1;
  config1.color.r = 1.f - config0.color.r;

  ColladaSceneBuilder sc0;
  EXPECT_EQ(config0.color.r, sc0.getConfig().color.r);

  ColladaSceneBuilder sc1(config1);
  sc1.add(createBoxMesh(1.f, 1.f, 1.f), Math::Transform());
  EXPECT_EQ(config1.color.r, sc1.getConfig().color.r);

  ColladaSceneBuilder sc2(sc1);
  sc2.add(createBoxMesh(.5f, .5f, .5f), Math::Transform(2.f, 0.f, 0.f));
  EXPECT_EQ(config1.color.r, sc2.getConfig().color.r);

  ColladaSceneBuilder sc3;
  sc3 = sc1;
  sc3.add(createBoxMesh(.5f, .5f, .5f), Math::Transform(2.f, 0.f, 0.f));
  EXPECT_EQ(config1.color.r, sc0.getConfig().color.r);

  WRITE_SCENE(sc0);
  WRITE_SCENE(sc1);
  WRITE_SCENE(sc2);
  WRITE_SCENE(sc3);
}

TEST_F(ColladaSceneBuilderTest, shapes) {
  ColladaSceneBuilder sc;
  float d = 3.f;
  float x = -d;
  float y = 0.f;
  float z = 0.f;

  // add a box to m0
  Mesh m0 = createBoxMesh(1.1f, 1.2f, 1.3f);
  sc.add(m0, Eigen::Affine3f(Eigen::Translation3f(x += d, y, z)));

  // add another box to m0
  addBoxMesh(0.55f, 0.6f, 1.9f, m0);
  sc.add(m0, Math::Transform(x += d, y, z));

  x = -d;
  y += 3.f;

  sc.add(createRoundedBoxMesh(0.7f, 0.8f, 0.9f, 0.4f, 0),
         Math::Transform(x += d, y, z));
  sc.add(createRoundedBoxMesh(0.7f, 0.8f, 0.9f, 0.4f, 1),
         Math::Transform(x += d, y, z));
  sc.add(createRoundedBoxMesh(0.7f, 0.8f, 0.9f, 0.4f, 2),
         Math::Transform(x += d, y, z));
  sc.add(createRoundedBoxMesh(0.3f, 0.4f, 0.5f, 0.8f, 2),
         Math::Transform(x += d, y, z));
  sc.add(createRoundedBoxMesh(0.f, 0.8f, 0.9f, 0.4f, 2),
         Math::Transform(x += d, y, z));
  sc.add(createRoundedBoxMesh(0.7f, 0.f, 0.9f, 0.4f, 2),
         Math::Transform(x += d, y, z));
  sc.add(createRoundedBoxMesh(0.7f, 0.8f, 0.f, 0.4f, 2),
         Math::Transform(x += d, y, z));
  sc.add(createRoundedBoxMesh(0.f, 0.f, 0.9f, 0.4f, 2),
         Math::Transform(x += d, y, z));
  sc.add(createRoundedBoxMesh(0.f, 0.8f, 0.f, 0.4f, 2),
         Math::Transform(x += d, y, z));
  sc.add(createRoundedBoxMesh(0.7f, 0.f, 0.f, 0.4f, 2),
         Math::Transform(x += d, y, z));
  sc.add(createRoundedBoxMesh(0.f, 0.f, 0.f, 0.4f, 2),
         Math::Transform(x += d, y, z));
  sc.add(createRoundedBoxMesh(0.f, 0.f, 0.f, 0.4f, 11),
         Math::Transform(x += d, y, z));
  // fall back to a box
  sc.add(createRoundedBoxMesh(1.1f, 1.2f, 1.3f, 0.f, 2),
         Eigen::Affine3f(Eigen::Translation3f(x += d, y, z)));

  x = -d;
  y += 3.f;

  // use a ptr to "forget" the concrete type of the shape and test the
  // visitor
  boost::scoped_ptr<Math::Shape3D> shape;
  shape.reset(new Math::Sphere(1.f));
  sc.add(*shape, Math::Transform(x += d, y, z));
  shape.reset(new Math::RoundedRectangle(0.7f, 0.8f, 0.4f));
  sc.add(*shape, Math::Transform(x += d, y, z));
  shape.reset(new Math::Pill(0.9f, 0.4f));
  sc.add(*shape, Math::Transform(x += d, y, z));
  shape.reset(new Math::Plane);
  EXPECT_ANY_THROW(sc.add(*shape, Math::Transform()));
  WRITE_SCENE(sc);
}
