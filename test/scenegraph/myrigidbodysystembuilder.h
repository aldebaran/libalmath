#ifndef LIB_ALMATH_TEST_SCENEGRAPH_MYRIGIDBODYSYSTEMBUILDER_H
#define LIB_ALMATH_TEST_SCENEGRAPH_MYRIGIDBODYSYSTEMBUILDER_H

#include <gtest/gtest.h>
#include <almath/scenegraph/rigidbodysystembuilder.h>

namespace AL {
namespace Math {
namespace RigidBodySystemBuilder {

class MyBuilder : public AL::Math::RigidBodySystemBuilder::Interface<double> {
 public:
  using Interface<double>::Link;
  using Interface<double>::StaticFrame;
  using Interface<double>::AffineCompact3;
  using Interface<double>::BodyMass;
  using Interface<double>::Vector3;
  using Interface<double>::Matrix3;

  MyBuilder() {}

  const Config _config;
  std::vector<Link> links;
  std::vector<StaticFrame> staticframes;

  const Config &config() const { return _config; }
  void addLink(Link data) { links.push_back(data); }
  void addStaticFrame(StaticFrame data) { staticframes.push_back(data); }
};

class RigidBodySystemBuilderTest : public ::testing::Test {
 public:
  MyBuilder b;                // back of the builders pipe
  NamesChecker<double> f{b};  // front
};
}
}
}
#endif
