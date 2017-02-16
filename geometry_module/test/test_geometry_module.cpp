#include <gtest/gtest.h>
#include <qi/anymodule.hpp>
#include <qi/geometry/geometry.hpp>
#include <qi/application.hpp>
#include <boost/math/constants/constants.hpp>

using namespace qi::geometry;

class GeometryModuleTest : public ::testing::Test{
public:
  qi::AnyModule gm = qi::import("geometry_module");
};

TEST_F(GeometryModuleTest, makeVector3) {
  auto v3 = gm.call<Vector3>("makeVector3", 1., 2., 3.);
  EXPECT_EQ(1., v3.x);
  EXPECT_EQ(2., v3.y);
  EXPECT_EQ(3., v3.z);
}

TEST_F(GeometryModuleTest, makeQuaternion) {
  auto q = gm.call<Quaternion>("makeQuaternion", 1., 2., 3., 4.);
  EXPECT_EQ(1., q.x);
  EXPECT_EQ(2., q.y);
  EXPECT_EQ(3., q.z);
  EXPECT_EQ(4., q.w);

  auto n = gm.call<double>("norm", q);
  EXPECT_DOUBLE_EQ(sqrt(1. + 4. + 9. + 16.), n);

  auto qn = gm.call<Quaternion>("normalized", q);
  EXPECT_DOUBLE_EQ(q.x/n, qn.x);
  EXPECT_DOUBLE_EQ(q.y/n, qn.y);
  EXPECT_DOUBLE_EQ(q.z/n, qn.z);
  EXPECT_DOUBLE_EQ(q.w/n, qn.w);
}

TEST_F(GeometryModuleTest, makeQuaternionFromAngleAxis) {
  auto axis = gm.call<Vector3>("makeVector3", 1., 1., 1.);
  auto norm = sqrt(3.);

  auto angle = 2 * boost::math::constants::pi<double>()/3;
  auto sin_ha = sin(angle/2);
  auto cos_ha = cos(angle/2);
  auto q = gm.call<Quaternion>("makeQuaternionFromAngleAxis", angle, axis);
  EXPECT_DOUBLE_EQ(sin_ha * axis.x/norm, q.x);
  EXPECT_DOUBLE_EQ(sin_ha * axis.y/norm, q.y);
  EXPECT_DOUBLE_EQ(sin_ha * axis.z/norm, q.z);
  EXPECT_DOUBLE_EQ(cos_ha, q.w);

  EXPECT_DOUBLE_EQ(1., gm.call<double>("norm", q));
}

// TODO: duplicate this test using quaternion which is not normalized
TEST_F(GeometryModuleTest, transform) {

  auto rotation = gm.call<Quaternion>("makeQuaternion", 0.5, 0.5, 0.5, 0.5);
  auto translation = gm.call<Vector3>("makeVector3", 1., 2., 3.);
  auto transform = gm.call<Transform>("makeTransform", rotation, translation);

  EXPECT_EQ(rotation.x, transform.rotation.x);
  EXPECT_EQ(rotation.y, transform.rotation.y);
  EXPECT_EQ(rotation.z, transform.rotation.z);
  EXPECT_EQ(rotation.w, transform.rotation.w);
  EXPECT_EQ(translation.x, transform.translation.x);
  EXPECT_EQ(translation.y, transform.translation.y);
  EXPECT_EQ(translation.z, transform.translation.z);

  auto inv_transform = gm.call<Transform>("inverse", transform);
  auto id_transform = gm.call<Transform>("multiply", transform, inv_transform);

  // id_transform should be the identity
  EXPECT_DOUBLE_EQ(0., id_transform.rotation.x);
  EXPECT_DOUBLE_EQ(0., id_transform.rotation.y);
  EXPECT_DOUBLE_EQ(0., id_transform.rotation.z);
  EXPECT_TRUE((id_transform.rotation.w == 1.) ||
              (id_transform.rotation.w == -1.));
  EXPECT_DOUBLE_EQ(0., id_transform.translation.x);
  EXPECT_DOUBLE_EQ(0., id_transform.translation.y);
  EXPECT_DOUBLE_EQ(0., id_transform.translation.z);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  qi::Application app(argc, argv);
  return RUN_ALL_TESTS();
}
