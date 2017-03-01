/**
 * @author Lucas Souchet - lsouchet@aldebaran.com
 * Aldebaran Robotics (c) 2016 All Rights Reserved - This file is confidential.
 *
 */

#include <gtest/gtest.h>
#include <qi/geometry/geometry.hpp>
#include <almath/scenegraph/almathqigeometry.hpp>
#include <almath/scenegraph/qigeometry.h>

TEST(ALMathQiGeometry, transformFromQiTransform)
{
  qi::geometry::Transform t = qi::geometry::makeTransform(
        qi::geometry::makeQuaternion(0.149, 0.129, 0.170, 0.965),
        qi::geometry::makeVector3(5.0, 6.0, 7.0));
  AL::Math::Transform tf = AL::Math::transformFromQiTransform(t);
  AL::Math::Quaternion q = AL::Math::quaternionFromTransform(tf);
  EXPECT_NEAR(q.x, 0.149f, 1e-3f);
  EXPECT_NEAR(q.y, 0.129f, 1e-3f);
  EXPECT_NEAR(q.z, 0.170f, 1e-3f);
  EXPECT_NEAR(q.w, 0.965f, 1e-3f);
  AL::Math::Position3D p = AL::Math::position3DFromTransform(tf);
  EXPECT_NEAR(p.x, 5.0f, 1e-5f);
  EXPECT_NEAR(p.y, 6.0f, 1e-5f);
  EXPECT_NEAR(p.z, 7.0f, 1e-5f);
}

TEST(ALMathQiGeometry, pose2DFromQiTransformSimple)
{
  qi::geometry::Transform t = qi::geometry::makeTransform(
        qi::geometry::makeQuaternion(0.0, 0.0, 0.247, 0.968),
        qi::geometry::makeVector3(5.0, 6.0, 7.0));
  AL::Math::Pose2D p = AL::Math::pose2DFromQiTransform(t);
  EXPECT_NEAR(p.x, 5.0f, 1e-5f);
  EXPECT_NEAR(p.y, 6.0f, 1e-5f);
  EXPECT_NEAR(p.theta, 0.5f, 2e-3f);
}

TEST(ALMathQiGeometry, pose2DFromQiTransformNotAlongZ)
{
  qi::geometry::Transform t = qi::geometry::makeTransform(
        qi::geometry::makeQuaternion(0.247, 0.0, 0.0, 0.968),
        qi::geometry::makeVector3(5.0, 6.0, 7.0));
  AL::Math::Pose2D p = AL::Math::pose2DFromQiTransform(t);
  EXPECT_NEAR(p.x, 5.0f, 1e-5f);
  EXPECT_NEAR(p.y, 6.0f, 1e-5f);
  EXPECT_NEAR(p.theta, 0.0f, 2e-3f);
}

TEST(ALMathQiGeometry, pose2DFromQiTransformComposed)
{
  qi::geometry::Transform t = qi::geometry::makeTransform(
        qi::geometry::makeQuaternion(0.0, 0.174, 0.174, 0.984),
        qi::geometry::makeVector3(5.0, 6.0, 7.0));
  AL::Math::Pose2D p = AL::Math::pose2DFromQiTransform(t);
  EXPECT_NEAR(p.x, 5.0f, 1e-5f);
  EXPECT_NEAR(p.y, 6.0f, 1e-5f);
  EXPECT_NEAR(p.theta, 0.371f, 2e-3f);
}

TEST(ALMathQiGeometry, qiTransformFromPose2D)
{
  AL::Math::Pose2D p(1.0f, 2.0f, 1.57f);
  qi::geometry::Transform t = AL::Math::qiTransformFromPose2D(p);
  EXPECT_NEAR(t.translation.x, 1.0, 1e-5f);
  EXPECT_NEAR(t.translation.y, 2.0, 1e-5f);
  EXPECT_NEAR(t.translation.z, 0.0, 1e-5f);
  EXPECT_NEAR(t.rotation.x, 0.0, 1e-5f);
  EXPECT_NEAR(t.rotation.y, 0.0, 1e-5f);
  EXPECT_NEAR(t.rotation.z, 0.706, 2e-3f);
  EXPECT_NEAR(t.rotation.w, 0.707, 2e-3f);
}
