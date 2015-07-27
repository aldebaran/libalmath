/**
* @author Nicolas Garcia
* @author Justine Lança
* Copyright (c) Aldebaran Robotics 2014 All Rights Reserved
*/

#include <gtest/gtest.h>
#include <almath/geometrics/shapes3d_utils.h>
#include <almath/tools/altrigonometry.h>

using namespace AL;

TEST(ShapeUtilsTest, pillX) {
  const float epsilon(1e-5f);
  const Math::Position3D jointEndTfToCenterA(-0.03f, 0.0f, 0.0f);
  const Math::Position3D jointEndTfToCenterB(0.03f, 0.0f, 0.0f);

  float halfExtent(0.0f);
  Math::Transform pillTf;
  Math::computePillParameters(jointEndTfToCenterA, jointEndTfToCenterB,
                              halfExtent, pillTf);

  const Math::Position3D pillTfZ(pillTf.r1_c3, pillTf.r2_c3, pillTf.r3_c3);
  const Math::Position3D x(1.0f, 0.0f, 0.0f);
  EXPECT_NEAR(0.0f, pillTfZ.crossProduct(x).norm(), epsilon);
  EXPECT_NEAR(1.0f, pillTfZ.dotProduct(x), epsilon);
}

TEST(ShapeUtilsTest, pillY) {
  const float epsilon(1e-5f);
  const Math::Position3D jointEndTfToCenterC(0.0f, -0.02f, 0.0f);
  const Math::Position3D jointEndTfToCenterD(0.0f, 0.02f, 0.0f);

  float halfExtent2(0.0f);
  Math::Transform pillTf2;
  Math::computePillParameters(jointEndTfToCenterC, jointEndTfToCenterD,
                              halfExtent2, pillTf2);

  const Math::Position3D pillTfZ2(pillTf2.r1_c3, pillTf2.r2_c3, pillTf2.r3_c3);
  const Math::Position3D y(0.0f, 1.0f, 0.0f);
  EXPECT_NEAR(0.0f, pillTfZ2.crossProduct(y).norm(), epsilon);
  EXPECT_NEAR(1.0f, pillTfZ2.dotProduct(y), epsilon);
}

TEST(ShapeUtilsTest, pillZ) {
  const float epsilon(1e-5f);
  const Math::Position3D jointEndTfToCenterE(0.0f, 0.0f, -0.01f);
  const Math::Position3D jointEndTfToCenterF(0.0f, 0.0f, 0.01f);

  float halfExtent3(0.0f);
  Math::Transform pillTf3;
  Math::computePillParameters(jointEndTfToCenterE, jointEndTfToCenterF,
                              halfExtent3, pillTf3);

  const Math::Position3D pillTfZ3(pillTf3.r1_c3, pillTf3.r2_c3, pillTf3.r3_c3);
  const Math::Position3D z(0.0f, 0.0f, 1.0f);
  EXPECT_NEAR(0.0f, pillTfZ3.crossProduct(z).norm(), epsilon);
  EXPECT_NEAR(1.0f, pillTfZ3.dotProduct(z), epsilon);
}
