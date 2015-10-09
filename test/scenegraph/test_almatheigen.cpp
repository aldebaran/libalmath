/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */

#include <almath/scenegraph/almatheigen.h>
#include <gtest/gtest.h>

using namespace AL;

TEST(ALMathEigen, pose_almath_to_eigen) {
  Math::Transform p;

  p.r1_c1 = 11;
  p.r1_c2 = 12;
  p.r1_c3 = 13;
  p.r1_c4 = 14;

  p.r2_c1 = 21;
  p.r2_c2 = 22;
  p.r2_c3 = 23;
  p.r2_c4 = 24;

  p.r3_c1 = 31;
  p.r3_c2 = 32;
  p.r3_c3 = 33;
  p.r3_c4 = 34;

  Math::Matrix34f m34 = Math::toEigenMatrix34(p);

  EXPECT_FLOAT_EQ(p.r1_c1, m34(0, 0));
  EXPECT_FLOAT_EQ(p.r1_c2, m34(0, 1));
  EXPECT_FLOAT_EQ(p.r1_c3, m34(0, 2));
  EXPECT_FLOAT_EQ(p.r1_c4, m34(0, 3));

  EXPECT_FLOAT_EQ(p.r2_c1, m34(1, 0));
  EXPECT_FLOAT_EQ(p.r2_c2, m34(1, 1));
  EXPECT_FLOAT_EQ(p.r2_c3, m34(1, 2));
  EXPECT_FLOAT_EQ(p.r2_c4, m34(1, 3));

  EXPECT_FLOAT_EQ(p.r3_c1, m34(2, 0));
  EXPECT_FLOAT_EQ(p.r3_c2, m34(2, 1));
  EXPECT_FLOAT_EQ(p.r3_c3, m34(2, 2));
  EXPECT_FLOAT_EQ(p.r3_c4, m34(2, 3));

  Eigen::Matrix3f m3 = Math::toEigenMatrix3(p);
  EXPECT_EQ((m34.block<3, 3>(0, 0)), m3);

  Eigen::AffineCompact3f ac3 = toEigenAffineCompact3(p);
  // for some reason, doing
  // EXPECT_EQ(m34, ac3.matrix());
  // fails with to build with
  // error C2718: 'AL::Math::Matrix34f': actual parameter with
  // __declspec(align('16')) won't be aligned
  EXPECT_TRUE(m34.isApprox(ac3.matrix()));
}

TEST(ALMathEigen, pose_eigen_to_almath) {
  Math::Matrix34f m34;
  m34 << 11, 12, 13, 14, 21, 22, 23, 24, 31, 32, 33, 34;

  Math::Transform p_ref = Math::toALMathTransform(m34);

  ASSERT_FLOAT_EQ(m34(0, 0), p_ref.r1_c1);
  ASSERT_FLOAT_EQ(m34(0, 1), p_ref.r1_c2);
  ASSERT_FLOAT_EQ(m34(0, 2), p_ref.r1_c3);
  ASSERT_FLOAT_EQ(m34(0, 3), p_ref.r1_c4);

  ASSERT_FLOAT_EQ(m34(1, 0), p_ref.r2_c1);
  ASSERT_FLOAT_EQ(m34(1, 1), p_ref.r2_c2);
  ASSERT_FLOAT_EQ(m34(1, 2), p_ref.r2_c3);
  ASSERT_FLOAT_EQ(m34(1, 3), p_ref.r2_c4);

  ASSERT_FLOAT_EQ(m34(2, 0), p_ref.r3_c1);
  ASSERT_FLOAT_EQ(m34(2, 1), p_ref.r3_c2);
  ASSERT_FLOAT_EQ(m34(2, 2), p_ref.r3_c3);
  ASSERT_FLOAT_EQ(m34(2, 3), p_ref.r3_c4);

  {
    Math::Transform p;
    Math::toALMathTransform(m34, p);
    EXPECT_TRUE(p_ref.isNear(p));
  }

  {
    Math::Matrix34frm m34rm = m34;
    Math::Transform p = Math::toALMathTransform(m34rm);
    EXPECT_TRUE(p_ref.isNear(p));

    Math::Transform pp;
    Math::toALMathTransform(m34rm, pp);
    EXPECT_TRUE(p_ref.isNear(pp));
  }

  {
    // just check it does compile
    Math::Transform p = Math::toALMathTransform(Math::Matrix34f::Identity());
    Math::toALMathTransform(Math::Matrix34f::Identity(), p);
  }
  {
    Eigen::AffineCompact3f ac3;
    ac3.matrix() = m34;
    Math::Transform p = Math::toALMathTransform(ac3);
    EXPECT_TRUE(p_ref.isNear(p));
    Math::Transform pp;
    Math::toALMathTransform(ac3, pp);
    EXPECT_TRUE(p_ref.isNear(pp));
  }
  {
    Eigen::Affine3f a3;
    a3.matrix().block<3, 4>(0, 0) = m34;
    a3.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1;
    Math::Transform p = Math::toALMathTransform(a3);
    EXPECT_TRUE(p_ref.isNear(p));
    Math::Transform pp;
    Math::toALMathTransform(a3, pp);
    EXPECT_TRUE(p_ref.isNear(pp));
  }
  {
    Eigen::Isometry3f i3;
    i3.matrix().block<3, 4>(0, 0) = m34;
    i3.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1;
    Math::Transform p = Math::toALMathTransform(i3);
    EXPECT_TRUE(p_ref.isNear(p));
    Math::Transform pp;
    Math::toALMathTransform(i3, pp);
    EXPECT_TRUE(p_ref.isNear(pp));
  }
}

TEST(ALMathEigen, position3d_almath_to_eigen) {
  Math::Position3D p(1, 2, 3);
  Eigen::Vector3f v = Math::toEigenVector3(p);

  EXPECT_FLOAT_EQ(p.x, v[0]);
  EXPECT_FLOAT_EQ(p.y, v[1]);
  EXPECT_FLOAT_EQ(p.z, v[2]);
}

TEST(ALMathEigen, velocity6d_eigen_to_almath) {
  Math::Vector6f v6;
  v6 << 1, 2, 3, 4, 5, 6;
  Math::Velocity6D v6d = Math::toALMathVelocity6D(v6);

  ASSERT_FLOAT_EQ(v6(0), v6d.xd);
  ASSERT_FLOAT_EQ(v6(1), v6d.yd);
  ASSERT_FLOAT_EQ(v6(2), v6d.zd);
  ASSERT_FLOAT_EQ(v6(3), v6d.wxd);
  ASSERT_FLOAT_EQ(v6(4), v6d.wyd);
  ASSERT_FLOAT_EQ(v6(5), v6d.wzd);

  Math::Velocity6D v6d_ref(v6d);
  Math::toALMathVelocity6D(v6, v6d);
  EXPECT_TRUE(v6d_ref.isNear(v6d));

  // just check it builds:
  Math::toALMathVelocity6D(v6.head<6>(), v6d);
  Math::toALMathVelocity6D(v6.block<6, 1>(0, 0), v6d);
  Math::toALMathVelocity6D(v6.head<6>(), v6d);
  Math::toALMathVelocity6D(v6.block<6, 1>(0, 0), v6d);
  // this does not build
  // Math::toALMathVelocity6D(v6.head(6), v6d);
  // Math::toALMathVelocity6D(v6.block(0, 6, 0, 1), v6d);
}
