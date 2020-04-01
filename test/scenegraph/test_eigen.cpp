/*
 *  Copyright 2017 SoftBank Robotics. All rights reserved.
 *
 */

#include <almath/scenegraph/almatheigen.h>
#include <almath/scenegraph/eigen.h>
#include <almath/tools/altransformhelpers.h>
#include <gtest/gtest.h>

using namespace AL;

void checkExpImplemsAreNear(Eigen::Vector3f v, Eigen::Vector3f w) {
  Math::Transform tr_expected =
      velocityExponential(Math::Velocity6D(v(0), v(1), v(2), w(0), w(1), w(2)));
  Eigen::AffineCompact3f ac = Math::exp<Eigen::AffineCompact3f>(v, w);
  Math::Transform tr;
  Eigen::Map<Math::Matrix34frm>(&tr.r1_c1) = ac.matrix();

  EXPECT_TRUE(tr.isNear(tr_expected));
}

TEST(Eigen, velocityExponential) {
  using Eigen::Vector3f;
  checkExpImplemsAreNear(Vector3f(1, 2, 3), Vector3f(0, 0, 0));
  checkExpImplemsAreNear(Vector3f(0, 0, 0), Vector3f(1, 0, 0));
  checkExpImplemsAreNear(Vector3f(0, 0, 0), Vector3f(0, 1, 0));
  checkExpImplemsAreNear(Vector3f(0, 0, 0), Vector3f(0, 0, 1));
  checkExpImplemsAreNear(Vector3f(0, 0, 0), Vector3f(4, 5, 6));
  checkExpImplemsAreNear(Vector3f(1, 2, 3), Vector3f(4, 5, 6));
  checkExpImplemsAreNear(Vector3f(1, 1, 1), Vector3f(1, 1, 1));
  checkExpImplemsAreNear(Vector3f(1.f, .4f, -.2f), Vector3f(-.5f, .6f, -.7f));
}

template <typename T>
void echo_twist(const T &twist, std::ostream &os) {
  os << "linear:  " << twist.linear().transpose() << "\n"
     << "angular: " << twist.angular().transpose() << "\n"
     << "matrix: " << twist.matrix().transpose() << std::endl;
}

TEST(Eigen, twist) {
  Math::Twist<float, Math::TwistPolarity::AngularFirst, 0> twa;
  Math::Twist<float, Math::TwistPolarity::LinearFirst, 0> twl;
  twl.linear() = twa.linear() = Eigen::Vector3f::Ones();
  twl.angular() = twa.angular() = Eigen::Vector3f::Zero();
  echo_twist(twa, std::cout);
  echo_twist(twl, std::cout);
}

TEST(Eigen, skew)
{
  float x = 1;
  float y = 2;
  float z = 3;
  Eigen::Vector3f v;
  v << x, y, z;
  Eigen::Matrix3f skew_v = Math::skew(v);
  Eigen::Matrix3f expected;
  expected << 0, -z, y,
              z, 0, -x,
              -y, x, 0;
  for (auto row = 0; row < 3; ++row)
  {
    for (auto col = 0; col < 3; ++col)
    {
      ASSERT_FLOAT_EQ(expected(row, col), skew_v(row, col));
    }
  }
}

TEST(Eigen, adjoint)
{
  using Twist_AngularFirst = Math::Twist<float, Math::TwistPolarity::AngularFirst, 0>;
  using Twist_LinearFirst = Math::Twist<float, Math::TwistPolarity::LinearFirst, 0>;
  // Define the same twist_a twice, with AngularFirst and LinearFirst
  // polarities.
  Twist_AngularFirst twist_a_angular;
  Twist_LinearFirst twist_a_linear;
  twist_a_angular.angular() << 1, 2, 3;
  twist_a_linear.angular() << 1, 2, 3;
  twist_a_angular.linear() << 4, 5, 6;
  twist_a_linear.linear() << 4, 5, 6;

  Eigen::Matrix<float, 3, 4> pose_a_b;
  pose_a_b << 1, 0, 0, 1.5,
              0, 1, 0, -2.3,
              0, 0, 1, 0.6;

  // Compute the adjoint matrix with both polarities.
  Eigen::Matrix<float, 6, 6> adjoint_a_b_angular =
      Math::adjoint<Math::TwistPolarity::AngularFirst>(pose_a_b);
  Eigen::Matrix<float, 6, 6> adjoint_a_b_linear =
      Math::adjoint<Math::TwistPolarity::LinearFirst>(pose_a_b);

  // Compute the twist_b with both polarities and check that the linear and
  // angular parts are equal.
  Twist_AngularFirst twist_b_angular;
  twist_b_angular << adjoint_a_b_angular * twist_a_angular;
  Twist_LinearFirst twist_b_linear;
  twist_b_linear << adjoint_a_b_linear * twist_a_linear;

  const float epsilon = 1e-5f;
  EXPECT_NEAR(twist_b_angular.angular()(0), twist_b_linear.angular()(0), epsilon);
  EXPECT_NEAR(twist_b_angular.angular()(1), twist_b_linear.angular()(1), epsilon);
  EXPECT_NEAR(twist_b_angular.angular()(2), twist_b_linear.angular()(2), epsilon);
  EXPECT_NEAR(twist_b_angular.linear()(0), twist_b_linear.linear()(0), epsilon);
  EXPECT_NEAR(twist_b_angular.linear()(1), twist_b_linear.linear()(1), epsilon);
  EXPECT_NEAR(twist_b_angular.linear()(2), twist_b_linear.linear()(2), epsilon);
}
