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
