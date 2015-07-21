/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */

#include <almath/scenegraph/urdfeigen.h>
#include <gtest/gtest.h>
#include <boost/math/constants/constants.hpp>
#include <boost/property_tree/ptree.hpp>

using namespace AL;
typedef AL::urdf::ptree ptree;
const double pi = boost::math::constants::pi<double>();

#define EXPECT_ARRAY3D_EQ(expected, actual)   \
  if (1) {                                    \
    const double eps = 1e-7;                  \
    EXPECT_NEAR(expected[0], actual[0], eps); \
    EXPECT_NEAR(expected[1], actual[1], eps); \
    EXPECT_NEAR(expected[2], actual[2], eps); \
  }

#define EXPECT_IS_APPROX(expected, actual)                           \
  if (1) {                                                           \
    EXPECT_TRUE((expected.matrix() - actual.matrix()).isZero(1e-10)) \
        << "expected:\n" << (expected).matrix() << "\nactual\n"      \
        << (actual).matrix() << "\ndiff\n"                           \
        << (expected.matrix() - actual.matrix()) << "\n";            \
  }

class RotationRPY : public ::testing::TestWithParam<
                        std::tr1::tuple<double, double, double> > {
 public:
  double r, p, y;
  virtual void SetUp() {
    r = std::tr1::get<0>(GetParam());
    p = std::tr1::get<1>(GetParam());
    y = std::tr1::get<2>(GetParam());
  }
};

// check that fromUrdfRpy(toUrdfRpy(x)) is idempotent
TEST_P(RotationRPY, rotation) {
  urdf::Array3d rpy = {{r, p, y}};
  typedef double Scalar;
  {
    typedef Eigen::Quaternion<Scalar, Eigen::DontAlign> T;
    T t = Math::eigenQuaternionFromUrdfRpy(rpy);
    urdf::Array3d back = Math::urdfRpyFromEigenMatrix3(t.matrix());
    EXPECT_IS_APPROX(t, Math::eigenQuaternionFromUrdfRpy(back))
  }
  {
    typedef Eigen::Matrix<Scalar, 3, 3, Eigen::DontAlign> T;
    T t(Math::eigenQuaternionFromUrdfRpy(rpy));
    urdf::Array3d back = Math::urdfRpyFromEigenMatrix3(t);
    EXPECT_IS_APPROX(t, T(Math::eigenQuaternionFromUrdfRpy(back)))
  }
  {
    typedef Eigen::Transform<Scalar, 3, Eigen::AffineCompact, Eigen::DontAlign>
        T;
    T t(Math::eigenQuaternionFromUrdfRpy(rpy));
    urdf::Array3d back = Math::urdfRpyFromEigenMatrix3(t.linear());
    EXPECT_IS_APPROX(t, T(Math::eigenQuaternionFromUrdfRpy(back)))
  }
}

// INSTANTIATE_TEST_CASE_P(Many, RotationRPY, ::testing::Combine(
//    testing::Range(-pi, pi, pi/6),
//    testing::Range(-pi, pi, pi/6),
//    testing::Range(-pi, pi, pi/6)));

INSTANTIATE_TEST_CASE_P(
    Some, RotationRPY,
    testing::Values(std::tr1::make_tuple(0., -pi / 2, 0.),
                    std::tr1::make_tuple(0., -pi / 2, pi / 3),
                    std::tr1::make_tuple(pi * 30 / 180, -pi / 2, pi * 75 / 180),
                    std::tr1::make_tuple(-pi / 3, pi / 2, pi / 3),
                    std::tr1::make_tuple(-3 * pi / 12, pi / 2, pi / 3),
                    std::tr1::make_tuple(pi / 3, pi / 2, pi / 2)));

TEST(UrdfEigen, pose) {
  typedef double Scalar;
  typedef Eigen::Transform<Scalar, 3, Eigen::AffineCompact, Eigen::DontAlign> T;
  ptree pt;
  pt.put("<xmlattr>.xyz", "1 2.2 3.3");
  std::ostringstream ss;
  ss << "0 " << boost::lexical_cast<std::string>(-pi / 2) << " 0";
  pt.put("<xmlattr>.rpy", ss.str());
  urdf::Pose p(pt);

  T t = Math::toEigenTransform(p);
  EXPECT_EQ(1, t(0, 3));
  EXPECT_EQ(2.2, t(1, 3));
  EXPECT_EQ(3.3, t(2, 3));

  EXPECT_IS_APPROX(Eigen::AngleAxisd(-pi / 2, Eigen::Vector3d::UnitY()),
                   t.linear());
}
