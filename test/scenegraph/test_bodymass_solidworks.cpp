/*
 * Copyright (c) 2016 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <gtest/gtest.h>
#include <sstream>
#include <fstream>
#include <Eigen/Dense>
#include <boost/math/constants/constants.hpp>

// Given inertia expressed at frame (p, a) and rotation matrix from
// basis b to basis a, return the inertia expressed at frame (p, b)
Eigen::Matrix3d rotateInertia(const Eigen::Matrix3d &inertia_p_a,
                              const Eigen::Matrix3d &R_a_b) {
  return R_a_b.transpose() * inertia_p_a * R_a_b;
}

// Return the inertia of a point mass located at p
// (expressed in the same frame as the p coordinates).
Eigen::Matrix3d pointMassInertia(double mass, const Eigen::Vector3d &p) {
  return mass * (p.array().square().sum() * Eigen::Matrix3d::Identity() -
                 (p * p.transpose()));
}

TEST(BodyMass, solidworks) {
  // std::ofstream os("/tmp/out");
  auto &os = std::cout;

  // a format which plays nicely with sphinx.
  Eigen::IOFormat rst(4, 0, ", ", "\n", "  [", "]", "\n", "\n");
  // a format which plays nicely with urdf.
  Eigen::IOFormat urdf(Eigen::StreamPrecision, Eigen::DontAlignCols);
  Eigen::Array3d half_extents(0.2, 0.3, 0.4);
  double mass = 2.;
  double pi = boost::math::constants::pi<double>();
  Eigen::Array3d half_extents2 = half_extents.square();
  Eigen::Vector3d moments = mass / 3 * (half_extents2.sum() - half_extents2);

  os << "Let consider a box of mass " << mass << " kg and half-lengths (m)::\n"
     << half_extents.transpose().format(rst) << "\n"
     << "its principal moments of inertia are (kg m^2)::\n"
     << moments.transpose().format(rst) << " \n\n";

  os << "Let name \"a\" the basis aligned with the box axis, and \"c\""
        " the box center (which is also its center of mass).\n"
        "The box rotational inertia matrix at center of mass, aligned with "
        "basis a (ie. in the (c,a) frame) is::\n";
  Eigen::Matrix3d inertia_c_a = moments.asDiagonal();
  os << inertia_c_a.format(rst) << "\n\n";

  os << "let name \"b\" the basis defined by rotating of pi/6 the basis a "
        "around the z-axis.\n"
     << "The corresponding rotation matrix is::\n";
  Eigen::Matrix3d R_a_b;
  double yaw_angle = pi / 6;
  R_a_b = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());
  os << R_a_b.format(rst) << "\n\n";

  os << "The box rotational inertia matrix at center of mass, aligned with "
        "basis b (ie. in the (c,b) frame) is::\n";
  Eigen::Matrix3d inertia_c_b = rotateInertia(inertia_c_a, R_a_b);
  os << inertia_c_b.format(rst) << "\n"
     << "The principal axis of inertia in basis b are the columns::\n"
     << R_a_b.transpose().format(rst) << "\n\n";

  Eigen::Vector3d p_c_a = half_extents;
  os << "Let name \"p\" a corner of the box.\n"
     << "It's coordinates in the (c, a) frame are::\n"
     << p_c_a.transpose().format(rst) << "\n"
     << "The box rotational inertia matrix at p, aligned with basis a is::\n";
  Eigen::Matrix3d inertia_p_a = pointMassInertia(mass, p_c_a) + inertia_c_a;
  os << inertia_p_a.format(rst) << "\n\n";

  os << "The coordinates of p in the (c, b) frame are::\n";
  Eigen::Vector3d p_c_b = R_a_b.transpose() * p_c_a;
  os << p_c_b.transpose().format(rst) << "\n\n";

  os << "The box rotational inertia matrix in the (p, b) frame can be "
        "computed from the one at (p, a)::\n";
  Eigen::Matrix3d inertia_p_b = rotateInertia(inertia_p_a, R_a_b);
  os << inertia_p_b.format(rst) << "\n\n"
     << "...or from the one at (c, b)::\n";
  Eigen::Matrix3d inertia_p_b_bis = pointMassInertia(mass, p_c_b) + inertia_c_b;
  os << inertia_p_b_bis.format(rst) << "\n\n";

  std::stringstream sv_c_a;
  sv_c_a << "      <visual>\n"
         << "          <origin xyz=\"" << (-p_c_b.transpose()).format(urdf)
         << "\" rpy=\"0 0 " << -yaw_angle << "\"/>\n"
         << "          <geometry>\n"
         << "              <box size=\"" << (2 * half_extents).transpose()
         << "\"/>\n"
         << "          </geometry>\n"
         << "      </visual>\n";

  std::stringstream si_c_b;
  si_c_b << "      <inertial>\n"
         << "          <mass value=\"" << mass << "\"/>\n"
         << "          <origin xyz=\"" << (-p_c_b.transpose()).format(urdf)
         << "\"/>\n"
         << "          <inertia"
         << " ixx=\"" << inertia_c_b(0, 0) << "\""
         << " ixy=\"" << inertia_c_b(0, 1) << "\""
         << " ixz=\"" << inertia_c_b(0, 2) << "\""
         << " iyy=\"" << inertia_c_b(1, 1) << "\""
         << " iyz=\"" << inertia_c_b(1, 2) << "\""
         << " izz=\"" << inertia_c_b(2, 2) << "\" />\n"
         << "      </inertial>\n";

  std::stringstream si_c_a;
  si_c_a << "      <inertial>\n"
         << "          <mass value=\"" << mass << "\"/>\n"
         << "          <origin xyz=\"" << (-p_c_b.transpose()).format(urdf)
         << "\" rpy=\"0 0 " << -yaw_angle << "\"/>\n"
         << "          <inertia"
         << " ixx=\"" << inertia_c_a(0, 0) << "\""
         << " ixy=\"" << inertia_c_a(0, 1) << "\""
         << " ixz=\"" << inertia_c_a(0, 2) << "\""
         << " iyy=\"" << inertia_c_a(1, 1) << "\""
         << " iyz=\"" << inertia_c_a(1, 2) << "\""
         << " izz=\"" << inertia_c_a(2, 2) << "\" />\n"
         << "      </inertial>\n";

  os << "Let now assume one wants to model this box as an URDF link, using"
        " frame (p, b) as the link origin.\n"
        "Our box visual can be displayed using the URDF \"box\" element,"
        " placed in frame (c, a)::\n\n" << sv_c_a.str() << "\n";

  os << "Let now assume one wants to model this box as an URDF link, using"
        " frame (p, b) as the link origin.\n"
        "The inertial URDF element can be defined in frame (c, b), resulting"
        " in::\n\n" << si_c_b.str() << "\n\n";

  os << "...or in frame (c, a), resulting in::\n\n" << si_c_a.str() << "\n";

  os << "The full URDF file being::\n\n"
     << "  <robot name=\"MyBox\">\n"
     << "  <link name=\"Box\">\n" << sv_c_a.str() << si_c_b.str()
     << "  </link>\n"
     << "  </robot>\n";

  EXPECT_TRUE(inertia_p_b_bis.isApprox(inertia_p_b));
}
