/*
 * Copyright (c) 2016 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <gtest/gtest.h>
#include <sstream>
#include <Eigen/Dense>
#include <almath/scenegraph/bodymass.h>
#include <array>

using namespace AL::Math;

using BodyMassd = BodyMass<double>;
BodyMassd makeCuboidInertia(double mass, const Eigen::Vector3d &com,
                            const Eigen::Array3d &half_length) {
  const Eigen::Array3d sq = half_length.square();
  return BodyMassd{mass, com, mass/3 * Eigen::DiagonalMatrix<double, 3>(
          sq[1]+sq[2], sq[0]+sq[2], sq[0]+sq[1])};
}

const double mass = 4.;
const double com_x = 8., com_y = 42., com_z = 13.;
const double hl_x = 2., hl_y = 1., hl_z = 2.;
// a cuboid
const auto cuboid4 = makeCuboidInertia(mass,
                                       Eigen::Vector3d{com_x, com_y, com_z},
                                       Eigen::Array3d{hl_x, hl_y, hl_z});
// the same cuboid split in 4 pieces
//
//  +------+------+-------------+    ^
//  |      |      |             |    | hl_y
//  |      |      +-------------+    x
//  |      |      |             |
//  +------+------+-------------+
//                x-------------> hl_x

const auto cuboid4_split = std::array<BodyMassd, 4>{
  makeCuboidInertia(
      mass/4,
      Eigen::Vector3d{com_x - 3 * hl_x / 4, com_y, com_z},
      Eigen::Array3d{hl_x / 4, hl_y, hl_z}),
  makeCuboidInertia(
      mass/4,
      Eigen::Vector3d{com_x - hl_x / 4, com_y, com_z},
      Eigen::Array3d{hl_x / 4, hl_y, hl_z}),
  makeCuboidInertia(
      mass/4,
      Eigen::Vector3d{com_x + hl_x / 2, com_y - hl_y / 2, com_z},
      Eigen::Array3d{hl_x / 2, hl_y / 2, hl_z}),
  makeCuboidInertia(
      mass/4,
      Eigen::Vector3d{com_x + hl_x / 2, com_y + hl_y / 2, com_z},
      Eigen::Array3d{hl_x / 2, hl_y / 2, hl_z})};

TEST(BodyMass, get_rotational_inertia_at) {
  EXPECT_EQ(BodyMassd::Matrix3::Zero(),
            BodyMassd::Zero().get_rotational_inertia_at(
              BodyMassd::Vector3::Zero()));
  EXPECT_EQ(cuboid4.rotational_inertia,
            cuboid4.get_rotational_inertia_at(cuboid4.center_of_mass));
}

TEST(BodyMass, squashBodyMasses) {
  EXPECT_EQ(BodyMassd::Zero(),
            squashBodyMasses<double>(
              std::array<BodyMassd, 1>{BodyMassd::Zero()}));
  EXPECT_EQ(cuboid4,
            squashBodyMasses<double>(std::array<BodyMassd, 1>{cuboid4}));
  EXPECT_EQ(BodyMassd::Zero(),
            squashBodyMasses<double>(
              std::array<BodyMassd, 2>{BodyMassd::Zero(), BodyMassd::Zero()}));
  EXPECT_EQ(cuboid4,
            squashBodyMasses<double>(
              std::array<BodyMassd, 2>{cuboid4, BodyMassd::Zero()}));
  EXPECT_EQ(cuboid4,
            squashBodyMasses<double>(
              std::array<BodyMassd, 2>{BodyMassd::Zero(), cuboid4}));

  const auto cuboid4_glued = squashBodyMasses<double>(cuboid4_split);
  EXPECT_EQ(cuboid4.mass, cuboid4_glued.mass);
  EXPECT_EQ(cuboid4.center_of_mass, cuboid4_glued.center_of_mass);
  EXPECT_TRUE(cuboid4.rotational_inertia.isApprox(
                cuboid4_glued.rotational_inertia));
}

TEST(BodyMass, squashBodyMasses_using_plus_sign) {
  const auto cuboid4_glued = cuboid4_split[0] + cuboid4_split[1] + \
                             cuboid4_split[2] + cuboid4_split[3];
  EXPECT_EQ(cuboid4.mass, cuboid4_glued.mass);
  EXPECT_EQ(cuboid4.center_of_mass, cuboid4_glued.center_of_mass);
  EXPECT_TRUE(cuboid4.rotational_inertia.isApprox(
                cuboid4_glued.rotational_inertia));
}
