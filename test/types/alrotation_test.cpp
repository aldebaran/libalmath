/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */
#include <almath/types/alrotation.h>

#include <gtest/gtest.h>
#include <almath/tools/altrigonometry.h>

TEST(ALRotationTest, isNear)
{
  AL::Math::Rotation pRotIn  = AL::Math::Rotation();
  AL::Math::Rotation pRotOut = AL::Math::Rotation();
  EXPECT_TRUE(pRotIn.isNear(pRotOut));

  pRotIn  = AL::Math::Rotation::fromRotX(0.1f);
  pRotOut = AL::Math::Rotation();
  EXPECT_FALSE(pRotIn.isNear(pRotOut));
}

TEST(ALRotationTest, transpose)
{
  AL::Math::Rotation pRotIn  = AL::Math::Rotation::fromRotX(0.3f);
  AL::Math::Rotation pRotOut = AL::Math::Rotation::fromRotX(-0.3f);
  EXPECT_TRUE(pRotIn.transpose().isNear(pRotOut, 0.0001f));

  pRotIn  = AL::Math::Rotation::fromRotY(0.3f);
  pRotOut = AL::Math::Rotation::fromRotY(-0.3f);
  EXPECT_TRUE(pRotIn.transpose().isNear(pRotOut, 0.0001f));

  pRotIn  = AL::Math::Rotation::fromRotZ(0.3f);
  pRotOut = AL::Math::Rotation::fromRotZ(-0.3f);
  EXPECT_TRUE(pRotIn.transpose().isNear(pRotOut, 0.0001f));
}

TEST(ALRotationTest, determinant0)
{
  AL::Math::Rotation pRot = AL::Math::Rotation::fromRotX(0.3f);
  EXPECT_NEAR(pRot.determinant(), 1.0f, 0.0001f);

  pRot = AL::Math::Rotation::fromRotY(0.3f);
  EXPECT_NEAR(pRot.determinant(), 1.0f, 0.0001f);

  pRot = AL::Math::Rotation::fromRotZ(0.3f);
  EXPECT_NEAR(pRot.determinant(), 1.0f, 0.0001f);
}


TEST(ALRotationTest, fromQuaternion)
{

}

TEST(ALRotationTest, fromAngleDirection)
{

}

TEST(ALRotationTest, fromRot)
{
  AL::Math::Rotation pRotIn  = AL::Math::Rotation::fromRotX(0.33f);
  AL::Math::Rotation pRotOut = AL::Math::Rotation();

  pRotOut.r1_c1 = 1.0f;
  pRotOut.r1_c2 = 0.0f;
  pRotOut.r1_c3 = 0.0f;
  pRotOut.r2_c1 = 0.0f;
  pRotOut.r2_c2 = 0.94604234352839f;
  pRotOut.r2_c3 = -0.32404302839487f;
  pRotOut.r3_c1 = 0.0f;
  pRotOut.r3_c2 = 0.32404302839487f;
  pRotOut.r3_c3 = 0.94604234352839f;
  EXPECT_TRUE(pRotIn.isNear(pRotOut, 0.0001f));

  pRotIn  = AL::Math::Rotation::fromRotY(-0.65f);
  pRotOut = AL::Math::Rotation();
  pRotOut.r1_c1 = 0.79608379854906f;
  pRotOut.r1_c2 = 0.0f;
  pRotOut.r1_c3 = -0.60518640573604f;
  pRotOut.r2_c1 = 0.0f;
  pRotOut.r2_c2 = 1.00000000000000f;
  pRotOut.r2_c3 = 0.0f;
  pRotOut.r3_c1 = 0.60518640573604f;
  pRotOut.r3_c2 = 0.0f;
  pRotOut.r3_c3 = 0.79608379854906f;
  EXPECT_TRUE(pRotIn.isNear(pRotOut, 0.0001f));

  pRotIn  = AL::Math::Rotation::fromRotZ(0.5f);
  pRotOut = AL::Math::Rotation();
  pRotOut.r1_c1 = 0.87758256189037f;
  pRotOut.r1_c2 = -0.47942553860420f;
  pRotOut.r2_c1 = 0.47942553860420f;
  pRotOut.r2_c2 = 0.87758256189037f;
  EXPECT_TRUE(pRotIn.isNear(pRotOut, 0.0001f));
}

TEST(ALRotationTest, from3DRotation)
{

}


TEST(ALRotationTest, determinant1)
{
  AL::Math::Rotation pRot = AL::Math::Rotation();
  EXPECT_NEAR(AL::Math::determinant(pRot), 1.0f, 0.0001f);

  pRot = AL::Math::rotationFromRotX(20.0f*AL::Math::TO_RAD);
  EXPECT_NEAR(AL::Math::determinant(pRot), 1.0f, 0.0001f);

  pRot = AL::Math::rotationFromRotY(30.0f*AL::Math::TO_RAD);
  EXPECT_NEAR(AL::Math::determinant(pRot), 1.0f, 0.0001f);

  pRot = AL::Math::rotationFromRotZ(40.0f*AL::Math::TO_RAD);
  EXPECT_NEAR(AL::Math::determinant(pRot), 1.0f, 0.0001f);
}

