/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */
#include <almath/types/alrotation.h>

#include <gtest/gtest.h>
#include <almath/tools/altrigonometry.h>

#include <almath/tools/almathio.h>

TEST(ALRotationTest, basicOperator)
{
  // operator *= with Rotation
  AL::Math::Rotation pRot1 = AL::Math::Rotation::from3DRotation(0.3f, -0.2f, 0.1f);
  AL::Math::Rotation pRot2 = AL::Math::Rotation();
  AL::Math::Rotation pRot3 = AL::Math::Rotation();

  pRot2 = pRot1;
  pRot2 *= pRot1;
  pRot1 *= pRot1;
  EXPECT_TRUE(pRot2.isNear(pRot1));

  pRot1 = AL::Math::Rotation::from3DRotation(0.3f, -0.2f, 0.1f);
  pRot2 = AL::Math::Rotation::from3DRotation(0.2f, -0.1f, 1.1f);

  pRot3 = pRot1*pRot2;
  pRot1 *= pRot2;

  EXPECT_TRUE(pRot3.isNear(pRot1));
}

TEST(ALRotationTest, isNear)
{
  AL::Math::Rotation pRotIn  = AL::Math::Rotation();
  AL::Math::Rotation pRotOut = AL::Math::Rotation();
  EXPECT_TRUE(pRotIn.isNear(pRotOut));

  pRotIn  = AL::Math::Rotation::fromRotX(0.1f);
  pRotOut = AL::Math::Rotation();
  EXPECT_FALSE(pRotIn.isNear(pRotOut));
}

TEST(RotationTest, isRotation)
{
  float pEps = 0.001f;
  AL::Math::Rotation rot;

  // std::cout << "test isRotation 0" << std::endl;
  rot = AL::Math::Rotation();
  EXPECT_TRUE(rot.isRotation(pEps));

  // std::cout << "test isRotation 1" << std::endl;
  rot = AL::Math::Rotation::from3DRotation(0.1f, 0.2f, 0.3f);
  EXPECT_TRUE(rot.isRotation(pEps));

  // std::cout << "test isRotation 2" << std::endl;
  rot = AL::Math::Rotation::fromRotX(0.6f);
  EXPECT_TRUE(rot.isRotation(pEps));

  // std::cout << "test isRotation 3" << std::endl;
  rot = AL::Math::Rotation::fromRotX(-0.6f);
  EXPECT_TRUE(rot.isRotation(pEps));

  // std::cout << "test isRotation 4" << std::endl;
  rot = AL::Math::Rotation::fromRotY(0.6f);
  EXPECT_TRUE(rot.isRotation(pEps));

  // std::cout << "test isRotation 5" << std::endl;
  rot = AL::Math::Rotation::fromRotY(-0.6f);
  EXPECT_TRUE(rot.isRotation(pEps));

  // std::cout << "test isRotation 6" << std::endl;
  rot = AL::Math::Rotation::fromRotZ(0.6f);
  EXPECT_TRUE(rot.isRotation(pEps));

  // std::cout << "test isRotation 7" << std::endl;
  rot = AL::Math::Rotation::fromRotZ(-0.6f);
  EXPECT_TRUE(rot.isRotation(pEps));

  // std::cout << "test isRotation r1_c1" << std::endl;
  rot = AL::Math::Rotation::from3DRotation(0.1f, 0.2f, 0.3f);
  rot.r1_c1 += 0.1f;
  EXPECT_FALSE(rot.isRotation(pEps));

  // std::cout << "test isRotation r1_c2" << std::endl;
  rot = AL::Math::Rotation::from3DRotation(0.1f, 0.2f, 0.3f);
  rot.r1_c2 += 0.1f;
  EXPECT_FALSE(rot.isRotation(pEps));

  // std::cout << "test isRotation r1_c3" << std::endl;
  rot = AL::Math::Rotation::from3DRotation(0.1f, 0.2f, 0.3f);
  rot.r1_c3 += 0.1f;
  EXPECT_FALSE(rot.isRotation(pEps));

  // std::cout << "test isRotation r2_c1" << std::endl;
  rot = AL::Math::Rotation::from3DRotation(0.1f, 0.2f, 0.3f);
  rot.r2_c1 += 0.1f;
  EXPECT_FALSE(rot.isRotation(pEps));

  // std::cout << "test isRotation r2_c2" << std::endl;
  rot = AL::Math::Rotation::from3DRotation(0.1f, 0.2f, 0.3f);
  rot.r2_c2 += 0.1f;
  EXPECT_FALSE(rot.isRotation(pEps));

  // std::cout << "test isRotation r2_c3" << std::endl;
  rot = AL::Math::Rotation::from3DRotation(0.1f, 0.2f, 0.3f);
  rot.r2_c3 += 0.1f;
  EXPECT_FALSE(rot.isRotation(pEps));

  // std::cout << "test isRotation r3_c1" << std::endl;
  rot = AL::Math::Rotation::from3DRotation(0.1f, 0.2f, 0.3f);
  rot.r3_c1 += 0.1f;
  EXPECT_FALSE(rot.isRotation(pEps));

  // std::cout << "test isRotation r3_c2" << std::endl;
  rot = AL::Math::Rotation::from3DRotation(0.1f, 0.2f, 0.3f);
  rot.r3_c2 += 0.1f;
  EXPECT_FALSE(rot.isRotation(pEps));

  // std::cout << "test isRotation r3_c3" << std::endl;
  rot = AL::Math::Rotation::from3DRotation(0.1f, 0.2f, 0.3f);
  rot.r3_c3 += 0.1f;
  EXPECT_FALSE(rot.isRotation(pEps));
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

TEST(ALRotationTest, normalizeRotation)
{
  AL::Math::Rotation rot1;
  AL::Math::Rotation rot2;
  AL::Math::Rotation rot3;

  // case null data
  rot1 = AL::Math::Rotation();
  rot1.r1_c1 = 0.0f;
  rot1.r2_c2 = 0.0f;
  rot1.r3_c3 = 0.0f;
  EXPECT_FALSE(rot1.isRotation());
  AL::Math::normalizeRotation(rot1);
  EXPECT_TRUE(rot1.isRotation());
  EXPECT_TRUE(rot1.isNear(AL::Math::Rotation()));

  // case not normalized
  rot1.r1_c1 = 0.5f;
  rot1.r2_c2 = 0.5f;
  rot1.r3_c3 = 0.5f;
  EXPECT_FALSE(rot1.isRotation());
  AL::Math::normalizeRotation(rot1);
  EXPECT_TRUE(rot1.isRotation());
  EXPECT_TRUE(rot1.isNear(AL::Math::Rotation()));

  for (unsigned int i=-360; i<360; ++i)
  {
    float angleX = static_cast<float>(i)*AL::Math::TO_RAD;
    for (unsigned int j=-360; j<360; ++j)
    {
      float angleY = static_cast<float>(j)*AL::Math::TO_RAD;
      for (unsigned int k=-360; k<360; ++k)
      {
        float angleZ = static_cast<float>(k)*AL::Math::TO_RAD;
        rot1 = AL::Math::Rotation::fromRotX(angleX)*\
            AL::Math::Rotation::fromRotY(angleY)*\
            AL::Math::Rotation::fromRotZ(angleZ);

        rot2 = rot1;
        rot3 = rot1;

        rot1.r1_c1 += 0.01f;
        rot1.r1_c2 += 0.01f;
        rot1.r1_c3 += 0.01f;

        rot1.r2_c1 += 0.01f;
        rot1.r2_c2 += 0.01f;
        rot1.r2_c3 += 0.01f;

        rot1.r3_c1 += 0.01f;
        rot1.r3_c2 += 0.01f;
        rot1.r3_c3 += 0.01f;
        EXPECT_FALSE(rot1.isRotation());
        AL::Math::normalizeRotation(rot1);
        EXPECT_TRUE(rot1.isRotation());

        EXPECT_TRUE(rot2.isRotation());
        AL::Math::normalizeRotation(rot2);
        EXPECT_TRUE(rot2.isRotation());
        EXPECT_TRUE(rot2.isNear(rot3));
      }
    }
  }
}

TEST(ALRotationTest, toVector)
{
  const float eps = 1e-4f;
  std::vector<float> input(9, 0.0f);
  input.at(0) = 1.0f;
  input.at(4) = 1.0f;
  input.at(8) = 1.0f;

  const AL::Math::Rotation rot(input);
  const std::vector<float> vec = rot.toVector();

  EXPECT_TRUE(vec.size()==9);
  EXPECT_NEAR(rot.r1_c1, vec.at(0), eps);
  EXPECT_NEAR(rot.r1_c2, vec.at(1), eps);
  EXPECT_NEAR(rot.r1_c3, vec.at(2), eps);

  EXPECT_NEAR(rot.r2_c1, vec.at(3), eps);
  EXPECT_NEAR(rot.r2_c2, vec.at(4), eps);
  EXPECT_NEAR(rot.r2_c3, vec.at(5), eps);

  EXPECT_NEAR(rot.r3_c1, vec.at(6), eps);
  EXPECT_NEAR(rot.r3_c2, vec.at(7), eps);
  EXPECT_NEAR(rot.r3_c3, vec.at(8), eps);
}
