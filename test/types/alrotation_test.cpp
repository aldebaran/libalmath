/*
 * Copyright (c) 2012, Aldebaran Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Aldebaran Robotics nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Aldebaran Robotics BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

