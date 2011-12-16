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
#include <almath/tools/almath.h>

#include <almath/tools/altrigonometry.h>

#include <gtest/gtest.h>
#include <stdexcept>

TEST(ALMathTest, clipData)
{
  float pMin  = -0.2f;
  float pMax  = 0.5f;
  float pData = 0.4f;
  EXPECT_FALSE(AL::Math::clipData(pMin, pMax, pData));

  pMin  = -0.2f;
  pMax  = 0.5f;
  pData = 0.6f;
  EXPECT_TRUE(AL::Math::clipData(pMin, pMax, pData));
  EXPECT_NEAR(pData, pMax, 0.0001f);

  pMin  = -0.2f;
  pMax  = 0.5f;
  pData = -0.1f;
  EXPECT_FALSE(AL::Math::clipData(pMin, pMax, pData));

  pMin  = -0.2f;
  pMax  = 0.5f;
  pData = -0.3f;
  EXPECT_TRUE(AL::Math::clipData(pMin, pMax, pData));
  EXPECT_NEAR(pData, pMin, 0.0001f);
}


TEST(ALMathTest, Position6DFromVelocity6D)
{
  AL::Math::Velocity6D pVIn    = AL::Math::Velocity6D(0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f);
  AL::Math::Position6D pPosIn  = AL::Math::position6DFromVelocity6D(pVIn);

  AL::Math::Position6D pPosOut = AL::Math::Position6D(0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f);

  EXPECT_TRUE(pPosIn.isNear(pPosOut, 0.0001f));
}

TEST(ALMathTest, variousOperator)
{
  //inline Position3D operator*(
  //  const Rotation&   pRot,
  //  const Position3D& pPos)
  AL::Math::Rotation   pRot;
  AL::Math::Position3D pPos3D;
  AL::Math::Position3D pPosIn = pRot*pPos3D;
  AL::Math::Position3D pPosOut = AL::Math::Position3D();
  EXPECT_TRUE(pPosIn.isNear(pPosOut, 0.0001f));

  pRot = AL::Math::Rotation();
  pPos3D = AL::Math::Position3D(1.0f, -1.0f, 0.5f);
  pPosIn = pRot*pPos3D;
  pPosOut = AL::Math::Position3D(1.0f, -1.0f, 0.5f);
  EXPECT_TRUE(pPosIn.isNear(pPosOut, 0.0001f));

  pRot = AL::Math::Rotation::fromRotX(0.5f);
  pPos3D = AL::Math::Position3D(1.0f, -1.0f, 0.5f);
  pPosIn = pRot*pPos3D;
  pPosOut = AL::Math::Position3D(1.00000000000000f, -1.11729533119247f, -0.04063425765902f);
  EXPECT_TRUE(pPosIn.isNear(pPosOut, 0.0001f));

  //inline Velocity6D operator*(
  //  const float       pK,
  //  const Position6D& pDelta)
  float pK = 10.0f;
  AL::Math::Position6D pPos6D = AL::Math::Position6D(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f);
  AL::Math::Velocity6D pVel6DIn = pK*pPos6D;
  AL::Math::Velocity6D pVel6DOut = AL::Math::Velocity6D(10.0f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f);
  EXPECT_TRUE(pVel6DIn.isNear(pVel6DOut, 0.0001f));
}

TEST(ALMathTest, isLeft)
{
}


TEST(ALMathTest, FilterPosition6D)
{
}

TEST(ALMathTest, AxisMaskToPosition6DOn)
{
}

TEST(ALMathTest, AxisMaskToPosition6DOff)
{
}

TEST(ALMathTest, AxisMaskToVelocity6DOn)
{
}

TEST(ALMathTest, RotationFromAngleDirection)
{
  AL::Math::Rotation pRotOut;
  float pTheta = 0.0f;
  AL::Math::Position3D pPos = AL::Math::Position3D(0.0f, 0.0f, 0.0f);

  // ****** test 0 ****** //
  ASSERT_THROW(AL::Math::rotationFromAngleDirection(pTheta, pPos), std::runtime_error);

  // ****** test 1 ****** //
  pTheta = 0.0f;
  pPos = AL::Math::Position3D(1.0f, 0.0f, 0.0f);
  pRotOut = AL::Math::rotationFromAngleDirection(pTheta, pPos);
  EXPECT_TRUE(pRotOut.isNear(AL::Math::Rotation()));

  // ****** test 2 ****** //
  pTheta = 0.0f;
  pPos = AL::Math::Position3D(0.0f, 1.0f, 0.0f);
  pRotOut = AL::Math::rotationFromAngleDirection(pTheta, pPos);
  EXPECT_TRUE(pRotOut.isNear(AL::Math::Rotation()));

  // ****** test 3 ****** //
  pTheta = 0.0f;
  pPos = AL::Math::Position3D(0.0f, 0.0f, 1.0f);
  pRotOut = AL::Math::rotationFromAngleDirection(pTheta, pPos);
  EXPECT_TRUE(pRotOut.isNear(AL::Math::Rotation()));


  // ****** test 4 ****** //
  pTheta = 15.0f*AL::Math::TO_RAD;
  pPos = AL::Math::Position3D(1.0f, 0.0f, 0.0f);
  pRotOut = AL::Math::rotationFromAngleDirection(pTheta, pPos);
  EXPECT_TRUE(pRotOut.isNear(AL::Math::Rotation::fromRotX(pTheta)));
  EXPECT_FALSE(pRotOut.isNear(AL::Math::Rotation()));

  // ****** test 5 ****** //
  pTheta = 15.0f*AL::Math::TO_RAD;
  pPos = AL::Math::Position3D(0.0f, 1.0f, 0.0f);
  pRotOut = AL::Math::rotationFromAngleDirection(pTheta, pPos);
  EXPECT_TRUE(pRotOut.isNear(AL::Math::Rotation::fromRotY(pTheta)));
  EXPECT_FALSE(pRotOut.isNear(AL::Math::Rotation()));

  // ****** test 6 ****** //
  pTheta = 15.0f*AL::Math::TO_RAD;
  pPos = AL::Math::Position3D(0.0f, 0.0f, 1.0f);
  pRotOut = AL::Math::rotationFromAngleDirection(pTheta, pPos);
  EXPECT_TRUE(pRotOut.isNear(AL::Math::Rotation::fromRotZ(pTheta)));
  EXPECT_FALSE(pRotOut.isNear(AL::Math::Rotation()));
}
