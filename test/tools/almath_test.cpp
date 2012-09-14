/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
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

TEST(ALMathTest, quaternionOperator)
{
  AL::Math::Quaternion pQuat;
  AL::Math::Position3D pPos;
  AL::Math::Position3D result;
  result = pQuat * pPos;
  EXPECT_NEAR(pPos.x, 0.0f, 1e-6f);
  EXPECT_NEAR(pPos.y, 0.0f, 1e-6f);
  EXPECT_NEAR(pPos.z, 0.0f, 1e-6f);

  // Test 90° rotation around the x axis.
  pQuat = AL::Math::Quaternion(0.7071f, 0.7071f, 0.0f, 0.0f);
  pPos = AL::Math::Position3D(0.0f, 1.0f, 0.0f);
  result = pQuat * pPos;
  EXPECT_NEAR(result.x, 0.0f, 1e-4f);
  EXPECT_NEAR(result.y, 0.0f, 1e-4f);
  EXPECT_NEAR(result.z, 1.0f, 1e-4f);

  // Test 90° rotation around the y axis.
  pQuat = AL::Math::Quaternion(0.7071f, 0.0f, 0.7071f, 0.0f);
  pPos = AL::Math::Position3D(0.0f, 0.0f, 1.0f);
  result = pQuat * pPos;
  EXPECT_NEAR(result.x, 1.0f, 1e-4f);
  EXPECT_NEAR(result.y, 0.0f, 1e-4f);
  EXPECT_NEAR(result.z, 0.0f, 1e-4f);

  // Test 90° rotation around the z axis.
  pQuat = AL::Math::Quaternion(0.7071f, 0.0f, 0.0f, 0.7071f);
  pPos = AL::Math::Position3D(1.0f, 0.0f, 0.0f);
  result = pQuat * pPos;
  EXPECT_NEAR(result.x, 0.0f, 1e-4f);
  EXPECT_NEAR(result.y, 1.0f, 1e-4f);
  EXPECT_NEAR(result.z, 0.0f, 1e-4f);

}

TEST(ALMathTest, position6DFromPose2DInPlace)
{
  AL::Math::Pose2D pPose2d = AL::Math::Pose2D(0.1f, 0.2f, 0.3f);
  AL::Math::Position6D pPose6dExpected = AL::Math::Position6D(0.1f, 0.2f, 0.0f, 0.0f, 0.0f, 0.3f);

  AL::Math::Position6D pPose6dComputed;
  AL::Math::position6DFromPose2DInPlace(
        pPose2d,
        pPose6dComputed);

  EXPECT_TRUE(pPose6dComputed.isNear(pPose6dExpected, 0.0001f));
}

TEST(ALMathTest, position6DFromPose2D)
{
  AL::Math::Pose2D pPose2d = AL::Math::Pose2D(0.1f, 0.2f, 0.3f);
  AL::Math::Position6D pPose6dExpected = AL::Math::Position6D(0.1f, 0.2f, 0.0f, 0.0f, 0.0f, 0.3f);

  AL::Math::Position6D pPose6dComputed = AL::Math::position6DFromPose2D(pPose2d);

  EXPECT_TRUE(pPose6dComputed.isNear(pPose6dExpected, 0.0001f));
}

TEST(ALMathTest, pose2DFromPosition6DInPlace)
{
  AL::Math::Position6D pPose6d = AL::Math::Position6D(0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f);
  AL::Math::Pose2D pPose2dExpected = AL::Math::Pose2D(0.1f, 0.2f, 0.6f);

  AL::Math::Pose2D pPose2dComputed;
  AL::Math::pose2DFromPosition6DInPlace(pPose6d, pPose2dComputed);

  EXPECT_TRUE(pPose2dComputed.isNear(pPose2dExpected, 0.0001f));
}

TEST(ALMathTest, pose2DFromPosition6D)
{
  AL::Math::Position6D pPose6d = AL::Math::Position6D(0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f);
  AL::Math::Pose2D pPose2dExpected = AL::Math::Pose2D(0.1f, 0.2f, 0.6f);

  AL::Math::Pose2D pPose2dComputed = AL::Math::pose2DFromPosition6D(pPose6d);

  EXPECT_TRUE(pPose2dComputed.isNear(pPose2dExpected, 0.0001f));
}


