/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */
#include <almath/tools/almath.h>

#include <almath/tools/altrigonometry.h>
#include <almath/tools/almathio.h>
#include <almath/tools/altransformhelpers.h>
#include <gtest/gtest.h>
#include <stdexcept>

TEST(ALMathTest, moduloPI)
{
  float epsilon = 0.001f;
  float pAngle  = 0.0f;

  // case 0
  pAngle = 0.0f;
  AL::Math::modulo2PIInPlace(pAngle);
  EXPECT_NEAR(pAngle, 0.0f, epsilon);

  pAngle = 0.99f*AL::Math::PI;
  AL::Math::modulo2PIInPlace(pAngle);
  EXPECT_NEAR(pAngle, 0.99f*AL::Math::PI, epsilon);

  pAngle = -0.99f*AL::Math::PI;
  AL::Math::modulo2PIInPlace(pAngle);
  EXPECT_NEAR(pAngle, -0.99f*AL::Math::PI, epsilon);

  // case 1
  pAngle = AL::Math::PI + 0.5f;
  AL::Math::modulo2PIInPlace(pAngle);
  EXPECT_NEAR(pAngle, 0.5f-AL::Math::PI, epsilon);

  pAngle = -AL::Math::PI - 0.5f;
  AL::Math::modulo2PIInPlace(pAngle);
  EXPECT_NEAR(pAngle, -0.5f+AL::Math::PI, epsilon);

  // case 2
  pAngle  = 10.0f*AL::Math::PI + 0.5f;
  AL::Math::modulo2PIInPlace(pAngle);
  EXPECT_NEAR(pAngle, 0.5f, epsilon);

  pAngle  = -10.0f*AL::Math::PI - 0.5f;
  AL::Math::modulo2PIInPlace(pAngle);
  EXPECT_NEAR(pAngle, -0.5f, epsilon);

  // case 3
  pAngle  = 11.0f*AL::Math::PI + 0.5f;
  AL::Math::modulo2PIInPlace(pAngle);
  EXPECT_NEAR(pAngle, 0.5f-AL::Math::PI, epsilon);

  pAngle  = -11.0f*AL::Math::PI - 0.5f;
  AL::Math::modulo2PIInPlace(pAngle);
  EXPECT_NEAR(pAngle, -0.5f+AL::Math::PI, epsilon);

  pAngle = 1.0f;
  EXPECT_NEAR(AL::Math::modulo2PI(pAngle), 1.0f, 1e-4f);
  pAngle = 2 * AL::Math::PI;
  EXPECT_NEAR(AL::Math::modulo2PI(pAngle), 0.0f, 1e-4f);
  pAngle = -5 * AL::Math::PI_2;
  EXPECT_NEAR(AL::Math::modulo2PI(pAngle), -AL::Math::PI_2, 1e-4f);
  pAngle = 3 * AL::Math::PI_2;
  EXPECT_NEAR(AL::Math::modulo2PI(pAngle), -AL::Math::PI_2, 1e-4f);
  pAngle = AL::Math::PI;
  EXPECT_NEAR(AL::Math::modulo2PI(pAngle), AL::Math::PI, 1e-4f);
}


TEST(ALMathTest, mean) {
  EXPECT_THROW(AL::Math::meanAngle(std::vector<float>()),
               std::runtime_error);
  EXPECT_THROW(AL::Math::weightedMeanAngle(std::vector<float>(),
                                           std::vector<float>()),
               std::runtime_error);
  std::vector<float> angles;
  std::vector<float> weights;
  angles.push_back(AL::Math::PI);
  EXPECT_NEAR(AL::Math::modulo2PI(AL::Math::meanAngle(angles) - AL::Math::PI),
              0.f, 1e-3f);
  EXPECT_THROW(AL::Math::weightedMeanAngle(angles, weights),
               std::runtime_error);
  weights.push_back(0.5f);
  EXPECT_NEAR(AL::Math::modulo2PI(AL::Math::weightedMeanAngle(angles, weights)
                                  - AL::Math::PI),
              0.f, 1e-3f);
  angles.push_back(-AL::Math::PI);
  weights.push_back(0.5f);
  EXPECT_NEAR(AL::Math::modulo2PI(AL::Math::meanAngle(angles) - AL::Math::PI),
              0.f, 1e-3f);
  EXPECT_NEAR(AL::Math::modulo2PI(AL::Math::weightedMeanAngle(angles, weights)
                                  - AL::Math::PI),
              0.f, 1e-3f);
  angles[0] = AL::Math::PI_2;
  EXPECT_NEAR(AL::Math::modulo2PI(AL::Math::meanAngle(angles) - 0.75f*AL::Math::PI),
              0.f, 1e-3f);
  EXPECT_NEAR(AL::Math::modulo2PI(AL::Math::weightedMeanAngle(angles, weights)
                                  - 0.75f*AL::Math::PI),
              0.f, 1e-3f);
  angles[0] = 0.f;
  EXPECT_THROW(AL::Math::meanAngle(angles), std::runtime_error);
  EXPECT_THROW(AL::Math::weightedMeanAngle(angles, weights), std::runtime_error);
  angles[0] = 0.f;
  weights[0] = 1.f;
  angles[1] = 0.5f*AL::Math::PI;
  weights[1] = 0.5f;
  EXPECT_NEAR(AL::Math::weightedMeanAngle(angles, weights),
              std::atan(0.5f), 1e-3f);
  weights[1] = 0.f;
  EXPECT_THROW(AL::Math::weightedMeanAngle(angles, weights),
              std::exception);
  weights[1] = -1.f;
  EXPECT_THROW(AL::Math::weightedMeanAngle(angles, weights),
               std::runtime_error);
}

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

TEST(ALMathTest, clipDataVector)
{
  const float lEpsilon = 0.0001f;
  std::vector<float> data(5, -1.0f);
  bool isClipped = AL::Math::clipData(0.0f, 1.0f, data);
  EXPECT_TRUE(isClipped);
  for (unsigned int i=0; i<data.size(); ++i)
  {
    EXPECT_NEAR(data[i], 0.0f, lEpsilon);
  }

  data.clear();
  data.push_back(-1.0f);
  data.push_back(1.0f);
  data.push_back(2.0f);
  data.push_back(-0.5f);
  data.push_back(3.0f);
  std::vector<float> expectedData;
  expectedData.push_back(0.0f);
  expectedData.push_back(1.0f);
  expectedData.push_back(1.0f);
  expectedData.push_back(0.0f);
  expectedData.push_back(1.0f);

  isClipped = AL::Math::clipData(0.0f, 1.0f, data);
  EXPECT_TRUE(data.size()==expectedData.size());
  EXPECT_TRUE(isClipped);
  for (unsigned int i=0; i<data.size(); ++i)
  {
    EXPECT_NEAR(data[i], expectedData[i], lEpsilon);
  }

  data.clear();
  data.push_back(0.0f);
  data.push_back(0.1f);
  data.push_back(0.2f);
  data.push_back(0.3f);
  data.push_back(0.4f);
  expectedData = data;
  isClipped = AL::Math::clipData(0.0f, 1.0f, data);
  EXPECT_TRUE(data.size()==expectedData.size());
  EXPECT_FALSE(isClipped);
  for (unsigned int i=0; i<data.size(); ++i)
  {
    EXPECT_NEAR(data[i], expectedData[i], lEpsilon);
  }
}

TEST(ALMathTest, clipDataVectorVector)
{
  const float lEpsilon = 0.0001f;
  std::vector<float> data(5, -1.0f);
  std::vector<std::vector<float> > dataList(5, data);
  bool isClipped = AL::Math::clipData(0.0f, 1.0f, dataList);
  EXPECT_TRUE(isClipped);
  for (unsigned int i=0; i<dataList.size(); ++i)
  {
    for (unsigned int j=0; j<dataList[i].size(); ++j)
    {
      EXPECT_NEAR(dataList[i][j], 0.0f, lEpsilon);
    }
  }
}

TEST(ALMathTest, changeReferencePose2D)
{
  float pTheta = 90.0f*AL::Math::TO_RAD;
  AL::Math::Pose2D pPosIn;
  AL::Math::Pose2D pPosOut;

  pPosIn = AL::Math::Pose2D(10.0f, 0.0f, 0.5f);
  AL::Math::changeReferencePose2D(pTheta, pPosIn, pPosOut);
  EXPECT_TRUE(pPosOut.isNear(AL::Math::Pose2D(0.0f, 10.0f, 0.5f), 0.001f));

  pPosIn = AL::Math::Pose2D(0.0f, 10.0f, 0.5f);
  AL::Math::changeReferencePose2D(pTheta, pPosIn, pPosOut);
  EXPECT_TRUE(pPosOut.isNear(AL::Math::Pose2D(-10.0f, 0.0f, 0.5f), 0.001f));

  pPosIn = AL::Math::Pose2D(-10.0f, 0.0f, 0.5f);
  AL::Math::changeReferencePose2D(pTheta, pPosIn, pPosOut);
  EXPECT_TRUE(pPosOut.isNear(AL::Math::Pose2D(0.0f, -10.0f, 0.5f), 0.001f));

  pPosIn = AL::Math::Pose2D(0.0f, -10.0f, 0.5f);
  AL::Math::changeReferencePose2D(pTheta, pPosIn, pPosOut);
  EXPECT_TRUE(pPosOut.isNear(AL::Math::Pose2D(10.0f, 0.0f, 0.5f), 0.001f));
}

TEST(ALMathTest, changeReferencePose2DInPlace)
{
  float pTheta = 90.0f*AL::Math::TO_RAD;
  AL::Math::Pose2D pPosIn;

  pPosIn = AL::Math::Pose2D(10.0f, 0.0f, 0.5f);
  AL::Math::changeReferencePose2DInPlace(pTheta, pPosIn);
  EXPECT_TRUE(pPosIn.isNear(AL::Math::Pose2D(0.0f, 10.0f, 0.5f), 0.001f));

  pPosIn = AL::Math::Pose2D(0.0f, 10.0f, 0.5f);
  AL::Math::changeReferencePose2DInPlace(pTheta, pPosIn);
  EXPECT_TRUE(pPosIn.isNear(AL::Math::Pose2D(-10.0f, 0.0f, 0.5f), 0.001f));

  pPosIn = AL::Math::Pose2D(-10.0f, 0.0f, 0.5f);
  AL::Math::changeReferencePose2DInPlace(pTheta, pPosIn);
  EXPECT_TRUE(pPosIn.isNear(AL::Math::Pose2D(0.0f, -10.0f, 0.5f), 0.001f));

  pPosIn = AL::Math::Pose2D(0.0f, -10.0f, 0.5f);
  AL::Math::changeReferencePose2DInPlace(pTheta, pPosIn);
  EXPECT_TRUE(pPosIn.isNear(AL::Math::Pose2D(10.0f, 0.0f, 0.5f), 0.001f));
}


TEST(ALMathTest, position3DFromPosition6D)
{
  AL::Math::Position6D pPose6d        = AL::Math::Position6D(0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f);
  AL::Math::Position3D pPos3dExpected = AL::Math::Position3D(0.1f, 0.2f, 0.3f);

  AL::Math::Position3D pPose3dResult = AL::Math::position3DFromPosition6D(pPose6d);

  EXPECT_TRUE(pPose3dResult.isNear(pPos3dExpected, 0.0001f));
}

TEST(ALMathTest, position2DFromPose2D)
{
  const AL::Math::Pose2D pPose2d(0.1f, 0.2f, 0.3f);
  const AL::Math::Position2D pPos2dExpected(0.1f, 0.2f);

  const AL::Math::Position2D& pPose2dResult =
      AL::Math::position2DFromPose2D(pPose2d);

  EXPECT_TRUE(pPose2dResult.isNear(pPos2dExpected, 0.0001f));
}

TEST(ALMathTest, pose2DFromPosition2DInPlace)
{
  const AL::Math::Position2D pPosition2d(0.1f, 0.2f);
  const AL::Math::Pose2D pPos2dExpected(0.1f, 0.2f, 0.5f);

  AL::Math::Pose2D pPose2dResult = AL::Math::Pose2D(10.0f, 20.0f, 30.0f);
  AL::Math::pose2DFromPosition2DInPlace(pPosition2d, 0.5f, pPose2dResult);

  EXPECT_TRUE(pPose2dResult.isNear(pPos2dExpected, 0.0001f));
}

TEST(ALMathTest, pose2DFromPosition2D)
{
  const AL::Math::Position2D pPosition2d(0.1f, 0.2f);
  const AL::Math::Pose2D pPos2dExpected(0.1f, 0.2f, 0.0f);

  const AL::Math::Pose2D& pPose2dResult =
      AL::Math::pose2DFromPosition2D(pPosition2d);

  EXPECT_TRUE(pPose2dResult.isNear(pPos2dExpected, 0.0001f));

  const AL::Math::Position2D pPosition2d2(0.1f, 0.2f);
  const AL::Math::Pose2D pPos2dExpected2(0.1f, 0.2f, 0.5f);

  const AL::Math::Pose2D& pPose2dResult2 =
      AL::Math::pose2DFromPosition2D(pPosition2d2, 0.5f);

  EXPECT_TRUE(pPose2dResult2.isNear(pPos2dExpected2, 0.0001f));
}


TEST(ALMathTest, Position6DFromVelocity6D)
{
  const AL::Math::Velocity6D pVIn =
      AL::Math::Velocity6D(0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f);
  const AL::Math::Position6D pPosIn =
      AL::Math::position6DFromVelocity6D(pVIn);

  const AL::Math::Position6D pPosOut =
      AL::Math::Position6D(0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f);

  EXPECT_TRUE(pPosIn.isNear(pPosOut, 0.0001f));
}

TEST(ALMathTest, position2DFromPose2DInPlace)
{
  const AL::Math::Pose2D pose2D = AL::Math::Pose2D(1.0f, 2.0f, 3.0f);
  AL::Math::Position2D position2D = AL::Math::Position2D(10.0f, 20.0f);

  AL::Math::position2DFromPose2DInPlace(pose2D, position2D);
  EXPECT_TRUE(position2D.isNear(AL::Math::Position2D(1.0f, 2.0f), 0.0001f));
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

  //Velocity3D operator* (const Rotation&   pRot, const Velocity3D& pVel);
  AL::Math::Velocity3D pVIn1 = AL::Math::Velocity3D(0.5f, 0.3f, 0.1f);
  AL::Math::Rotation   pRot1 = AL::Math::rotationFromRotZ(AL::Math::PI_2);
  AL::Math::Velocity3D pVIn2 = pRot1*pVIn1;
  AL::Math::Velocity3D pVOut = AL::Math::Velocity3D(-0.3f, 0.5f, 0.1f);
  EXPECT_TRUE(pVIn2.isNear(pVOut, 0.0001f));

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
  const AL::Math::Pose2D pPose2d = AL::Math::Pose2D(0.1f, 0.2f, 0.3f);
  const AL::Math::Position6D pPose6dExpected =
      AL::Math::Position6D(0.1f, 0.2f, 0.0f, 0.0f, 0.0f, 0.3f);

  AL::Math::Position6D pPose6dComputed =
      AL::Math::Position6D(10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f);
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

TEST(ALMathTest, position6DFromPosition3DInPlace)
{
  const AL::Math::Position3D position3D =
      AL::Math::Position3D(0.1f, 0.2f, 0.3f);

  AL::Math::Position6D position6D =
      AL::Math::Position6D(10.1f, 10.2f, 10.0f, 10.0f, 10.0f, 10.3f);

  AL::Math::position6DFromPosition3DInPlace(position3D, position6D);

  EXPECT_TRUE(position6D.isNear(
                AL::Math::Position6D(0.1f, 0.2f, 0.3f, 0.0f, 0.0f, 0.0f), 0.0001f));
}

TEST(ALMathTest, position6DFromPosition3D)
{
  const AL::Math::Position3D position3D =
      AL::Math::Position3D(0.1f, 0.2f, 0.3f);

  const AL::Math::Position6D position6D =
      AL::Math::position6DFromPosition3D(position3D);

  EXPECT_TRUE(position6D.isNear(
                AL::Math::Position6D(0.1f, 0.2f, 0.3f, 0.0f, 0.0f, 0.0f), 0.0001f));
}


TEST(ALMathTest, multiplicationPose2DPosition2D)
{
  AL::Math::Pose2D pVal;
  AL::Math::Position2D pPos;
  AL::Math::Position2D pExpected;
  AL::Math::Position2D pResult;

  pVal      = AL::Math::Pose2D(0.0f, 0.0f, AL::Math::PI_2);
  pPos      = AL::Math::Position2D(1.0f, 0.0f);
  pExpected = AL::Math::Position2D(0.0f, 1.0f);
  pResult   = pVal*pPos;
  EXPECT_TRUE(pResult.isNear(pExpected, 0.0001f));

  pVal      = AL::Math::Pose2D(0.0f, 0.0f, AL::Math::PI_2);
  pPos      = AL::Math::Position2D(0.0f, 1.0f);
  pExpected = AL::Math::Position2D(-1.0f, 0.0f);
  pResult   = pVal*pPos;
  EXPECT_TRUE(pResult.isNear(pExpected, 0.0001f));

  pVal      = AL::Math::Pose2D(1.0f, 1.0f, -AL::Math::PI_2);
  pPos      = AL::Math::Position2D(1.0f, 0.0f);
  pExpected = AL::Math::Position2D(1.0f, 0.0f);
  pResult   = pVal*pPos;
  EXPECT_TRUE(pResult.isNear(pExpected, 0.0001f));
}

TEST(ALMathTest, quaternionFromRotation3D)
{
  for (unsigned int i=0; i<360; ++i)
  {
    const float angleX = static_cast<float>(i)*AL::Math::TO_RAD;
    const AL::Math::Quaternion quatX =
        AL::Math::quaternionFromAngleAndAxisRotation(angleX, 1.0f, 0.0f, 0.0f);
    for (unsigned int j=0; j<360; ++j)
    {
      const float angleY = static_cast<float>(j)*AL::Math::TO_RAD;
      const AL::Math::Quaternion quatY =
          AL::Math::quaternionFromAngleAndAxisRotation(angleY, 0.0f, 1.0f, 0.0f);
      for (unsigned int k=0; k<360; ++k)
      {
        const float angleZ = static_cast<float>(k)*AL::Math::TO_RAD;
        const AL::Math::Quaternion quatZ =
            AL::Math::quaternionFromAngleAndAxisRotation(angleZ, 0.0f, 0.0f, 1.0f);

        const AL::Math::Quaternion quatExpected = quatZ*quatY*quatX;
        const AL::Math::Rotation3D rot3DZYX(angleX, angleY, angleZ);
        const AL::Math::Quaternion quatResult =
            AL::Math::quaternionFromRotation3D(rot3DZYX);
        EXPECT_TRUE(quatResult.isNear(quatExpected, 0.001f));
      }
    }
  }
}

TEST(ALMathTest, rotation3DFromQuaternion1)
{
  const float lAngleX = 0.5f;
  const float lAngleY = -0.3f;
  AL::Math::Quaternion quat =
      AL::Math::quaternionFromAngleAndAxisRotation(lAngleY, 0.0f, 1.0f , 0.0f)*
      AL::Math::quaternionFromAngleAndAxisRotation(lAngleX, 1.0f, 0.0f , 0.0f);

  AL::Math::Rotation3D lRotation3D;
  AL::Math::rotation3DFromQuaternion(quat, lRotation3D);
  EXPECT_NEAR(lRotation3D.wx, lAngleX, 0.0001f);
  EXPECT_NEAR(lRotation3D.wy, lAngleY, 0.0001f);

  AL::Math::Quaternion lQuat;
  AL::Math::quaternionFromRotation3D(lRotation3D, lQuat);
  EXPECT_NEAR(lQuat.w, quat.w, 0.0001f);
  EXPECT_NEAR(lQuat.x, quat.x, 0.0001f);
  EXPECT_NEAR(lQuat.y, quat.y, 0.0001f);
  EXPECT_NEAR(lQuat.z, quat.z, 0.0001f);

  // new test jory
  AL::Math::Rotation3D rotation3DRef(0.0213f, 0.0139f, 0.02619f);
  quat = AL::Math::Quaternion(0.99983f, 0.01056f, 0.00712f, 0.01302f);
  quat = quat.normalize();
  AL::Math::Rotation3D rotation3D;
  AL::Math::rotation3DFromQuaternion(quat, rotation3D);
  EXPECT_TRUE(rotation3D.isNear(rotation3DRef, 0.0001f));

  rotation3DRef = AL::Math::Rotation3D(-0.00350f, 0.0f, 0.0314f);
  quat = AL::Math::Quaternion(0.99988f, -0.00175f,
                              -0.00003f, 0.01571f);

  AL::Math::rotation3DFromQuaternion(quat, rotation3D);

  EXPECT_TRUE(rotation3D.isNear(rotation3DRef, 0.0001f));

  const float wx = -180.0f*AL::Math::TO_RAD;
  const float wy = -90.0f*AL::Math::TO_RAD;
  const float wz = -175.0f*AL::Math::TO_RAD;
  quat =
      AL::Math::quaternionFromAngleAndAxisRotation(wz, 0.0f, 0.0f, 1.0f)*
      AL::Math::quaternionFromAngleAndAxisRotation(wy, 0.0f, 1.0f, 0.0f)*
      AL::Math::quaternionFromAngleAndAxisRotation(wx, 1.0f, 0.0f, 0.0f);
  const AL::Math::Rotation3D rot3D = AL::Math::rotation3DFromQuaternion(quat);
  EXPECT_TRUE(rot3D.wx == rot3D.wx);
  EXPECT_TRUE(rot3D.wy == rot3D.wy);
  EXPECT_TRUE(rot3D.wz == rot3D.wz);
}

TEST(ALMathTest, rotation3DFromQuaternion2)
{
  // function quaternionFromRotation3D must be check before
  for (int i=-180; i<180; ++i)
  {
    const float angleX = static_cast<float>(i)*AL::Math::TO_RAD;
    const AL::Math::Quaternion quatX =
        AL::Math::quaternionFromAngleAndAxisRotation(angleX, 1.0f, 0.0f, 0.0f);
    for (int j=-180; j<180; ++j)
    {
      const float angleY = static_cast<float>(j)*AL::Math::TO_RAD;
      const AL::Math::Quaternion quatY =
          AL::Math::quaternionFromAngleAndAxisRotation(angleY, 0.0f, 1.0f, 0.0f);
      for (int k=-180; k<180; ++k)
      {
        const float angleZ = static_cast<float>(k)*AL::Math::TO_RAD;
        const AL::Math::Quaternion quatZ =
            AL::Math::quaternionFromAngleAndAxisRotation(angleZ, 0.0f, 0.0f, 1.0f);

        const AL::Math::Quaternion quatExpected = quatZ*quatY*quatX;
        const AL::Math::Rotation3D rot3D =
            AL::Math::rotation3DFromQuaternion(quatExpected);

        const AL::Math::Quaternion quatResult =
            AL::Math::quaternionFromRotation3D(rot3D);

        EXPECT_TRUE(quatExpected.isNear(quatResult, 0.001f));
      }
    }
  }
}

TEST(ALMathTest, quaternionPosition3DFromPosition6D)
{
  // function quaternionFromRotation3D must be check before
  const float lEpsilon = 0.001f;
  const AL::Math::Position6D pos6D =
      AL::Math::Position6D(0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f);
  AL::Math::Quaternion qua;
  AL::Math::Position3D pos3D;

  AL::Math::quaternionPosition3DFromPosition6D(pos6D, qua, pos3D);

  const AL::Math::Quaternion quaExpected =
      AL::Math::quaternionFromRotation3D(AL::Math::Rotation3D(0.4f, 0.5f, 0.6f));
  EXPECT_TRUE(qua.isNear(quaExpected, lEpsilon));
  EXPECT_TRUE(pos3D.isNear(AL::Math::Position3D(0.1f, 0.2f, 0.3f), lEpsilon));
}

TEST(ALMathTest, pointMassRotationalInertia)
{
  AL::Math::Position3D pos123(1.f, 2.f, 3.f);
  std::vector<float> inertia;

  pointMassRotationalInertia(0.f, pos123, inertia);
  ASSERT_EQ(9, inertia.size());
  ASSERT_EQ(std::vector<float>(9, 0.f), inertia);

  pointMassRotationalInertia(1.f, AL::Math::Position3D(), inertia);
  ASSERT_EQ(std::vector<float>(9, 0.f), inertia);

  float m10 = 10.f;
  pointMassRotationalInertia(m10, pos123, inertia);
  // check symmetry
  ASSERT_TRUE(inertia[1] == inertia[3]);
  ASSERT_TRUE(inertia[2] == inertia[6]);
  ASSERT_TRUE(inertia[5] == inertia[7]);
  // check values
  ASSERT_EQ(m10*(4+9), inertia[0]);
  ASSERT_EQ(m10*(1+9), inertia[4]);
  ASSERT_EQ(m10*(1+4), inertia[8]);
  ASSERT_EQ(-m10*1*2, inertia[1]);
  ASSERT_EQ(-m10*1*3, inertia[2]);
  ASSERT_EQ(-m10*2*3, inertia[5]);
}
