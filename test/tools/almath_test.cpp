/**
* @author Cyrille Collette - ccollette@aldebaran-robotics.com
* Aldebaran Robotics (c) 2009 All Rights Reserved
*
*/
#include <almath/tools/almath.h>
#include "../almathtestutils.h"

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


TEST(ALMathTest, diffLog)
{
  // ************ DiffLog ************
  AL::Math::Velocity6D pMathVTmp = AL::Math::Velocity6D();
  AL::Math::Velocity6D pMathVIn  = AL::Math::Velocity6D();
  AL::Math::Velocity6D pMathVOut = AL::Math::Velocity6D();

  AL::Math::diffLog(pMathVTmp, pMathVIn, pMathVOut);
  compareVelocity6D(pMathVOut, AL::Math::Velocity6D());

  pMathVTmp = AL::Math::Velocity6D();
  pMathVIn  = AL::Math::Velocity6D(1.1f, 1.2f, 1.3f, 1.4f, 1.5f, 1.6f);
  pMathVOut = AL::Math::Velocity6D();

  AL::Math::diffLog(pMathVTmp, pMathVIn, pMathVOut);
  compareVelocity6D(pMathVOut, AL::Math::Velocity6D(1.1f, 1.2f, 1.3f, 1.4f, 1.5f, 1.6f));


  pMathVTmp = AL::Math::Velocity6D(0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f);
  pMathVIn  = AL::Math::Velocity6D(1.0f, 0.4f, -0.2f, -0.5f, 0.6f, -0.7f);
  pMathVOut = AL::Math::Velocity6D();

  AL::Math::diffLog(pMathVTmp, pMathVIn, pMathVOut);
  compareVelocity6D(pMathVOut, AL::Math::Velocity6D(0.64771665004999f, 0.66168175212585f, -0.24379584098681f, -0.83834409132305f, 0.54968881246280f, -0.43251128283696f));
}

TEST(ALMathTest, invDiffLog)
{
  // ************ InvDiffLog ************
  AL::Math::Velocity6D pMathVTmp = AL::Math::Velocity6D();
  AL::Math::Velocity6D pMathVIn  = AL::Math::Velocity6D();
  AL::Math::Velocity6D pMathVOut = AL::Math::Velocity6D();

  AL::Math::invDiffLog(pMathVTmp, pMathVIn, pMathVOut);
  compareVelocity6D(pMathVOut, AL::Math::Velocity6D());

  pMathVTmp = AL::Math::Velocity6D();
  pMathVIn  = AL::Math::Velocity6D(1.1f, 1.2f, 1.3f, 1.4f, 1.5f, 1.6f);
  pMathVOut = AL::Math::Velocity6D();

  AL::Math::invDiffLog(pMathVTmp, pMathVIn, pMathVOut);
  compareVelocity6D(pMathVOut, AL::Math::Velocity6D(1.1f, 1.2f, 1.3f, 1.4f, 1.5f, 1.6f));


  pMathVTmp = AL::Math::Velocity6D(0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f);
  pMathVIn  = AL::Math::Velocity6D(1.0f, 0.4f, -0.2f, -0.5f, 0.6f, -0.7f);
  pMathVOut = AL::Math::Velocity6D();

  AL::Math::invDiffLog(pMathVTmp, pMathVIn, pMathVOut);
  compareVelocity6D(pMathVOut, AL::Math::Velocity6D(1.21526851243252f, 0.03119028323490f, 0.01706594106026f, -0.13163715164398f, 0.50211652782035f, -0.86400567208764f));
}

TEST(ALMathTest, Sign)
{
  float pValue = 10.1f;
  EXPECT_NEAR(AL::Math::sign(pValue), 1.0f, 0.0001f);

  pValue = -5.0f;
  EXPECT_NEAR(AL::Math::sign(pValue), -1.0f, 0.0001f);

  pValue = 0.0f;
  EXPECT_NEAR(AL::Math::sign(pValue), 1.0f, 0.0001f);
}


TEST(ALMathTest, TransformFromPosition3DAndRotation)
{
  AL::Math::Rotation  pRot = AL::Math::Rotation::fromRotX(0.5f);
  AL::Math::Transform pHIn = AL::Math::transformFromPosition3DAndRotation(0.2f, 0.5f, -1.2f, pRot);
  AL::Math::Transform pHOut = AL::Math::Transform::fromRotX(0.5f);
  pHOut.r1_c4 =  0.2f;
  pHOut.r2_c4 =  0.5f;
  pHOut.r3_c4 = -1.2f;
  compareTransform(pHIn, pHOut, 0.0001f);


  pRot = AL::Math::Rotation::fromRotY(0.5f);
  pHIn = AL::Math::transformFromPosition3DAndRotation(0.2f, 0.5f, -1.2f, pRot);
  pHOut = AL::Math::Transform::fromRotY(0.5f);
  pHOut.r1_c4 =  0.2f;
  pHOut.r2_c4 =  0.5f;
  pHOut.r3_c4 = -1.2f;
  compareTransform(pHIn, pHOut, 0.0001f);


  pRot = AL::Math::Rotation::fromRotZ(0.5f);
  pHIn = AL::Math::transformFromPosition3DAndRotation(0.2f, 0.5f, -1.2f, pRot);
  pHOut = AL::Math::Transform::fromRotZ(0.5f);
  pHOut.r1_c4 =  0.2f;
  pHOut.r2_c4 =  0.5f;
  pHOut.r3_c4 = -1.2f;
  compareTransform(pHIn, pHOut, 0.0001f);


  AL::Math::Position3D pPos3D = AL::Math::Position3D(0.1f, 0.2f, 0.3f);
  pRot = AL::Math::Rotation::fromRotX(0.5f);
  pHOut = AL::Math::Transform::fromRotX(0.5f);
  pHOut.r1_c4 = 0.1f;
  pHOut.r2_c4 = 0.2f;
  pHOut.r3_c4 = 0.3f;
  pHIn = AL::Math::transformFromPosition3DAndRotation(pPos3D, pRot);
  compareTransform(pHIn, pHOut, 0.0001f);


  pPos3D = AL::Math::Position3D(0.1f, 0.2f, 0.3f);
  pRot = AL::Math::Rotation::fromRotY(0.5f);
  pHOut = AL::Math::Transform::fromRotY(0.5f);
  pHOut.r1_c4 = 0.1f;
  pHOut.r2_c4 = 0.2f;
  pHOut.r3_c4 = 0.3f;
  pHIn = AL::Math::transformFromPosition3DAndRotation(pPos3D, pRot);
  compareTransform(pHIn, pHOut, 0.0001f);


  pPos3D = AL::Math::Position3D(0.1f, 0.2f, 0.3f);
  pRot = AL::Math::Rotation::fromRotZ(0.5f);
  pHOut = AL::Math::Transform::fromRotZ(0.5f);
  pHOut.r1_c4 = 0.1f;
  pHOut.r2_c4 = 0.2f;
  pHOut.r3_c4 = 0.3f;
  pHIn = AL::Math::transformFromPosition3DAndRotation(pPos3D, pRot);
  compareTransform(pHIn, pHOut, 0.0001f);
}

TEST(ALMathTest, Position6DFromVelocity6D)
{
  AL::Math::Velocity6D pVIn    = AL::Math::Velocity6D(0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f);
  AL::Math::Position6D pPosIn  = AL::Math::position6DFromVelocity6D(pVIn);

  AL::Math::Position6D pPosOut = AL::Math::Position6D(0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f);

  comparePosition6D(pPosIn, pPosOut, 0.0001f);
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
  comparePosition3D(pPosIn, pPosOut, 0.0001f);

  pRot = AL::Math::Rotation();
  pPos3D = AL::Math::Position3D(1.0f, -1.0f, 0.5f);
  pPosIn = pRot*pPos3D;
  pPosOut = AL::Math::Position3D(1.0f, -1.0f, 0.5f);
  comparePosition3D(pPosIn, pPosOut, 0.0001f);

  pRot = AL::Math::Rotation::fromRotX(0.5f);
  pPos3D = AL::Math::Position3D(1.0f, -1.0f, 0.5f);
  pPosIn = pRot*pPos3D;
  pPosOut = AL::Math::Position3D(1.00000000000000f, -1.11729533119247f, -0.04063425765902f);
  comparePosition3D(pPosIn, pPosOut, 0.0001f);

  //inline Velocity6D operator*(
  //  const float       pK,
  //  const Position6D& pDelta)
  float pK = 10.0f;
  AL::Math::Position6D pPos6D = AL::Math::Position6D(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f);
  AL::Math::Velocity6D pVel6DIn = pK*pPos6D;
  AL::Math::Velocity6D pVel6DOut = AL::Math::Velocity6D(10.0f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f);
  compareVelocity6D(pVel6DIn, pVel6DOut, 0.0001f);
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
}

TEST(ALMathTest, ApplyRotation)
{
}

TEST(ALMathTest, smoothTrapezoid)
{
}

