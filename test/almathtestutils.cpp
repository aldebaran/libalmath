/**
 * @author Cyrille Collette - ccollette@aldebaran-robotics.com
 * Aldebaran Robotics (c) 2010 All Rights Reserved
 *
 */

#include "almathtestutils.h"


void validateFindRotation(
    const AL::Math::Position3D& pA,
    const AL::Math::Position3D& pB)
{
  float eps = 0.001f;

  AL::Math::Position3D pSol;
  AL::Math::Transform HOut;

  AL::Math::findRotation(pA, pB, HOut);

  EXPECT_TRUE(HOut.isTransform(eps));

  pSol = HOut*pA;
  comparePosition3D(pSol, pB, eps);

  pSol = HOut.inverse()*pB;
  comparePosition3D(pSol, pA, eps);
}


void comparePose2D(
             const AL::Math::Pose2D& pPos1,
             const AL::Math::Pose2D& pPos2,
             const float pEps)
{
  EXPECT_NEAR(pPos1.x, pPos2.x, pEps);
  EXPECT_NEAR(pPos1.y, pPos2.y, pEps);
  EXPECT_NEAR(pPos1.theta, pPos2.theta, pEps);
}


void compareTransform(
             const AL::Math::Transform& pH1,
             const AL::Math::Transform& pH2,
             const float pEps)
{
  EXPECT_NEAR(pH1.r1_c1, pH2.r1_c1, pEps);
  EXPECT_NEAR(pH1.r1_c2, pH2.r1_c2, pEps);
  EXPECT_NEAR(pH1.r1_c3, pH2.r1_c3, pEps);

  EXPECT_NEAR(pH1.r2_c1, pH2.r2_c1, pEps);
  EXPECT_NEAR(pH1.r2_c2, pH2.r2_c2, pEps);
  EXPECT_NEAR(pH1.r2_c3, pH2.r2_c3, pEps);

  EXPECT_NEAR(pH1.r3_c1, pH2.r3_c1, pEps);
  EXPECT_NEAR(pH1.r3_c2, pH2.r3_c2, pEps);
  EXPECT_NEAR(pH1.r3_c3, pH2.r3_c3, pEps);

  EXPECT_NEAR(pH1.r1_c4, pH2.r1_c4, pEps);
  EXPECT_NEAR(pH1.r2_c4, pH2.r2_c4, pEps);
  EXPECT_NEAR(pH1.r3_c4, pH2.r3_c4, pEps);
}


void comparePosition2D(
  const AL::Math::Position2D& pPos1,
  const AL::Math::Position2D& pPos2,
  const float pEps)
{
  EXPECT_NEAR(pPos1.x, pPos2.x, pEps);
  EXPECT_NEAR(pPos1.y, pPos2.y, pEps);
}


void comparePosition3D(
  const AL::Math::Position3D& pPos1,
  const AL::Math::Position3D& pPos2,
  const float pEps)
{
  EXPECT_NEAR(pPos1.x, pPos2.x, pEps);
  EXPECT_NEAR(pPos1.y, pPos2.y, pEps);
  EXPECT_NEAR(pPos1.z, pPos2.z, pEps);
}


void comparePosition6D(
  const AL::Math::Position6D& pPos1,
  const AL::Math::Position6D& pPos2,
  const float pEps)
{
  EXPECT_NEAR(pPos1.x, pPos2.x, pEps);
  EXPECT_NEAR(pPos1.y, pPos2.y, pEps);
  EXPECT_NEAR(pPos1.z, pPos2.z, pEps);
  EXPECT_NEAR(pPos1.wx, pPos2.wx, pEps);
  EXPECT_NEAR(pPos1.wy, pPos2.wy, pEps);
  EXPECT_NEAR(pPos1.wz, pPos2.wz, pEps);
}


void compareVelocity3D(
  const AL::Math::Velocity3D& pVel1,
  const AL::Math::Velocity3D& pVel2,
  const float pEps)
{
  EXPECT_NEAR(pVel1.xd, pVel2.xd, pEps);
  EXPECT_NEAR(pVel1.yd, pVel2.yd, pEps);
  EXPECT_NEAR(pVel1.zd, pVel2.zd, pEps);
}


void compareVelocity6D(
  const AL::Math::Velocity6D& pVel1,
  const AL::Math::Velocity6D& pVel2,
  const float pEps)
{
  EXPECT_NEAR(pVel1.xd, pVel2.xd, pEps);
  EXPECT_NEAR(pVel1.yd, pVel2.yd, pEps);
  EXPECT_NEAR(pVel1.zd, pVel2.zd, pEps);
  EXPECT_NEAR(pVel1.wxd, pVel2.wxd, pEps);
  EXPECT_NEAR(pVel1.wyd, pVel2.wyd, pEps);
  EXPECT_NEAR(pVel1.wzd, pVel2.wzd, pEps);
}


void compareRotation(
  const AL::Math::Rotation& pRot1,
  const AL::Math::Rotation& pRot2,
  const float pEps)
{
  EXPECT_NEAR(pRot1.r1_c1, pRot2.r1_c1, pEps);
  EXPECT_NEAR(pRot1.r1_c2, pRot2.r1_c2, pEps);
  EXPECT_NEAR(pRot1.r1_c3, pRot2.r1_c3, pEps);
  EXPECT_NEAR(pRot1.r2_c1, pRot2.r2_c1, pEps);
  EXPECT_NEAR(pRot1.r2_c2, pRot2.r2_c2, pEps);
  EXPECT_NEAR(pRot1.r2_c3, pRot2.r2_c3, pEps);
  EXPECT_NEAR(pRot1.r3_c1, pRot2.r3_c1, pEps);
  EXPECT_NEAR(pRot1.r3_c2, pRot2.r3_c2, pEps);
  EXPECT_NEAR(pRot1.r3_c3, pRot2.r3_c3, pEps);
}


void compareRotation3D(
  const AL::Math::Rotation3D& pRot1,
  const AL::Math::Rotation3D& pRot2,
  const float pEps)
{
  EXPECT_NEAR(pRot1.wx, pRot2.wx, pEps);
  EXPECT_NEAR(pRot1.wy, pRot2.wy, pEps);
  EXPECT_NEAR(pRot1.wz, pRot2.wz, pEps);
}

void compareComplex(
  const AL::Math::Complex& pIn1,
  const AL::Math::Complex& pIn2,
  const float pEps)
{
  EXPECT_NEAR(pIn1.re, pIn2.re, pEps);
  EXPECT_NEAR(pIn1.im, pIn2.im, pEps);
}


void comparePositionAndVelocity(
  const AL::Math::PositionAndVelocity& pPosVel1,
  const AL::Math::PositionAndVelocity& pPosVel2,
  const float pEps)
{
  EXPECT_NEAR(pPosVel1.q, pPosVel2.q, pEps);
  EXPECT_NEAR(pPosVel1.dq, pPosVel2.dq, pEps);
}

void compareTransformAndVelocity6D(
  const AL::Math::TransformAndVelocity6D& pDat1,
  const AL::Math::TransformAndVelocity6D& pDat2,
  const float pEps)
{
  compareTransform(pDat1.T, pDat2.T, pEps);
  compareVelocity6D(pDat1.V, pDat2.V, pEps);
}

