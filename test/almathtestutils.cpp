/**
 * @author Cyrille Collette - ccollette@aldebaran-robotics.com
 * Aldebaran Robotics (c) 2010 All Rights Reserved
 *
 */

#include <almath/tools/altransformhelpers.h>

#include <gtest/gtest.h>

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
  EXPECT_TRUE(pSol.isNear(pB, eps));

  pSol = HOut.inverse()*pB;
  EXPECT_TRUE(pSol.isNear(pA, eps));
}

