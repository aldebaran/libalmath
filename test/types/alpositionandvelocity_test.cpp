/**
 * @author Cyrille Collette - ccollette@aldebaran-robotics.com
 * Aldebaran Robotics (c) 2009 All Rights Reserved
 *
 */
#include <almath/types/alpositionandvelocity.h>

#include <gtest/gtest.h>

TEST(ALPositionAndVelocityTest, isNear)
{
  AL::Math::PositionAndVelocity pTmp = AL::Math::PositionAndVelocity(-1.0f, -0.5f);

  EXPECT_TRUE(pTmp.isNear(AL::Math::PositionAndVelocity(-1.0f, -0.5f), 0.001f));
  EXPECT_FALSE(pTmp.isNear(AL::Math::PositionAndVelocity(-0.9f, -0.5f), 0.001f));
  EXPECT_FALSE(pTmp.isNear(AL::Math::PositionAndVelocity(-1.0f, -0.4f), 0.001f));
}

