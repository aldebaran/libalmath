/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
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

TEST(ALPositionAndVelocityTest, toVector)
{
  const float eps = 1e-4f;
  const AL::Math::PositionAndVelocity pos(1.0f, 2.0f);
  const std::vector<float> vec = pos.toVector();

  EXPECT_TRUE(vec.size()==2u);
  EXPECT_NEAR(pos.q, vec[0], eps);
  EXPECT_NEAR(pos.dq, vec[1], eps);
}
