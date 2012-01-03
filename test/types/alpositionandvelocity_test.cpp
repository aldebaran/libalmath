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

