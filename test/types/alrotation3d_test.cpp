/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */
#include <almath/types/alrotation3d.h>

#include <gtest/gtest.h>

TEST(ALRotation3DTest, norm)
{
  float kEpsilon = 0.0001f;
  AL::Math::Rotation3D pRot = AL::Math::Rotation3D(0.0f, 0.0f, 0.0f);
  EXPECT_NEAR(AL::Math::norm(pRot), 0.0f, kEpsilon);
}

