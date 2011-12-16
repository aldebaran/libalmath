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
#include <almath/types/alpose2d.h>

#include <gtest/gtest.h>

TEST(ALPose2DTest, basicOperator)
{
  AL::Math::Pose2D pPos2D  = AL::Math::Pose2D();
  AL::Math::Pose2D pPos2D1 = AL::Math::Pose2D(0.5f, -0.3f, 0.1f);
  AL::Math::Pose2D pPos2D2 = AL::Math::Pose2D(0.5f, -0.3f, 0.1f);

  // operator !=
  EXPECT_FALSE(pPos2D1 != pPos2D2);
  EXPECT_TRUE(pPos2D1 != AL::Math::Pose2D());

  // operator ==
  EXPECT_TRUE(pPos2D1 == pPos2D2);
  EXPECT_FALSE(pPos2D1 == AL::Math::Pose2D());

  // operator + (a+b)
  pPos2D1 = AL::Math::Pose2D(0.5f, -0.3f, 0.1f);
  pPos2D2 = AL::Math::Pose2D(0.5f, -0.3f, 0.1f);
  pPos2D = pPos2D1 + pPos2D2;
  EXPECT_TRUE(pPos2D.isNear(AL::Math::Pose2D(1.0f, -0.6f, 0.2f), 0.001f));

  // operator + (+a)
  pPos2D1 = AL::Math::Pose2D(0.5f, -0.3f, 0.1f);
  pPos2D  = AL::Math::Pose2D();
  pPos2D = +pPos2D1;
  EXPECT_TRUE(pPos2D.isNear(AL::Math::Pose2D(0.5f, -0.3f, 0.1f), 0.001f));

  // operator +=
  pPos2D1 = AL::Math::Pose2D(0.5f, -0.3f, 0.1f);
  pPos2D2 = AL::Math::Pose2D(0.5f, -0.3f, 0.1f);
  pPos2D1 += pPos2D2;
  EXPECT_TRUE(pPos2D1.isNear(AL::Math::Pose2D(1.0f, -0.6f, 0.2f), 0.001f));

  // operator - (a-b)
  pPos2D1 = AL::Math::Pose2D(0.5f, +0.3f, 0.1f);
  pPos2D2 = AL::Math::Pose2D(0.5f, -0.3f, 0.1f);
  pPos2D = pPos2D2 - pPos2D1;
  EXPECT_TRUE(pPos2D.isNear(AL::Math::Pose2D(0.0f, -0.6f, 0.0f), 0.001f));

  // operator - (-a)
  pPos2D1 = AL::Math::Pose2D(0.5f, -0.3f, 0.1f);
  pPos2D  = AL::Math::Pose2D();
  pPos2D  = - pPos2D1;
  EXPECT_TRUE(pPos2D.isNear(AL::Math::Pose2D(-0.5f, 0.3f, -0.1f), 0.001f));

  // operator -=
  pPos2D1 = AL::Math::Pose2D(0.6f, -0.4f, 0.2f);
  pPos2D2 = AL::Math::Pose2D(0.5f, -0.3f, 0.1f);
  pPos2D1 -= pPos2D2;
  EXPECT_TRUE(pPos2D1.isNear(AL::Math::Pose2D(0.1f, -0.1f, 0.1f), 0.001f));

  // operator *
  pPos2D1 = AL::Math::Pose2D(1.0f, 0.0f, 0.0f);
  pPos2D2 = AL::Math::Pose2D(0.0f, 1.0f, 0.2f);
  pPos2D = pPos2D1*pPos2D2;
  EXPECT_TRUE(pPos2D.isNear(AL::Math::Pose2D(1.0f, 1.0f, 0.2f), 0.001f));

//  // operator *
//  pPos2D1 = AL::Math::Pose2D(-0.13027f, 0.348845f, 3.14151f);
//  pPos2D2 = AL::Math::Pose2D(0.1f, 0.0f, 0.0f);
//  pPos2D = pPos2D1*pPos2D2;
//  EXPECT_TRUE(pPos2D.isNear(AL::Math::Pose2D(-0.212624f, 0.349525f, 3.14155f), 0.001f));

  // operator *=
  pPos2D1 = AL::Math::Pose2D(1.0f, 0.0f, 0.0f);
  pPos2D2 = AL::Math::Pose2D(0.0f, 1.0f, 0.2f);
  pPos2D1 *= pPos2D2;
  EXPECT_TRUE(pPos2D1.isNear(AL::Math::Pose2D(1.0f, 1.0f, 0.2f), 0.001f));
}


TEST(ALPose2DTest, isNear)
{
  AL::Math::Pose2D pPos2D = AL::Math::Pose2D(0.5f, -0.3f, 0.1f);

  EXPECT_TRUE(pPos2D.isNear(AL::Math::Pose2D(0.5f, -0.3f, 0.1f), 0.0001f));
  EXPECT_FALSE(pPos2D.isNear(AL::Math::Pose2D(0.4f, -0.3f, 0.1f), 0.0001f));
  EXPECT_FALSE(pPos2D.isNear(AL::Math::Pose2D(0.5f, -0.4f, 0.1f), 0.0001f));
  EXPECT_FALSE(pPos2D.isNear(AL::Math::Pose2D(0.5f, -0.3f, 0.2f), 0.0001f));
}

TEST(ALPose2DTest, distance)
{
  AL::Math::Pose2D pPos2D = AL::Math::Pose2D(1.0f, 2.0f, 3.0f);
  EXPECT_NEAR(pPos2D.distance(AL::Math::Pose2D(1.0f, 1.0f, 1.0f)), 1.0f, 0.0001f);
}

TEST(ALPose2DTest, distanceSquared)
{
  AL::Math::Pose2D pPos2D = AL::Math::Pose2D(1.0f, 2.0f, 3.0f);
  EXPECT_NEAR(pPos2D.distanceSquared(AL::Math::Pose2D(0.0f, 0.0f, 0.0f)), 5.0f, 0.0001f);
}

TEST(ALPose2DTest, inverse)
{
  AL::Math::Pose2D pPose2D = AL::Math::Pose2D(0.1f, -0.1f, -0.5f);
  AL::Math::Pose2D pResult = AL::Math::Pose2D(-0.135701f, 0.0398157f, 0.5f);
  AL::Math::Pose2D pInverse = pPose2D.inverse();
  EXPECT_NEAR(pInverse.x, pResult.x, 0.0001f);
  EXPECT_NEAR(pInverse.y, pResult.y, 0.0001f);
  EXPECT_NEAR(pInverse.theta, pResult.theta, 0.0001f);

  pInverse = AL::Math::Pose2D();
  pInverse = AL::Math::pose2DInverse(pPose2D);
  EXPECT_NEAR(pInverse.x, pResult.x, 0.0001f);
  EXPECT_NEAR(pInverse.y, pResult.y, 0.0001f);
  EXPECT_NEAR(pInverse.theta, pResult.theta, 0.0001f);

  pInverse = AL::Math::Pose2D();
  AL::Math::pose2DInverse(pPose2D, pInverse);
  EXPECT_NEAR(pInverse.x, pResult.x, 0.0001f);
  EXPECT_NEAR(pInverse.y, pResult.y, 0.0001f);
  EXPECT_NEAR(pInverse.theta, pResult.theta, 0.0001f);
}

