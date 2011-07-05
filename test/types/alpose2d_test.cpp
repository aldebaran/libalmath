/**
 * @author Cyrille Collette - ccollette@aldebaran-robotics.com
 * Aldebaran Robotics (c) 2009 All Rights Reserved
 *
 */
#include <almath/types/alpose2d.h>
#include "../almathtestutils.h"

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
  comparePose2D(pPos2D, AL::Math::Pose2D(1.0f, -0.6f, 0.2f), 0.001f);

  // operator + (+a)
  pPos2D1 = AL::Math::Pose2D(0.5f, -0.3f, 0.1f);
  pPos2D  = AL::Math::Pose2D();
  pPos2D = +pPos2D1;
  comparePose2D(pPos2D, AL::Math::Pose2D(0.5f, -0.3f, 0.1f), 0.001f);

  // operator +=
  pPos2D1 = AL::Math::Pose2D(0.5f, -0.3f, 0.1f);
  pPos2D2 = AL::Math::Pose2D(0.5f, -0.3f, 0.1f);
  pPos2D1 += pPos2D2;
  comparePose2D(pPos2D1, AL::Math::Pose2D(1.0f, -0.6f, 0.2f), 0.001f);

  // operator - (a-b)
  pPos2D1 = AL::Math::Pose2D(0.5f, +0.3f, 0.1f);
  pPos2D2 = AL::Math::Pose2D(0.5f, -0.3f, 0.1f);
  pPos2D = pPos2D2 - pPos2D1;
  comparePose2D(pPos2D, AL::Math::Pose2D(0.0f, -0.6f, 0.0f), 0.001f);

  // operator - (-a)
  pPos2D1 = AL::Math::Pose2D(0.5f, -0.3f, 0.1f);
  pPos2D  = AL::Math::Pose2D();
  pPos2D  = - pPos2D1;
  comparePose2D(pPos2D, AL::Math::Pose2D(-0.5f, 0.3f, -0.1f), 0.001f);

  // operator -=
  pPos2D1 = AL::Math::Pose2D(0.6f, -0.4f, 0.2f);
  pPos2D2 = AL::Math::Pose2D(0.5f, -0.3f, 0.1f);
  pPos2D1 -= pPos2D2;
  comparePose2D(pPos2D1, AL::Math::Pose2D(0.1f, -0.1f, 0.1f), 0.001f);

  // operator *
  pPos2D1 = AL::Math::Pose2D(1.0f, 0.0f, 0.0f);
  pPos2D2 = AL::Math::Pose2D(0.0f, 1.0f, 0.2f);
  pPos2D = pPos2D1 * pPos2D2;
  comparePose2D(pPos2D, AL::Math::Pose2D(1.0f, 1.0f, 0.2f), 0.001f);

//  // operator *
//  pPos2D1 = AL::Math::Pose2D(-0.13027f, 0.348845f, 3.14151f);
//  pPos2D2 = AL::Math::Pose2D(0.1f, 0.0f, 0.0f);
//  pPos2D = pPos2D1*pPos2D2;
//  comparePose2D(pPos2D, AL::Math::Pose2D(-0.212624f, 0.349525f, 3.14155f), 0.001f);

  // operator *=
  pPos2D1 = AL::Math::Pose2D(1.0f, 0.0f, 0.0f);
  pPos2D2 = AL::Math::Pose2D(0.0f, 1.0f, 0.2f);
  pPos2D1 *= pPos2D2;
  comparePose2D(pPos2D1, AL::Math::Pose2D(1.0f, 1.0f, 0.2f), 0.001f);
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

