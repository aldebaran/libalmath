/**
* @author Cyrille Collette - ccollette@aldebaran-robotics.com
* Aldebaran Robotics (c) 2009 All Rights Reserved
*
*/
#include <almath/types/altransformandvelocity6d.h>

#include <gtest/gtest.h>

TEST(ALTransformAndVelocityTest, isNear)
{
  AL::Math::TransformAndVelocity6D pTmp;
  EXPECT_TRUE(pTmp.isNear(AL::Math::TransformAndVelocity6D(), 0.0001f));

  pTmp = AL::Math::TransformAndVelocity6D();
  pTmp.T.r1_c4 = 0.2f;
  EXPECT_FALSE(pTmp.isNear(AL::Math::TransformAndVelocity6D(), 0.0001f));

  pTmp = AL::Math::TransformAndVelocity6D();
  pTmp.T.r2_c4 = 0.2f;
  EXPECT_FALSE(pTmp.isNear(AL::Math::TransformAndVelocity6D(), 0.0001f));

  pTmp = AL::Math::TransformAndVelocity6D();
  pTmp.T.r3_c4 = 0.2f;
  EXPECT_FALSE(pTmp.isNear(AL::Math::TransformAndVelocity6D(), 0.0001f));

  pTmp = AL::Math::TransformAndVelocity6D();
  pTmp.V.xd = 0.2f;
  EXPECT_FALSE(pTmp.isNear(AL::Math::TransformAndVelocity6D(), 0.0001f));

  pTmp = AL::Math::TransformAndVelocity6D();
  pTmp.V.yd = 0.2f;
  EXPECT_FALSE(pTmp.isNear(AL::Math::TransformAndVelocity6D(), 0.0001f));

  pTmp = AL::Math::TransformAndVelocity6D();
  pTmp.V.zd = 0.2f;
  EXPECT_FALSE(pTmp.isNear(AL::Math::TransformAndVelocity6D(), 0.0001f));

  pTmp = AL::Math::TransformAndVelocity6D();
  pTmp.V.wxd = 0.2f;
  EXPECT_FALSE(pTmp.isNear(AL::Math::TransformAndVelocity6D(), 0.0001f));

  pTmp = AL::Math::TransformAndVelocity6D();
  pTmp.V.wyd = 0.2f;
  EXPECT_FALSE(pTmp.isNear(AL::Math::TransformAndVelocity6D(), 0.0001f));

  pTmp = AL::Math::TransformAndVelocity6D();
  pTmp.V.wzd = 0.2f;
  EXPECT_FALSE(pTmp.isNear(AL::Math::TransformAndVelocity6D(), 0.0001f));
}

