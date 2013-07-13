/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */
#include <almath/types/alvelocity6d.h>

#include <gtest/gtest.h>
#include <stdexcept>

AL::Math::Velocity6D pVel6D1 = AL::Math::Velocity6D();
AL::Math::Velocity6D pVel6D2 = AL::Math::Velocity6D();

TEST(ALVelocity6DTest, norm)
{
  float kEpsilon = 0.0001f;

  //std::cout << "-------------- norm 0 --------------" << std::endl;
  pVel6D1 = AL::Math::Velocity6D();
  EXPECT_NEAR(AL::Math::norm(pVel6D1), 0.0f, kEpsilon);

  //std::cout << "-------------- norm 1 --------------" << std::endl;
  pVel6D1 = AL::Math::Velocity6D(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  EXPECT_NEAR(AL::Math::norm(pVel6D1), 1.0f, kEpsilon);


  ////std::cout << "-------------- norm 2 --------------" << std::endl;
  //pVel6D1 = AL::Math::Velocity6D(1.0f, -0.8f, 0.2f, 0.4f, 0.3f, -0.3f);
  //EXPECT_NEAR(AL::Math::norm(pVel6D1), 0.0f, kEpsilon); // TODO
}


TEST(ALVelocity6DTest, normalize)
{
  //std::cout << "-------------- normalize 0 --------------" << std::endl;
  pVel6D1 = AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  ASSERT_THROW(AL::Math::normalize(pVel6D1), std::runtime_error);


  //std::cout << "-------------- normalize 1 --------------" << std::endl;
  pVel6D1 = AL::Math::Velocity6D(0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  EXPECT_TRUE(AL::Math::normalize(pVel6D1).isNear(AL::Math::Velocity6D(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f)));


  //std::cout << "-------------- normalize 2 --------------" << std::endl;
  pVel6D1 = AL::Math::Velocity6D(1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  EXPECT_TRUE(AL::Math::normalize(pVel6D1).isNear(AL::Math::Velocity6D(0.70710678118655f, -0.70710678118655f, 0.0f, 0.0f, 0.0f, 0.0f)));
}


TEST(ALVelocity6DTest, Divers)
{
  //std::cout << "-------------- soustraction 0 (a = b-c) --------------" << std::endl;
  pVel6D1 = AL::Math::Velocity6D(+1.2f, 1.3f, 0.2f, +0.3f, 10.0f, -10.0f);
  pVel6D2 = AL::Math::Velocity6D(-0.5f, 0.2f, 0.4f, -0.5f,  0.2f,  +0.4f);
  EXPECT_TRUE((pVel6D2-pVel6D1).isNear(AL::Math::Velocity6D(-1.7f, -1.1f, 0.2f, -0.8f, -9.8f, +10.4f)));

  //std::cout << "-------------- soustraction 0 (a = -b) --------------" << std::endl;
  pVel6D1 = AL::Math::Velocity6D(+1.2f, 1.3f, 0.2f, +0.3f, 10.0f, -10.0f);
  pVel6D2 = AL::Math::Velocity6D();
  pVel6D2 = -pVel6D1;
  EXPECT_TRUE(pVel6D2.isNear(AL::Math::Velocity6D(-1.2f, -1.3f, -0.2f, -0.3f, -10.0f, +10.0f)));

  //std::cout << "-------------- addition 0 (a = b+c) --------------" << std::endl;
  pVel6D1 = AL::Math::Velocity6D(+1.2f, 1.3f, 0.3f, +1.2f, 1.3f, 0.3f);
  pVel6D2 = AL::Math::Velocity6D(-0.5f, 0.2f, 0.2f, -0.5f, 0.2f, 0.2f);
  EXPECT_TRUE((pVel6D1+pVel6D2).isNear(AL::Math::Velocity6D(0.7f, 1.5f, 0.5f, 0.7f, 1.5f, 0.5f)));

  //std::cout << "-------------- addition 0 (a = +b) --------------" << std::endl;
  pVel6D1 = AL::Math::Velocity6D(+1.2f, 1.3f, 0.3f, -1.2f, 1.3f, 0.3f);
  pVel6D2 = AL::Math::Velocity6D();
  pVel6D2 = +pVel6D1;
  EXPECT_TRUE(pVel6D2.isNear(AL::Math::Velocity6D(+1.2f, 1.3f, 0.3f, -1.2f, 1.3f, 0.3f)));

  //std::cout << "-------------- multiplication 0 --------------" << std::endl;
  pVel6D1 = AL::Math::Velocity6D(1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f);
  EXPECT_TRUE((pVel6D1*2.0f).isNear(AL::Math::Velocity6D(2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f)));


  //std::cout << "-------------- multiplication 1 --------------" << std::endl;
  pVel6D1 = AL::Math::Velocity6D(1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f);
  EXPECT_TRUE((2.0f*pVel6D1).isNear(AL::Math::Velocity6D(2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f)));


  //std::cout << "-------------- division 0 --------------" << std::endl;
  pVel6D1 = AL::Math::Velocity6D(2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f);
  EXPECT_TRUE((pVel6D1/2.0f).isNear(AL::Math::Velocity6D(1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f)));


  //std::cout << "-------------- division 1 --------------" << std::endl;
  pVel6D1 = AL::Math::Velocity6D(2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f);
  ASSERT_THROW((pVel6D1/0.0f), std::runtime_error);
}

TEST(ALVelocity6DTest, toVector)
{
  const float eps = 1e-4f;
  const AL::Math::Velocity6D vel(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f);
  const std::vector<float> vec = vel.toVector();

  EXPECT_TRUE(vec.size()==6);
  EXPECT_NEAR(vel.xd, vec.at(0), eps);
  EXPECT_NEAR(vel.yd, vec.at(1), eps);
  EXPECT_NEAR(vel.zd, vec.at(2), eps);
  EXPECT_NEAR(vel.wxd, vec.at(3), eps);
  EXPECT_NEAR(vel.wyd, vec.at(4), eps);
  EXPECT_NEAR(vel.wzd, vec.at(5), eps);
}
