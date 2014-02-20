/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */
#include <almath/types/alposition6d.h>

#include <gtest/gtest.h>

#include <stdexcept>

AL::Math::Position6D pPos6D1 = AL::Math::Position6D();
AL::Math::Position6D pPos6D2 = AL::Math::Position6D();

TEST(ALPosition6DTest, Distance)
{
  float kEpsilon = 0.0001f;

  //std::cout << "-------------- Distance 0 --------------" << std::endl;
  pPos6D1 = AL::Math::Position6D();
  pPos6D2 = AL::Math::Position6D();
  EXPECT_NEAR(AL::Math::distance(pPos6D1, pPos6D2), 0.0f, kEpsilon);


  //std::cout << "-------------- Distance 1 --------------" << std::endl;
  pPos6D1 = AL::Math::Position6D();
  pPos6D2 = AL::Math::Position6D(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  EXPECT_NEAR(AL::Math::distance(pPos6D1, pPos6D2), 1.0f, kEpsilon);


  ////std::cout << "-------------- Distance 2 --------------" << std::endl;
  //pPos6D1 = AL::Math::Position6D(0.5f, -0.8f, 1.0f, +0.4f, -0.7f, -0.6f);
  //pPos6D2 = AL::Math::Position6D(1.0f, 1.2f, 1.2f, 1.0f, 1.2f, 1.2f);
  //EXPECT_NEAR(AL::Math::distance(pPos6D1, pPos6D2), 0.0f, kEpsilon); // TODO
}


TEST(ALPosition6DTest, norm)
{
  float kEpsilon = 0.0001f;

  //std::cout << "-------------- norm 0 --------------" << std::endl;
  pPos6D1 = AL::Math::Position6D();
  EXPECT_NEAR(AL::Math::norm(pPos6D1), 0.0f, kEpsilon);


  //std::cout << "-------------- norm 1 --------------" << std::endl;
  pPos6D1 = AL::Math::Position6D(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  EXPECT_NEAR(AL::Math::norm(pPos6D1), 1.0f, kEpsilon);


  ////std::cout << "-------------- norm 2 --------------" << std::endl;
  //pPos6D1 = AL::Math::Position6D(1.0f, -0.8f, 0.2f, 0.4f, 0.3f, -0.3f);
  //EXPECT_NEAR(AL::Math::norm(pPos6D1), 0.0f, kEpsilon); // TODO
}


TEST(ALPosition6DTest, normalize)
{
  //std::cout << "-------------- normalize 0 --------------" << std::endl;
  pPos6D1 = AL::Math::Position6D(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  ASSERT_THROW(AL::Math::normalize(pPos6D1), std::runtime_error);


  //std::cout << "-------------- normalize 1 --------------" << std::endl;
  pPos6D1 = AL::Math::Position6D(0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  EXPECT_TRUE(AL::Math::normalize(pPos6D1).isNear(AL::Math::Position6D(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f)));


  //std::cout << "-------------- normalize 2 --------------" << std::endl;
  pPos6D1 = AL::Math::Position6D(1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  EXPECT_TRUE(AL::Math::normalize(pPos6D1).isNear(AL::Math::Position6D(0.70710678118655f, -0.70710678118655f, 0.0f, 0.0f, 0.0f, 0.0f)));
}


TEST(ALPosition6DTest, Divers)
{
  //std::cout << "-------------- soustraction 0 (a-b) --------------" << std::endl;
  pPos6D1 = AL::Math::Position6D(+1.2f, 1.3f, 0.2f, +0.3f, 10.0f, -10.0f);
  pPos6D2 = AL::Math::Position6D(-0.5f, 0.2f, 0.4f, -0.5f,  0.2f,  +0.4f);
  EXPECT_TRUE((pPos6D2-pPos6D1).isNear(AL::Math::Position6D(-1.7f, -1.1f, 0.2f, -0.8f, -9.8f, +10.4f)));


  //std::cout << "-------------- soustraction 0 (a = -b) --------------" << std::endl;
  pPos6D1 = AL::Math::Position6D(+1.2f, 1.3f, 0.2f, +0.3f, 10.0f, -10.0f);
  pPos6D2 = AL::Math::Position6D();
  pPos6D2 = -pPos6D1;
  EXPECT_TRUE(pPos6D2.isNear(AL::Math::Position6D(-1.2f, -1.3f, -0.2f, -0.3f, -10.0f, +10.0f)));


  //std::cout << "-------------- soustraction 0 (a -= b) --------------" << std::endl;
  pPos6D1 = AL::Math::Position6D(+1.2f, 1.3f, 0.2f, +0.3f, 10.0f, -10.0f);
  pPos6D2 = AL::Math::Position6D(1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f);
  pPos6D2 -= pPos6D1;
  EXPECT_TRUE(pPos6D2.isNear(AL::Math::Position6D(-0.2f, -0.3f, 0.8f, 0.7f, -9.0f, +11.0f)));


  //std::cout << "-------------- addition 0 (a+b)--------------" << std::endl;
  pPos6D1 = AL::Math::Position6D(+1.2f, 1.3f, 0.3f, +1.2f, 1.3f, 0.3f);
  pPos6D2 = AL::Math::Position6D(-0.5f, 0.2f, 0.2f, -0.5f, 0.2f, 0.2f);
  EXPECT_TRUE((pPos6D1+pPos6D2).isNear(AL::Math::Position6D(0.7f, 1.5f, 0.5f, 0.7f, 1.5f, 0.5f)));


  //std::cout << "-------------- addition 0 (a = +b)--------------" << std::endl;
  pPos6D1 = AL::Math::Position6D(+1.2f, 1.3f, 0.3f, +1.2f, 1.3f, 0.3f);
  pPos6D2 = AL::Math::Position6D();
  pPos6D2 = +pPos6D1;
  EXPECT_TRUE(pPos6D2.isNear(AL::Math::Position6D(1.2f, 1.3f, 0.3f, 1.2f, 1.3f, 0.3f)));


  //std::cout << "-------------- addition 0 (a += b)--------------" << std::endl;
  pPos6D1 = AL::Math::Position6D(+1.2f, 1.3f, 0.3f, +1.2f, 1.3f, 0.3f);
  pPos6D2 = AL::Math::Position6D(+1.2f, 1.3f, 0.3f, +1.2f, 1.3f, 0.3f);
  pPos6D2 += pPos6D1;
  EXPECT_TRUE(pPos6D2.isNear(AL::Math::Position6D(2.4f, 2.6f, 0.6f, 2.4f, 2.6f, 0.6f)));

  //std::cout << "-------------- multiplication 0 --------------" << std::endl;
  pPos6D1 = AL::Math::Position6D(1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f);
  EXPECT_TRUE((pPos6D1*2.0f).isNear(AL::Math::Position6D(2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f)));


  ////std::cout << "-------------- multiplication 1 --------------" << std::endl;
  //pPos6D1 = AL::Math::Position6D(1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f);
  //EXPECT_TRUE((2.0f*pPos6D1).isNear(AL::Math::Position6D(2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f)));


  //std::cout << "-------------- division 0 --------------" << std::endl;
  pPos6D1 = AL::Math::Position6D(2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f);
  EXPECT_TRUE((pPos6D1/2.0f).isNear(AL::Math::Position6D(1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f)));


  //std::cout << "-------------- division 1 --------------" << std::endl;
  pPos6D1 = AL::Math::Position6D(2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f);
  ASSERT_THROW((pPos6D1/0.0f), std::runtime_error);
}

TEST(ALPosition6DTest, toVector)
{
  const float eps = 1e-4f;
  const AL::Math::Position6D pos(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f);
  const std::vector<float> vec = pos.toVector();

  EXPECT_TRUE(vec.size()==6u);
  EXPECT_NEAR(pos.x, vec[0], eps);
  EXPECT_NEAR(pos.y, vec[1], eps);
  EXPECT_NEAR(pos.z, vec[2], eps);
  EXPECT_NEAR(pos.wx, vec[3], eps);
  EXPECT_NEAR(pos.wy, vec[4], eps);
  EXPECT_NEAR(pos.wz, vec[5], eps);
}

