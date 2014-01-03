/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */
#include <almath/types/alposition3d.h>

#include <gtest/gtest.h>

#include <stdexcept>



TEST(ALPosition3DTest, ExplicitConstructor)
{
  AL::Math::Position3D pPos3D1 = AL::Math::Position3D(-10.3f);
  AL::Math::Position3D pPos3D2 = AL::Math::Position3D(-10.3f, -10.3f, -10.3f);
  EXPECT_TRUE(pPos3D1.isNear(pPos3D2));
}

TEST(ALPosition3DTest, Distance)
{
  float kEpsilon = 0.0001f;
  AL::Math::Position3D pPos3D1 = AL::Math::Position3D();
  AL::Math::Position3D pPos3D2 = AL::Math::Position3D();

  //std::cout << "-------------- Distance 0 --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D();
  pPos3D2 = AL::Math::Position3D();
  EXPECT_NEAR(AL::Math::distance(pPos3D1, pPos3D2), 0.0f, kEpsilon);


  //std::cout << "-------------- Distance 1 --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D();
  pPos3D2 = AL::Math::Position3D(1.0f, 0.0f, 0.0f);
  EXPECT_NEAR(AL::Math::distance(pPos3D1, pPos3D2), 1.0f, kEpsilon);


  //std::cout << "-------------- Distance 2 --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(0.5f, -0.8f, 1.0f);
  pPos3D2 = AL::Math::Position3D(1.0f, 1.2f, 1.2f);
  EXPECT_NEAR(AL::Math::distance(pPos3D1, pPos3D2), 2.07123151772080f, kEpsilon);
}


TEST(ALPosition3DTest, norm)
{
  float kEpsilon = 0.0001f;
  AL::Math::Position3D pPos3D1 = AL::Math::Position3D();

  //std::cout << "-------------- Norm 0 --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(0.0f, 0.0f, 0.0f);
  EXPECT_NEAR(AL::Math::norm(pPos3D1), 0.0f, kEpsilon);


  //std::cout << "-------------- Norm 1 --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(1.0f, 0.0f, 0.0f);
  EXPECT_NEAR(AL::Math::norm(pPos3D1), 1.0f, kEpsilon);


  //std::cout << "-------------- Norm 2 --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(1.0f, -0.8f, 0.2f);
  EXPECT_NEAR(AL::Math::norm(pPos3D1), 1.29614813968157f, kEpsilon);
}


TEST(ALPosition3DTest, normalize)
{
  AL::Math::Position3D pPos3D1 = AL::Math::Position3D();

  //std::cout << "-------------- normalize 0 --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(0.0f, 0.0f, 0.0f);
  ASSERT_THROW(AL::Math::normalize(pPos3D1), std::runtime_error);


  //std::cout << "-------------- normalize 1 --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(0.5f, 0.0f, 0.0f);
  EXPECT_TRUE(AL::Math::normalize(pPos3D1).isNear(AL::Math::Position3D(1.0f, 0.0f, 0.0f)));


  //std::cout << "-------------- normalize 2 --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(1.0f, -1.0f, 0.0f);
  EXPECT_TRUE(AL::Math::normalize(pPos3D1).isNear(AL::Math::Position3D(0.70710678118655f, -0.70710678118655f, 0.0f)));
}


TEST(ALPosition3DTest, crossProduct)
{
  AL::Math::Position3D pPos3D1 = AL::Math::Position3D();
  AL::Math::Position3D pPos3D2 = AL::Math::Position3D();

  //std::cout << "-------------- crossProduct 0 --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D();
  pPos3D2 = AL::Math::Position3D();
  EXPECT_TRUE(AL::Math::crossProduct(pPos3D1, pPos3D2).isNear(AL::Math::Position3D()));


  //std::cout << "-------------- crossProduct 1 --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(+1.0f, 2.0f, 1.0f);
  pPos3D2 = AL::Math::Position3D(-0.5f, 0.2f, 0.2f);
  EXPECT_TRUE(AL::Math::crossProduct(pPos3D1, pPos3D2).isNear(AL::Math::Position3D(0.2f, -0.7f, 1.2f)));
}


TEST(ALPosition3DTest, Divers)
{
  AL::Math::Position3D pPos3D1 = AL::Math::Position3D();
  AL::Math::Position3D pPos3D2 = AL::Math::Position3D();

  //std::cout << "-------------- soustraction 0 (-a) --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(+1.2f, -1.3f, 0.2f);
  pPos3D2 = -pPos3D1;
  EXPECT_TRUE(pPos3D2.isNear(AL::Math::Position3D(-1.2f, +1.3f, -0.2f)));

  //std::cout << "-------------- soustraction 0 (a-b) --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(+1.2f, 1.3f, 0.2f);
  pPos3D2 = AL::Math::Position3D(-0.5f, 0.2f, 0.4f);
  EXPECT_TRUE((pPos3D2-pPos3D1).isNear(AL::Math::Position3D(-1.7f, -1.1f, 0.2f)));

  //std::cout << "-------------- soustraction 0 (a-=b) --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(-1.2f, +1.3f, -0.2f);
  pPos3D2 = AL::Math::Position3D(+1.2f, -1.3f, 0.2f);
  pPos3D2 -= pPos3D1;
  EXPECT_TRUE(pPos3D2.isNear(AL::Math::Position3D(2.4f, -2.6f, 0.4f)));

  //std::cout << "-------------- addition 0 (+a) --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(+1.2f, 1.3f, 0.3f);
  pPos3D2 = +pPos3D1;
  EXPECT_TRUE(pPos3D2.isNear(AL::Math::Position3D(1.2f, 1.3f, 0.3f)));

  //std::cout << "-------------- addition 0 (a+b) --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(+1.2f, 1.3f, 0.3f);
  pPos3D2 = AL::Math::Position3D(-0.5f, 0.2f, 0.2f);
  EXPECT_TRUE((pPos3D1+pPos3D2).isNear(AL::Math::Position3D(0.7f, 1.5f, 0.5f)));


  //std::cout << "-------------- addition 0 (a+=b) --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(+1.5f, 1.3f, 0.3f);
  pPos3D2 = AL::Math::Position3D(-0.5f, 0.2f, 0.2f);
  pPos3D2 += pPos3D1;
  EXPECT_TRUE(pPos3D2.isNear(AL::Math::Position3D(1.0f, 1.5f, 0.5f)));


  //std::cout << "-------------- multiplication 0 --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(1.0f, 1.0f, 1.0f);
  EXPECT_TRUE((pPos3D1*2.0f).isNear(AL::Math::Position3D(2.0f, 2.0f, 2.0f)));

  //std::cout << "-------------- division 0 --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(2.0f, 2.0f, 2.0f);
  EXPECT_TRUE((pPos3D1/2.0f).isNear(AL::Math::Position3D(1.0f, 1.0f, 1.0f)));

  //std::cout << "-------------- division 1 --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(2.0f, 2.0f, 2.0f);
  ASSERT_THROW((pPos3D1/0.0f), std::runtime_error);
}

TEST(ALPosition3DTest, toVector)
{
  const float eps = 1e-4f;
  const AL::Math::Position3D pos(1.0f, 2.0f, 3.0f);
  const std::vector<float> vec = pos.toVector();

  EXPECT_TRUE(vec.size()==3u);
  EXPECT_NEAR(pos.x, vec[0], eps);
  EXPECT_NEAR(pos.y, vec[1], eps);
  EXPECT_NEAR(pos.z, vec[2], eps);
}
