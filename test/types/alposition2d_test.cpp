/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */
#include <almath/types/alposition2d.h>

#include <gtest/gtest.h>
#include <stdexcept>

//#include "../almathtestutils.h"

AL::Math::Position2D pPos2D1 = AL::Math::Position2D();
AL::Math::Position2D pPos2D2 = AL::Math::Position2D();

TEST(ALPosition2DTest, Distance)
{
  float kEpsilon = 0.0001f;

  //std::cout << "-------------- Distance 0 --------------" << std::endl;
  pPos2D1 = AL::Math::Position2D();
  pPos2D2 = AL::Math::Position2D();
  EXPECT_NEAR(AL::Math::distance(pPos2D1, pPos2D2), 0.0f, kEpsilon);


  //std::cout << "-------------- Distance 1 --------------" << std::endl;
  pPos2D1 = AL::Math::Position2D();
  pPos2D2 = AL::Math::Position2D(1.0f, 0.0f);
  EXPECT_NEAR(AL::Math::distance(pPos2D1, pPos2D2), 1.0f, kEpsilon);


  //std::cout << "-------------- Distance 2 --------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(0.5f, -0.8f);
  pPos2D2 = AL::Math::Position2D(1.0f, 1.2f);
  EXPECT_NEAR(AL::Math::distance(pPos2D1, pPos2D2), 2.06155281280883f, kEpsilon);
}

TEST(ALPosition2DTest, norm)
{
    float kEpsilon = 0.0001f;

  //std::cout << "-------------- norm 0 --------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(0.0f, 0.0f);
  EXPECT_NEAR(AL::Math::norm(pPos2D1), 0.0f, kEpsilon);


  //std::cout << "-------------- norm 1 --------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(1.0f, 0.0f);
  EXPECT_NEAR(AL::Math::norm(pPos2D1), 1.0f, kEpsilon);


  //std::cout << "-------------- norm 2 --------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(1.0f, -0.8f);
  EXPECT_NEAR(AL::Math::norm(pPos2D1), 1.28062484748657f, kEpsilon);
}


TEST(ALPosition2DTest, normalize)
{
  //std::cout << "-------------- normalize 0 --------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(0.0f, 0.0f);
  ASSERT_THROW(AL::Math::normalize(pPos2D1), std::runtime_error);


  //std::cout << "-------------- normalize 1 --------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(0.5f, 0.0f);
  EXPECT_TRUE(AL::Math::normalize(pPos2D1).isNear(AL::Math::Position2D(1.0f, 0.0f)));


  //std::cout << "-------------- normalize 2 --------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(1.0f, -1.0f);
  EXPECT_TRUE(AL::Math::normalize(pPos2D1).isNear(AL::Math::Position2D(0.70710678118655f, -0.70710678118655f)));
}


TEST(ALPosition2DTest, crossProduct)
{
  float kEpsilon = 0.0001f;

  //std::cout << "-------------- crossProduct 0 --------------" << std::endl;
  pPos2D1 = AL::Math::Position2D();
  pPos2D2 = AL::Math::Position2D();
  EXPECT_NEAR(AL::Math::crossProduct(pPos2D1, pPos2D2), 0.0f, kEpsilon);


  //std::cout << "-------------- crossProduct 1 --------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(+1.0f, 2.0f);
  pPos2D2 = AL::Math::Position2D(-0.5f, 0.2f);
  EXPECT_NEAR(AL::Math::crossProduct(pPos2D1, pPos2D2), 1.2f, kEpsilon);
}

TEST(ALPosition2DTest, Divers)
{
  //std::cout << "-------------- soustraction 0 (a-b) --------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(+1.2f, 1.3f);
  pPos2D2 = AL::Math::Position2D(-0.5f, 0.2f);
  EXPECT_TRUE((pPos2D2-pPos2D1).isNear(AL::Math::Position2D(-1.7f, -1.1f)));

  //std::cout << "-------------- soustraction 0 (-a) --------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(1.2f, -1.3f);
  pPos2D2 = AL::Math::Position2D();
  pPos2D2 = - pPos2D1;
  EXPECT_TRUE(pPos2D2.isNear(AL::Math::Position2D(-1.2f, 1.3f)));


  //std::cout << "-------------- soustraction 0 (-=a)--------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(1.2f, -1.5f);
  pPos2D2 = AL::Math::Position2D(-1.2f, +1.5f);
  pPos2D2 -= pPos2D1;
  EXPECT_TRUE(pPos2D2.isNear(AL::Math::Position2D(-2.4f, +3.0f)));


  //std::cout << "-------------- addition 0 (a+b)--------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(+1.2f, 1.3f);
  pPos2D2 = AL::Math::Position2D(-0.5f, 0.2f);
  EXPECT_TRUE((pPos2D1+pPos2D2).isNear(AL::Math::Position2D(0.7f, 1.5f)));


  //std::cout << "-------------- addition 0 (+a)--------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(1.2f, -1.5f);
  pPos2D2 = AL::Math::Position2D();
  pPos2D2 = + pPos2D1;
  EXPECT_TRUE(pPos2D2.isNear(AL::Math::Position2D(1.2f, -1.5f)));


  //std::cout << "-------------- addition 0 (+=a)--------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(1.2f, -1.5f);
  pPos2D2 = AL::Math::Position2D(1.2f, -1.5f);
  pPos2D2 += pPos2D1;
  EXPECT_TRUE(pPos2D2.isNear(AL::Math::Position2D(2.4f, -3.0f)));


  //std::cout << "-------------- multiplication 0 --------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(1.0f, 1.0f);
  EXPECT_TRUE((pPos2D1*2.0f).isNear(AL::Math::Position2D(2.0f, 2.0f)));


  //std::cout << "-------------- multiplication 1 --------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(1.0f, 1.0f);
  EXPECT_TRUE((2.0f*pPos2D1).isNear(AL::Math::Position2D(2.0f, 2.0f)));


  //std::cout << "-------------- division 0 --------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(2.0f, 2.0f);
  EXPECT_TRUE((pPos2D1/2.0f).isNear(AL::Math::Position2D(1.0f, 1.0f)));


  //std::cout << "-------------- division 1 --------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(2.0f, 2.0f);
  ASSERT_THROW((pPos2D1/0.0f), std::runtime_error);
}

