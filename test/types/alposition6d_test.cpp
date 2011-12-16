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

