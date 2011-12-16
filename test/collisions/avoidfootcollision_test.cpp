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

#include <almath/tools/avoidfootcollision.h>
#include <almath/types/alpose2d.h>
#include <almath/tools/altrigonometry.h>

#include <gtest/gtest.h>

TEST(avoidFootCollisionTest, Log)
{

  AL::Math::Pose2D pRFootFL = AL::Math::Pose2D( 0.080f,  0.038f, 0.0f);
  AL::Math::Pose2D pRFootFR = AL::Math::Pose2D( 0.080f, -0.050f, 0.0f);
  AL::Math::Pose2D pRFootRR = AL::Math::Pose2D(-0.047f, -0.050f, 0.0f);
  AL::Math::Pose2D pRFootRL = AL::Math::Pose2D(-0.047f,  0.038f, 0.0f);

  AL::Math::Pose2D pLFootFL = AL::Math::Pose2D( 0.080f,  0.050f, 0.0f);
  AL::Math::Pose2D pLFootFR = AL::Math::Pose2D( 0.080f, -0.038f, 0.0f);
  AL::Math::Pose2D pLFootRR = AL::Math::Pose2D(-0.047f, -0.038f, 0.0f);
  AL::Math::Pose2D pLFootRL = AL::Math::Pose2D(-0.047f,  0.050f, 0.0f);

  std::vector<AL::Math::Pose2D> pRFootBoundingBox;
  pRFootBoundingBox.push_back(pRFootFL);
  pRFootBoundingBox.push_back(pRFootFR);
  pRFootBoundingBox.push_back(pRFootRR);
  pRFootBoundingBox.push_back(pRFootRL);

  std::vector<AL::Math::Pose2D> pLFootBoundingBox;
  pLFootBoundingBox.push_back(pLFootFL);
  pLFootBoundingBox.push_back(pLFootFR);
  pLFootBoundingBox.push_back(pLFootRR);
  pLFootBoundingBox.push_back(pLFootRL);

  AL::Math::Pose2D pMove;
  bool pResult;

  //std::cout << "***** avoidFootCollision *****" << std::endl;
  pMove = AL::Math::Pose2D( 0.0f, 0.1f, 0.0f*AL::Math::TO_RAD);
  pResult = avoidFootCollision( pLFootBoundingBox,
                                pRFootBoundingBox,
                                false,
                                pMove);
  EXPECT_FALSE(pResult);
  EXPECT_NEAR(pMove.x, 0.0f, 0.0001f );
  EXPECT_NEAR(pMove.y, 0.1f, 0.0001f );
  EXPECT_NEAR(pMove.theta, 0.0f, 0.0001f );

  //std::cout << "***** avoidFootCollision *****" << std::endl;
  pMove = AL::Math::Pose2D( 0.0f, 0.085f, 40.0f*AL::Math::TO_RAD);
  pResult = avoidFootCollision( pLFootBoundingBox,
                                pRFootBoundingBox,
                                false,
                                pMove);
  EXPECT_TRUE(pResult);
  EXPECT_NEAR(pMove.x, 0.0f, 0.0001f );
  EXPECT_NEAR(pMove.y, 0.085f, 0.0001f );
  EXPECT_NEAR(pMove.theta, 0.207257f, 0.0001f );
}


TEST(clipFootWithEllipse, Log)
{
  AL::Math::Pose2D pMove;
  bool pResult;

  //std::cout << "***** clipFootWithEllipse *****" << std::endl;
  pMove = AL::Math::Pose2D( 0.05f, 0.04f, 0.1f);
  pResult = clipFootWithEllipse( 0.08f,
                                 0.06f,
                                 pMove);
  EXPECT_FALSE(pResult);
  EXPECT_NEAR(pMove.x, 0.05f, 0.0001f );
  EXPECT_NEAR(pMove.y, 0.04f, 0.0001f );
  EXPECT_NEAR(pMove.theta, 0.1f, 0.0001f );

  //std::cout << "***** clipFootWithEllipse *****" << std::endl;
  pMove = AL::Math::Pose2D( 0.08f, 0.06f, 0.5f);
  pResult = clipFootWithEllipse( 0.08f,
                                 0.06f,
                                 pMove);
  EXPECT_TRUE(pResult);
  EXPECT_NEAR(pMove.x, 0.05656f, 0.0001f );
  EXPECT_NEAR(pMove.y, 0.04242f, 0.0001f );
  EXPECT_NEAR(pMove.theta, 0.5f, 0.0001f );

  //std::cout << "***** clipFootWithEllipse *****" << std::endl;
  pMove = AL::Math::Pose2D( -0.08f, -0.05f, 0.5f);
  pResult = clipFootWithEllipse( 0.08f,
                                 0.06f,
                                 pMove);
  EXPECT_TRUE(pResult);
  EXPECT_NEAR(pMove.x, -0.06145f, 0.0001f );
  EXPECT_NEAR(pMove.y, -0.03841f, 0.0001f );
  EXPECT_NEAR(pMove.theta, 0.5f, 0.0001f );

  //std::cout << "***** clipFootWithEllipse *****" << std::endl;
  pMove = AL::Math::Pose2D( -0.04f, 0.06f, -0.5f);
  pResult = clipFootWithEllipse( 0.04f,
                                 0.06f,
                                 pMove);
  EXPECT_TRUE(pResult);
  EXPECT_NEAR(pMove.x, -0.02828f, 0.0001f );
  EXPECT_NEAR(pMove.y, 0.04242f, 0.0001f );
  EXPECT_NEAR(pMove.theta, -0.5f, 0.0001f );

  //std::cout << "***** clipFootWithEllipse *****" << std::endl;
  pMove = AL::Math::Pose2D( -0.04f, 0.06f, -0.5f);
  pResult = clipFootWithEllipse( -0.04f,
                                 -0.06f,
                                 pMove);
  EXPECT_TRUE(pResult);
  EXPECT_NEAR(pMove.x, -0.02828f, 0.0001f );
  EXPECT_NEAR(pMove.y, 0.04242f, 0.0001f );
  EXPECT_NEAR(pMove.theta, -0.5f, 0.0001f );

  //std::cout << "***** clipFootWithEllipse *****" << std::endl;
  //  {x: +0.00000, y:+0.160000, theta:+0.00000}
  //  {x: +0.00000, y:-0.160000, theta:+0.00000}

  pMove = AL::Math::Pose2D(0.0f, 0.16f, 0.0f);
  pResult = clipFootWithEllipse( 0.06f,
                                 0.16f,
                                 pMove);
  EXPECT_FALSE(pResult);
}

