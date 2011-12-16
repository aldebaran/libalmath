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
#include <almath/tools/aldubinscurve.h>

#include <gtest/gtest.h>
//#include "../almathtestutils.h"

float pCircleRadius = 0.0f;
AL::Math::Pose2D pTargetPose = AL::Math::Pose2D();

TEST(ALDubinsCurveTest, test0)
{
  pCircleRadius = 0.1f;
  pTargetPose = AL::Math::Pose2D(0.5f, 0.5f, 0.0f);

  std::vector<AL::Math::Pose2D> solution = AL::Math::getDubinsSolutions(pTargetPose, pCircleRadius);

  std::cout << "-------------- test 0 - 0 --------------" << std::endl;
  EXPECT_TRUE(solution.at(0).isNear(AL::Math::Pose2D(+0.0777402f, +0.0370996f, +0.890525f)));

  std::cout << "-------------- test 0 - 1 --------------" << std::endl;
  EXPECT_TRUE(solution.at(1).isNear(AL::Math::Pose2D(+0.422260f, +0.462900f, +0.890525f)));

  std::cout << "-------------- test 0 - 2 --------------" << std::endl;
  EXPECT_TRUE(solution.at(2).isNear(AL::Math::Pose2D(+0.500000f, +0.500000f, +0.000000f)));
}


TEST(ALDubinsCurveTest, test1)
{
  pCircleRadius = 0.1f;
  pTargetPose = AL::Math::Pose2D(-0.5f, 0.5f, 0.3f);

  std::vector<AL::Math::Pose2D> solution = AL::Math::getDubinsSolutions(pTargetPose, pCircleRadius);

  std::cout << "-------------- test 1 - 0 --------------" << std::endl;
  EXPECT_TRUE(solution.at(0).isNear(AL::Math::Pose2D(+0.0207914f, +0.197815f, +2.93215f)));

  std::cout << "-------------- test 1 - 1 --------------" << std::endl;
  EXPECT_TRUE(solution.at(1).isNear(AL::Math::Pose2D(-0.491239f, +0.306652f, +2.93215f)));

  std::cout << "-------------- test 1 - 2 --------------" << std::endl;
  EXPECT_TRUE(solution.at(2).isNear(AL::Math::Pose2D(-0.500000f, +0.500000f, +0.300000f)));
}


TEST(ALDubinsCurveTest, test2)
{
  pCircleRadius = 0.1f;
  pTargetPose = AL::Math::Pose2D(0.0f, -1.0f, 0.3f);

  std::vector<AL::Math::Pose2D> solution = AL::Math::getDubinsSolutions(pTargetPose, pCircleRadius);

  std::cout << "-------------- test 2 - 0 --------------" << std::endl;
  EXPECT_TRUE(solution.at(0).isNear(AL::Math::Pose2D(+0.0958873f, -0.128384f, -1.85859f)));

  std::cout << "-------------- test 2 - 1 --------------" << std::endl;
  EXPECT_TRUE(solution.at(1).isNear(AL::Math::Pose2D(-0.125439f, -0.876083f, -1.85859f)));

  std::cout << "-------------- test 2 - 2 --------------" << std::endl;
  EXPECT_TRUE(solution.at(2).isNear(AL::Math::Pose2D(0.0f, -1.0f, +0.3f)));
}

