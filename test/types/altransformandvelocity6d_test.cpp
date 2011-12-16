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

