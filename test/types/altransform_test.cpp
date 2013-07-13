/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */
#include <almath/types/altransform.h>
#include <almath/tools/almathio.h>

#include <almath/tools/altrigonometry.h>

#include <almath/tools/altransformhelpers.h>

#include <gtest/gtest.h>
#include <cmath>

TEST(TransformTest, constructor)
{
  std::vector<float> pFloats;
  pFloats.resize(8);
  AL::Math::Transform pT = AL::Math::Transform(pFloats);

  EXPECT_TRUE(pT.isNear(AL::Math::Transform()));

  pFloats.resize(12);
  pFloats.at(0)  = 1.0f;
  pFloats.at(1)  = 0.0f;
  pFloats.at(2)  = 0.0f;
  pFloats.at(3)  = 0.0f;
  pFloats.at(4)  = 0.0f;
  pFloats.at(5)  = std::cos(10.0f*AL::Math::TO_RAD);
  pFloats.at(6)  = -std::sin(10.0f*AL::Math::TO_RAD);
  pFloats.at(7)  = 0.0f;
  pFloats.at(8)  = 0.0f;
  pFloats.at(9)  = std::sin(10.0f*AL::Math::TO_RAD);
  pFloats.at(10) = std::cos(10.0f*AL::Math::TO_RAD);
  pFloats.at(11) = 0.0f;
  pT = AL::Math::Transform(pFloats);

  EXPECT_TRUE(pT.isNear(AL::Math::Transform::fromRotX(10.0f*AL::Math::TO_RAD)));

  for (int i=-1; i<2; ++i)
  {
    for (unsigned int j=0; j<12; ++j)
    {
      pFloats[j] = static_cast<float>(i);
    }
    pT = AL::Math::Transform(pFloats);
    EXPECT_TRUE(pT.isTransform());
  }
}


TEST(TransformTest, variousOperator)
{
  AL::Math::Transform pHIn1 = AL::Math::Transform();
  AL::Math::Transform pHIn2 = AL::Math::Transform();
  AL::Math::Transform pHSol = AL::Math::Transform();

  // operator*=
  pHIn1.r1_c4 = 1.0f;
  pHIn1.r2_c4 = 0.5f;
  pHIn1.r3_c4 = 0.2f;

  pHIn2.r1_c4 = 1.0f;
  pHIn2.r2_c4 = 0.5f;
  pHIn2.r3_c4 = 0.2f;

  pHSol.r1_c4 = 2.0f;
  pHSol.r2_c4 = 1.0f;
  pHSol.r3_c4 = 0.4f;

  pHIn1 *= pHIn2;
  EXPECT_TRUE(pHIn1.isNear(pHSol, 0.0001f));

  pHIn1 = AL::Math::Transform::from3DRotation(0.3f, -0.2f, 0.1f);
  pHIn2 = AL::Math::Transform();
  pHIn2 = pHIn1;
  pHIn2 *= pHIn1;
  pHIn1 *= pHIn1;
  EXPECT_TRUE(pHIn2.isNear(pHIn1));

  // operator*
  pHIn1 = AL::Math::Transform();
  pHIn2 = AL::Math::Transform();
  pHSol = AL::Math::Transform();
  AL::Math::Transform pHIn3 = AL::Math::Transform();

  pHIn1.r1_c4 = 1.0f;
  pHIn1.r2_c4 = 0.5f;
  pHIn1.r3_c4 = 0.2f;

  pHIn2.r1_c4 = 1.0f;
  pHIn2.r2_c4 = 0.5f;
  pHIn2.r3_c4 = 0.2f;

  pHIn3 = pHIn1*pHIn2;

  pHSol.r1_c4 = 2.0f;
  pHSol.r2_c4 = 1.0f;
  pHSol.r3_c4 = 0.4f;

  EXPECT_TRUE(pHIn3.isNear(pHSol, 0.0001f));
}

TEST(TransformTest, isNear)
{
  AL::Math::Transform pHIn  = AL::Math::Transform();
  AL::Math::Transform pHSol = AL::Math::Transform();

  EXPECT_TRUE(pHIn.isNear(AL::Math::Transform(), 0.0001f));

  pHSol = AL::Math::Transform();
  pHSol.r1_c1 = 0.1f;
  EXPECT_FALSE(pHIn.isNear(pHSol, 0.0001f));

  pHSol = AL::Math::Transform();
  pHSol.r1_c2 = 0.1f;
  EXPECT_FALSE(pHIn.isNear(pHSol, 0.0001f));

  pHSol = AL::Math::Transform();
  pHSol.r1_c3 = 0.1f;
  EXPECT_FALSE(pHIn.isNear(pHSol, 0.0001f));

  pHSol = AL::Math::Transform();
  pHSol.r1_c4 = 0.1f;
  EXPECT_FALSE(pHIn.isNear(pHSol, 0.0001f));

  pHSol = AL::Math::Transform();
  pHSol.r2_c1 = 0.1f;
  EXPECT_FALSE(pHIn.isNear(pHSol, 0.0001f));

  pHSol = AL::Math::Transform();
  pHSol.r2_c2 = 0.1f;
  EXPECT_FALSE(pHIn.isNear(pHSol, 0.0001f));

  pHSol = AL::Math::Transform();
  pHSol.r2_c3 = 0.1f;
  EXPECT_FALSE(pHIn.isNear(pHSol, 0.0001f));

  pHSol = AL::Math::Transform();
  pHSol.r2_c4 = 0.1f;
  EXPECT_FALSE(pHIn.isNear(pHSol, 0.0001f));

  pHSol = AL::Math::Transform();
  pHSol.r3_c1 = 0.1f;
  EXPECT_FALSE(pHIn.isNear(pHSol, 0.0001f));

  pHSol = AL::Math::Transform();
  pHSol.r3_c2 = 0.1f;
  EXPECT_FALSE(pHIn.isNear(pHSol, 0.0001f));

  pHSol = AL::Math::Transform();
  pHSol.r3_c3 = 0.1f;
  EXPECT_FALSE(pHIn.isNear(pHSol, 0.0001f));

  pHSol = AL::Math::Transform();
  pHSol.r3_c4 = 0.1f;
  EXPECT_FALSE(pHIn.isNear(pHSol, 0.0001f));
}

TEST(TransformTest, isTransform)
{
  float pEps = 0.001f;
  AL::Math::Transform pH;

  // std::cout << "test isTransform 0" << std::endl;
  pH = AL::Math::Transform();
  EXPECT_TRUE(pH.isTransform(pEps));

  // std::cout << "test isTransform 1" << std::endl;
  pH = AL::Math::Transform::from3DRotation(0.1f, 0.2f, 0.3f);
  EXPECT_TRUE(pH.isTransform(pEps));

  // std::cout << "test isTransform 2" << std::endl;
  pH = AL::Math::Transform::fromRotX(0.6f);
  EXPECT_TRUE(pH.isTransform(pEps));

  // std::cout << "test isTransform 3" << std::endl;
  pH = AL::Math::Transform::fromRotX(-0.6f);
  EXPECT_TRUE(pH.isTransform(pEps));

  // std::cout << "test isTransform 4" << std::endl;
  pH = AL::Math::Transform::fromRotY(0.6f);
  EXPECT_TRUE(pH.isTransform(pEps));

  // std::cout << "test isTransform 5" << std::endl;
  pH = AL::Math::Transform::fromRotY(-0.6f);
  EXPECT_TRUE(pH.isTransform(pEps));

  // std::cout << "test isTransform 6" << std::endl;
  pH = AL::Math::Transform::fromRotZ(0.6f);
  EXPECT_TRUE(pH.isTransform(pEps));

  // std::cout << "test isTransform 7" << std::endl;
  pH = AL::Math::Transform::fromRotZ(-0.6f);
  EXPECT_TRUE(pH.isTransform(pEps));

  // std::cout << "test isTransform r1_c1" << std::endl;
  pH = AL::Math::Transform::from3DRotation(0.1f, 0.2f, 0.3f);
  pH.r1_c1 += 0.1f;
  EXPECT_FALSE(pH.isTransform(pEps));

  // std::cout << "test isTransform r1_c2" << std::endl;
  pH = AL::Math::Transform::from3DRotation(0.1f, 0.2f, 0.3f);
  pH.r1_c2 += 0.1f;
  EXPECT_FALSE(pH.isTransform(pEps));

  // std::cout << "test isTransform r1_c3" << std::endl;
  pH = AL::Math::Transform::from3DRotation(0.1f, 0.2f, 0.3f);
  pH.r1_c3 += 0.1f;
  EXPECT_FALSE(pH.isTransform(pEps));

  // std::cout << "test isTransform r2_c1" << std::endl;
  pH = AL::Math::Transform::from3DRotation(0.1f, 0.2f, 0.3f);
  pH.r2_c1 += 0.1f;
  EXPECT_FALSE(pH.isTransform(pEps));

  // std::cout << "test isTransform r2_c2" << std::endl;
  pH = AL::Math::Transform::from3DRotation(0.1f, 0.2f, 0.3f);
  pH.r2_c2 += 0.1f;
  EXPECT_FALSE(pH.isTransform(pEps));

  // std::cout << "test isTransform r2_c3" << std::endl;
  pH = AL::Math::Transform::from3DRotation(0.1f, 0.2f, 0.3f);
  pH.r2_c3 += 0.1f;
  EXPECT_FALSE(pH.isTransform(pEps));

  // std::cout << "test isTransform r3_c1" << std::endl;
  pH = AL::Math::Transform::from3DRotation(0.1f, 0.2f, 0.3f);
  pH.r3_c1 += 0.1f;
  EXPECT_FALSE(pH.isTransform(pEps));

  // std::cout << "test isTransform r3_c2" << std::endl;
  pH = AL::Math::Transform::from3DRotation(0.1f, 0.2f, 0.3f);
  pH.r3_c2 += 0.1f;
  EXPECT_FALSE(pH.isTransform(pEps));

  // std::cout << "test isTransform r3_c3" << std::endl;
  pH = AL::Math::Transform::from3DRotation(0.1f, 0.2f, 0.3f);
  pH.r3_c3 += 0.1f;
  EXPECT_FALSE(pH.isTransform(pEps));
}

TEST(TransformTest, norm)
{
  AL::Math::Transform pH = AL::Math::Transform();
  EXPECT_NEAR(pH.norm(), 0.0f, 0.0001f);

  pH = AL::Math::Transform();
  pH.r1_c4 = 1.0f;
  EXPECT_NEAR(pH.norm(), 1.0f, 0.0001f);

  pH = AL::Math::Transform();
  pH.r2_c4 = 1.0f;
  EXPECT_NEAR(pH.norm(), 1.0f, 0.0001f);

  pH = AL::Math::Transform();
  pH.r3_c4 = 1.0f;
  EXPECT_NEAR(pH.norm(), 1.0f, 0.0001f);

  pH = AL::Math::Transform();
  pH.r1_c4 = -1.0f;
  pH.r2_c4 = -1.0f;
  pH.r3_c4 = -1.0f;
  EXPECT_NEAR(pH.norm(), std::sqrt(3.0f), 0.0001f);
}


TEST(TransformTest, determinant)
{
  AL::Math::Transform pH = AL::Math::Transform();
  EXPECT_NEAR(pH.determinant(), 1.0f, 0.0001f);

  pH = AL::Math::Transform::fromRotX(0.5f);
  pH.r1_c4 = 1.0f;
  EXPECT_NEAR(pH.determinant(), 1.0f, 0.0001f);

  pH = AL::Math::Transform::fromRotY(0.5f);
  pH.r2_c4 = 1.0f;
  EXPECT_NEAR(pH.determinant(), 1.0f, 0.0001f);

  pH = AL::Math::Transform::fromRotZ(0.5f);
  pH.r3_c4 = 1.0f;
  EXPECT_NEAR(pH.determinant(), 1.0f, 0.0001f);
}


TEST(TransformTest, inverse)
{
  AL::Math::Transform pHIn = AL::Math::Transform::fromRotZ(-0.4f);
  pHIn.r1_c4 = 1.0f;
  pHIn.r2_c4 = 3.0f;
  pHIn.r3_c4 = -2.0f;

  AL::Math::Transform pHSol = AL::Math::Transform();
  pHSol.r1_c1 = 0.92106099400289f;
  pHSol.r1_c2 = -0.38941834230865f;
  pHSol.r1_c3 = 0.00000000000000f;
  pHSol.r1_c4 = 0.24719403292307f;
  pHSol.r2_c1 = 0.38941834230865f;
  pHSol.r2_c2 = 0.92106099400289f;
  pHSol.r2_c3 = 0.00000000000000f;
  pHSol.r2_c4 = -3.15260132431731f;
  pHSol.r3_c1 = 0.000000000000000f;
  pHSol.r3_c2 = 0.0000000000000000f;
  pHSol.r3_c3 = 1.00000000000000f;
  pHSol.r3_c4 = 2.00000000000000f;

  EXPECT_TRUE(pHIn.inverse().isNear(pHSol, 0.0001f));
}


TEST(TransformTest, fromRotXYZ)
{
  AL::Math::Transform pHIn  = AL::Math::Transform();
  AL::Math::Transform pHOut = AL::Math::Transform();

  // fromRotX
  pHIn = AL::Math::Transform::fromRotX(0.33f);
  pHOut = AL::Math::Transform();
  pHOut.r1_c1 = 1.0f;
  pHOut.r1_c2 = 0.0f;
  pHOut.r1_c3 = 0.0f;
  pHOut.r2_c1 = 0.0f;
  pHOut.r2_c2 = 0.94604234352839f;
  pHOut.r2_c3 = -0.32404302839487f;
  pHOut.r3_c1 = 0.0f;
  pHOut.r3_c2 = 0.32404302839487f;
  pHOut.r3_c3 = 0.94604234352839f;

  EXPECT_TRUE(pHIn.isNear(pHOut, 0.0001f));

  // fromRotY
  pHIn  = AL::Math::Transform::fromRotY(-0.65f);
  pHOut = AL::Math::Transform();

  pHOut.r1_c1 = 0.79608379854906f;
  pHOut.r1_c2 = 0.0f;
  pHOut.r1_c3 = -0.60518640573604f;
  pHOut.r2_c1 = 0.0f;
  pHOut.r2_c2 = 1.00000000000000f;
  pHOut.r2_c3 = 0.0f;
  pHOut.r3_c1 = 0.60518640573604f;
  pHOut.r3_c2 = 0.0f;
  pHOut.r3_c3 = 0.79608379854906f;

  EXPECT_TRUE(pHIn.isNear(pHOut, 0.0001f));

  // fromRotZ
  pHIn  = AL::Math::Transform::fromRotZ(0.5f);
  pHOut = AL::Math::Transform();
  pHOut.r1_c1 = 0.87758256189037f;
  pHOut.r1_c2 = -0.47942553860420f;
  pHOut.r2_c1 = 0.47942553860420f;
  pHOut.r2_c2 = 0.87758256189037f;
  EXPECT_TRUE(pHIn.isNear(pHOut, 0.0001f));
}

TEST(ALTransformHelpersTest, fromRotXYZ2)
{
  // ************ Position6DFromTransform ************
  std::vector<float> pAngleList;
  AL::Math::Transform pHIn  = AL::Math::Transform();
  AL::Math::Transform pHOut = AL::Math::Transform();

  pAngleList.resize(11);
  pAngleList[0]  =     0.0f;
  pAngleList[1]  =   +10.0f*AL::Math::TO_RAD;
  pAngleList[2]  =   -10.0f*AL::Math::TO_RAD;
  pAngleList[3]  =  +90.00f*AL::Math::TO_RAD;
  pAngleList[4]  =  -90.00f*AL::Math::TO_RAD;
  pAngleList[5]  = +150.00f*AL::Math::TO_RAD;
  pAngleList[6]  = -150.00f*AL::Math::TO_RAD;
  pAngleList[7]  = +180.00f*AL::Math::TO_RAD;
  pAngleList[8]  = -180.00f*AL::Math::TO_RAD;
  pAngleList[9]  = +210.00f*AL::Math::TO_RAD;
  pAngleList[10] = -210.00f*AL::Math::TO_RAD;

  for (unsigned int i=0; i<pAngleList.size(); ++i)
  {
    pHIn  = AL::Math::transformFromRotX(pAngleList[i]);
    pHOut = AL::Math::transformFromPosition6D(AL::Math::position6DFromTransform(pHIn));
    EXPECT_TRUE(pHIn.isNear(pHOut));

    pHIn  = AL::Math::transformFromRotY(pAngleList[i]);
    pHOut = AL::Math::transformFromPosition6D(AL::Math::position6DFromTransform(pHIn));
    EXPECT_TRUE(pHIn.isNear(pHOut));

    pHIn  = AL::Math::transformFromRotZ(pAngleList[i]);
    pHOut = AL::Math::transformFromPosition6D(AL::Math::position6DFromTransform(pHIn));
    EXPECT_TRUE(pHIn.isNear(pHOut));
  }

  std::vector<std::vector<float> > pList;
  pList.resize(5);

  pAngleList.resize(3);

  // 0
  pAngleList[0] =   10.0f*AL::Math::TO_RAD;
  pAngleList[1] = -100.0f*AL::Math::TO_RAD;
  pAngleList[2] =   40.0f*AL::Math::TO_RAD;

  pList[0] = pAngleList;

  // 1
  pAngleList[0] = -180.0f*AL::Math::TO_RAD;
  pAngleList[1] = +150.0f*AL::Math::TO_RAD;
  pAngleList[2] =  270.0f*AL::Math::TO_RAD;

  pList[1] = pAngleList;

  // 2
  pAngleList[0] = -260.0f*AL::Math::TO_RAD;
  pAngleList[1] =  240.0f*AL::Math::TO_RAD;
  pAngleList[2] =  290.0f*AL::Math::TO_RAD;

  pList[2] = pAngleList;

  // 3
  pAngleList[0] = -10.0f*AL::Math::TO_RAD;
  pAngleList[1] = -10.0f*AL::Math::TO_RAD;
  pAngleList[2] = -10.0f*AL::Math::TO_RAD;

  pList[3] = pAngleList;

  // 4
  pAngleList[0] = +180.0f*AL::Math::TO_RAD;
  pAngleList[1] =  -90.0f*AL::Math::TO_RAD;
  pAngleList[2] = +180.0f*AL::Math::TO_RAD;

  pList[4] = pAngleList;

  for (unsigned int i=0; i<pList.size(); ++i)
  {
    pHIn  = AL::Math::transformFromRotX(pList[i][0])*
        AL::Math::transformFromRotY(pList[i][1])*
        AL::Math::transformFromRotZ(pList[i][2]);
    pHOut = AL::Math::transformFromPosition6D(AL::Math::position6DFromTransform(pHIn));
    EXPECT_TRUE(pHIn.isNear(pHOut));

    pHIn  = AL::Math::transformFromRotX(pList[i][2])*
        AL::Math::transformFromRotY(pList[i][0])*
        AL::Math::transformFromRotZ(pList[i][1]);
    pHOut = AL::Math::transformFromPosition6D(AL::Math::position6DFromTransform(pHIn));
    EXPECT_TRUE(pHIn.isNear(pHOut));

    pHIn  = AL::Math::transformFromRotX(pList[i][1])*
        AL::Math::transformFromRotY(pList[i][2])*
        AL::Math::transformFromRotZ(pList[i][0]);
    pHOut = AL::Math::transformFromPosition6D(AL::Math::position6DFromTransform(pHIn));
    EXPECT_TRUE(pHIn.isNear(pHOut));
  }

} // end transformFromRot


TEST(TransformTest, from3DRotation)
{
  AL::Math::Transform pHIn  = AL::Math::Transform::from3DRotation(0.0f, 0.0f, 0.0f);
  AL::Math::Transform pHOut = AL::Math::Transform();
  EXPECT_TRUE(pHIn.isNear(pHOut, 0.00001f));

  // fromRotX
  pHIn  = AL::Math::Transform::from3DRotation(0.33f, 0.0f, 0.0f);
  pHOut = AL::Math::Transform::fromRotX(0.33f);
  EXPECT_TRUE(pHIn.isNear(pHOut, 0.00001f));

  // fromRotY
  pHIn  = AL::Math::Transform::from3DRotation(0.0f, 0.33f, 0.0f);
  pHOut = AL::Math::Transform::fromRotY(0.33f);
  EXPECT_TRUE(pHIn.isNear(pHOut, 0.00001f));

  // fromRotZ
  pHIn  = AL::Math::Transform::from3DRotation(0.0f, 0.0f, 0.33f);
  pHOut = AL::Math::Transform::fromRotZ(0.33f);
  EXPECT_TRUE(pHIn.isNear(pHOut, 0.00001f));
}


TEST(TransformTest, fromPosition)
{
  AL::Math::Transform pHIn  = AL::Math::Transform::fromPosition(1.0f, 0.2f, 0.1f);
  AL::Math::Transform pHOut = AL::Math::Transform();
  pHOut.r1_c4 = 1.0f;
  pHOut.r2_c4 = 0.2f;
  pHOut.r3_c4 = 0.1f;
  EXPECT_TRUE(pHIn.isNear(pHOut, 0.00001f));
}


TEST(TransformTest, fromPositionAndRotation)
{
  AL::Math::Transform pHIn  = AL::Math::Transform::fromPosition(1.0f, 0.2f, 0.1f, 0.0f, 0.0f, 0.0f);
  AL::Math::Transform pHOut = AL::Math::Transform();
  pHOut.r1_c4 = 1.0f;
  pHOut.r2_c4 = 0.2f;
  pHOut.r3_c4 = 0.1f;
  EXPECT_TRUE(pHIn.isNear(pHOut, 0.00001f));

  pHIn  = AL::Math::Transform::fromPosition(1.0f, 0.2f, 0.1f, 0.2f, 0.0f, 0.0f);
  pHOut = AL::Math::Transform::fromRotX(0.2f);
  pHOut.r1_c4 = 1.0f;
  pHOut.r2_c4 = 0.2f;
  pHOut.r3_c4 = 0.1f;
  EXPECT_TRUE(pHIn.isNear(pHOut, 0.00001f));

  pHIn  = AL::Math::Transform::fromPosition(1.0f, 0.2f, 0.1f, 0.0f, 0.2f, 0.0f);
  pHOut = AL::Math::Transform::fromRotY(0.2f);
  pHOut.r1_c4 = 1.0f;
  pHOut.r2_c4 = 0.2f;
  pHOut.r3_c4 = 0.1f;
  EXPECT_TRUE(pHIn.isNear(pHOut, 0.00001f));

  pHIn  = AL::Math::Transform::fromPosition(1.0f, 0.2f, 0.1f, 0.0f, 0.0f, 0.2f);
  pHOut = AL::Math::Transform::fromRotZ(0.2f);
  pHOut.r1_c4 = 1.0f;
  pHOut.r2_c4 = 0.2f;
  pHOut.r3_c4 = 0.1f;
  EXPECT_TRUE(pHIn.isNear(pHOut, 0.00001f));
}


TEST(TransformTest, diff)
{
  AL::Math::Transform pHIn1 = AL::Math::Transform::fromRotX(0.2f);
  AL::Math::Transform pHIn2 = AL::Math::Transform::fromRotX(-0.3f);
  AL::Math::Transform pHOut = AL::Math::Transform::fromRotX(-0.5f);
  EXPECT_TRUE(pHIn1.diff(pHIn2).isNear(pHOut, 0.0001f));

  pHIn1 = AL::Math::Transform::fromRotY(0.2f);
  pHIn2 = AL::Math::Transform::fromRotY(-0.3f);
  pHOut = AL::Math::Transform::fromRotY(-0.5f);
  EXPECT_TRUE(pHIn1.diff(pHIn2).isNear(pHOut, 0.0001f));

  pHIn1 = AL::Math::Transform::fromRotZ(0.2f);
  pHIn2 = AL::Math::Transform::fromRotZ(-0.3f);
  pHOut = AL::Math::Transform::fromRotZ(-0.5f);
  EXPECT_TRUE(pHIn1.diff(pHIn2).isNear(pHOut, 0.0001f));
}


TEST(TransformTest, distanceSquared)
{
  AL::Math::Transform pHIn1 = AL::Math::Transform::fromPosition(1.0f, 1.0f, 1.0f);
  AL::Math::Transform pHIn2 = AL::Math::Transform::fromPosition(0.0f, 0.0f, 0.0f);
  EXPECT_NEAR(pHIn1.distanceSquared(pHIn2), 3.0f, 0.0001f);
}


TEST(TransformTest, distance)
{
  AL::Math::Transform pHIn1 = AL::Math::Transform::fromPosition(1.0f, 1.0f, 1.0f);
  AL::Math::Transform pHIn2 = AL::Math::Transform::fromPosition(0.0f, 0.0f, 0.0f);
  EXPECT_NEAR(pHIn1.distance(pHIn2), std::sqrt(3.0f), 0.0001f);
}

TEST(TransformTest, Determinant)
{
  AL::Math::Transform pHIn = AL::Math::Transform();
  EXPECT_NEAR(pHIn.determinant(), 1.0f, 0.0001f);

  pHIn = AL::Math::Transform::fromRotX(0.5f);
  EXPECT_NEAR(pHIn.determinant(), 1.0f, 0.0001f);

  pHIn = AL::Math::Transform::fromRotY(0.6f);
  EXPECT_NEAR(pHIn.determinant(), 1.0f, 0.0001f);

  pHIn = AL::Math::Transform::fromRotZ(0.7f);
  EXPECT_NEAR(pHIn.determinant(), 1.0f, 0.0001f);
}

TEST(TransformTest, inverse0)
{
  EXPECT_TRUE(AL::Math::transformInverse(AL::Math::Transform()).isNear(AL::Math::Transform()));

  AL::Math::Transform pHIn  = AL::Math::Transform::fromRotX(0.5f);
  AL::Math::Transform pHOut = AL::Math::Transform::fromRotX(-0.5f);
  EXPECT_TRUE(pHIn.inverse().isNear(pHOut, 0.0001f));

  pHIn  = AL::Math::Transform::fromRotY(0.5f);
  pHOut = AL::Math::Transform::fromRotY(-0.5f);
  EXPECT_TRUE(pHIn.inverse().isNear(pHOut, 0.0001f));

  pHIn  = AL::Math::Transform::fromRotZ(0.5f);
  pHOut = AL::Math::Transform::fromRotZ(-0.5f);
  EXPECT_TRUE(pHIn.inverse().isNear(pHOut, 0.0001f));

  pHIn  = AL::Math::Transform::fromPosition(0.1f, 0.2f, 0.3f);
  pHOut = AL::Math::Transform::fromPosition(-0.1f, -0.2f, -0.3f);
  EXPECT_TRUE(pHIn.inverse().isNear(pHOut, 0.0001f));
}


TEST(TransformTest, strangeConstruction)
{
 std::vector<float> listData;
 listData.push_back(1.0f);
 listData.push_back(2.0f);
 listData.push_back(3.0f);

 AL::Math::Transform pH(listData);

 EXPECT_TRUE(pH.isNear(AL::Math::Transform()));
}

TEST(TransformTest, transformFromPosition)
{
  const float lEpsilon = 0.00001f;
  const AL::Math::Transform tf0 = AL::Math::transformFromPosition(0.1f, 0.2f, 0.3f);
  const AL::Math::Transform tf1 = AL::Math::Transform::fromPosition(0.1f, 0.2f, 0.3f);

  EXPECT_TRUE(tf0.isNear(tf1, lEpsilon));
  EXPECT_NEAR(tf0.r1_c4, 0.1f, lEpsilon);
  EXPECT_NEAR(tf0.r2_c4, 0.2f, lEpsilon);
  EXPECT_NEAR(tf0.r3_c4, 0.3f, lEpsilon);
}


TEST(TransformTest, normalizeTransform)
{
  AL::Math::Transform tf1;
  AL::Math::Transform tf2;
  AL::Math::Transform tf3;

  // case null data
  tf1 = AL::Math::Transform();
  tf1.r1_c1 = 0.0f;
  tf1.r2_c2 = 0.0f;
  tf1.r3_c3 = 0.0f;
  EXPECT_FALSE(tf1.isTransform());
  AL::Math::normalizeTransform(tf1);
  EXPECT_TRUE(tf1.isTransform());
  EXPECT_TRUE(tf1.isNear(AL::Math::Transform()));

  // case not normalized
  tf1.r1_c1 = 0.5f;
  tf1.r2_c2 = 0.5f;
  tf1.r3_c3 = 0.5f;
  EXPECT_FALSE(tf1.isTransform());
  AL::Math::normalizeTransform(tf1);
  EXPECT_TRUE(tf1.isTransform());
  EXPECT_TRUE(tf1.isNear(AL::Math::Transform()));

  for (unsigned int i=-360; i<360; ++i)
  {
    float angleX = static_cast<float>(i)*AL::Math::TO_RAD;
    for (unsigned int j=-360; j<360; ++j)
    {
      float angleY = static_cast<float>(j)*AL::Math::TO_RAD;
      for (unsigned int k=-360; k<360; ++k)
      {
        float angleZ = static_cast<float>(k)*AL::Math::TO_RAD;
        tf1 = AL::Math::Transform::fromRotX(angleX)*\
            AL::Math::Transform::fromRotY(angleY)*\
            AL::Math::Transform::fromRotZ(angleZ);

        tf2 = tf1;
        tf3 = tf1;

        tf1.r1_c1 += 0.01f;
        tf1.r1_c2 += 0.01f;
        tf1.r1_c3 += 0.01f;

        tf1.r2_c1 += 0.01f;
        tf1.r2_c2 += 0.01f;
        tf1.r2_c3 += 0.01f;

        tf1.r3_c1 += 0.01f;
        tf1.r3_c2 += 0.01f;
        tf1.r3_c3 += 0.01f;
        EXPECT_FALSE(tf1.isTransform());
        AL::Math::normalizeTransform(tf1);
        EXPECT_TRUE(tf1.isTransform());

        EXPECT_TRUE(tf2.isTransform());
        AL::Math::normalizeTransform(tf2);
        EXPECT_TRUE(tf2.isTransform());
        EXPECT_TRUE(tf2.isNear(tf3));
      }
    }
  }
}


TEST(ALTransformTest, toVector)
{
  const float eps = 1e-4f;
  std::vector<float> input(12, 0.0f);
  input.at(0)  = 1.0f;
  input.at(5)  = 1.0f;
  input.at(10) = 1.0f;

  const AL::Math::Transform tf(input);
  const std::vector<float> vec = tf.toVector();

  EXPECT_TRUE(vec.size()==16);

  EXPECT_NEAR(tf.r1_c1, vec.at(0), eps);
  EXPECT_NEAR(tf.r1_c2, vec.at(1), eps);
  EXPECT_NEAR(tf.r1_c3, vec.at(2), eps);
  EXPECT_NEAR(tf.r1_c4, vec.at(3), eps);

  EXPECT_NEAR(tf.r2_c1, vec.at(4), eps);
  EXPECT_NEAR(tf.r2_c2, vec.at(5), eps);
  EXPECT_NEAR(tf.r2_c3, vec.at(6), eps);
  EXPECT_NEAR(tf.r2_c4, vec.at(7), eps);

  EXPECT_NEAR(tf.r3_c1, vec.at(8), eps);
  EXPECT_NEAR(tf.r3_c2, vec.at(9), eps);
  EXPECT_NEAR(tf.r3_c3, vec.at(10), eps);
  EXPECT_NEAR(tf.r3_c4, vec.at(11), eps);
}
