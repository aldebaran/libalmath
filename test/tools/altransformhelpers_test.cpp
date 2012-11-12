/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/tools/altransformhelpers.h>
#include <almath/tools/almath.h> // for Velocity6D = float * Position6D
#include <almath/tools/almathio.h>
#include <almath/tools/altrigonometry.h>

#include <gtest/gtest.h>
#include <stdexcept>
#include <cmath>

TEST(ALTransformHelpersTest, transformFromRotationPosition3D)
{
  AL::Math::Rotation  pRot = AL::Math::Rotation::fromRotX(0.5f);
  AL::Math::Transform pHIn = AL::Math::transformFromRotationPosition3D(pRot, 0.2f, 0.5f, -1.2f);
  AL::Math::Transform pHOut = AL::Math::Transform::fromRotX(0.5f);
  pHOut.r1_c4 =  0.2f;
  pHOut.r2_c4 =  0.5f;
  pHOut.r3_c4 = -1.2f;
  EXPECT_TRUE(pHIn.isNear(pHOut, 0.0001f));


  pRot = AL::Math::Rotation::fromRotY(0.5f);
  pHIn = AL::Math::transformFromRotationPosition3D(pRot, 0.2f, 0.5f, -1.2f);
  pHOut = AL::Math::Transform::fromRotY(0.5f);
  pHOut.r1_c4 =  0.2f;
  pHOut.r2_c4 =  0.5f;
  pHOut.r3_c4 = -1.2f;
  EXPECT_TRUE(pHIn.isNear(pHOut, 0.0001f));


  pRot = AL::Math::Rotation::fromRotZ(0.5f);
  pHIn = AL::Math::transformFromRotationPosition3D(pRot, 0.2f, 0.5f, -1.2f);
  pHOut = AL::Math::Transform::fromRotZ(0.5f);
  pHOut.r1_c4 =  0.2f;
  pHOut.r2_c4 =  0.5f;
  pHOut.r3_c4 = -1.2f;
  EXPECT_TRUE(pHIn.isNear(pHOut, 0.0001f));


  AL::Math::Position3D pPos3D = AL::Math::Position3D(0.1f, 0.2f, 0.3f);
  pRot = AL::Math::Rotation::fromRotX(0.5f);
  pHOut = AL::Math::Transform::fromRotX(0.5f);
  pHOut.r1_c4 = 0.1f;
  pHOut.r2_c4 = 0.2f;
  pHOut.r3_c4 = 0.3f;
  pHIn = AL::Math::transformFromRotationPosition3D(pRot, pPos3D);
  EXPECT_TRUE(pHIn.isNear(pHOut, 0.0001f));


  pPos3D = AL::Math::Position3D(0.1f, 0.2f, 0.3f);
  pRot = AL::Math::Rotation::fromRotY(0.5f);
  pHOut = AL::Math::Transform::fromRotY(0.5f);
  pHOut.r1_c4 = 0.1f;
  pHOut.r2_c4 = 0.2f;
  pHOut.r3_c4 = 0.3f;
  pHIn = AL::Math::transformFromRotationPosition3D(pRot, pPos3D);
  EXPECT_TRUE(pHIn.isNear(pHOut, 0.0001f));


  pPos3D = AL::Math::Position3D(0.1f, 0.2f, 0.3f);
  pRot = AL::Math::Rotation::fromRotZ(0.5f);
  pHOut = AL::Math::Transform::fromRotZ(0.5f);
  pHOut.r1_c4 = 0.1f;
  pHOut.r2_c4 = 0.2f;
  pHOut.r3_c4 = 0.3f;
  pHIn = AL::Math::transformFromRotationPosition3D(pRot, pPos3D);
  EXPECT_TRUE(pHIn.isNear(pHOut, 0.0001f));
}

TEST(ALTransformHelpersTest, normalizeTransform)
{
  float eps = 0.0001f;

  std::vector<AL::Math::Transform> pHList;
  pHList.resize(9);

  for (unsigned int i=0; i<9; i++)
  {
    pHList.at(i) = AL::Math::Transform::from3DRotation(0.2f, 0.3f, 0.4f);
  }

  pHList.at(0).r1_c1 += 0.1f;
  pHList.at(1).r1_c2 += 0.1f;
  pHList.at(2).r1_c3 += 0.1f;

  pHList.at(3).r2_c1 += 0.1f;
  pHList.at(4).r2_c2 += 0.1f;
  pHList.at(5).r2_c3 += 0.1f;

  pHList.at(6).r3_c1 += 0.1f;
  pHList.at(7).r3_c2 += 0.1f;
  pHList.at(8).r3_c3 += 0.1f;

  AL::Math::Transform pHInit;
  AL::Math::Position6D pTmp;
  AL::Math::Transform pHFinal;

  for (unsigned int i=0; i<pHList.size(); i++)
  {
    pHInit = pHList.at(i);

    EXPECT_FALSE(pHInit.isTransform(eps));

    pTmp = AL::Math::position6DFromTransform(pHInit);
    pHFinal = AL::Math::transformFromPosition6D(pTmp);

    EXPECT_TRUE(pHFinal.isTransform(eps));
  }
}

TEST(ALTransformHelpersTest, transformLogarithmInPlace)
{
  // ************ TransformLogarithme ************

  // X01
  AL::Math::Transform pMathHIn  = AL::Math::transformFromRotX(10.0f*AL::Math::TO_RAD);
  AL::Math::Velocity6D pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.1745329f, 0.0f, 0.0f), 1.0e-5f));

  // X02
  pMathHIn  = AL::Math::transformFromRotX(150.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 2.61799387799149f, 0.0f, 0.0f), 1.0e-5f));

  // X03
  pMathHIn  = AL::Math::transformFromRotX(179.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 3.12413936106983f, 0.0f, 0.0f), 1.0e-5f));

  // X04
  pMathHIn  = AL::Math::transformFromRotX(181.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, -3.12413936106983f, 0.0f, 0.0f), 1.0e-5f));

  // X05
  pMathHIn  = AL::Math::transformFromRotX(-179.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, -3.12413936106983f, 0.0f, 0.0f), 1.0e-5f));

  // X06
  pMathHIn  = AL::Math::transformFromRotX(-181.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 3.12413936106983f, 0.0f, 0.0f), 1.0e-5f));

  // X07
  pMathHIn  = AL::Math::transformFromRotX(90.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 1.57079632679490f, 0.0f, 0.0f), 1.0e-5f));

  // X08
  pMathHIn  = AL::Math::transformFromRotX(10.0f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = 0.1f;
  pMathHIn.r2_c4 = 0.2f;
  pMathHIn.r3_c4 = 0.3f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.10000000000000f, 0.22567198513826f, 0.28178477701757f, 0.17453292519943f, 0.0f, 0.0f), 1.0e-5f));

  // X09
  pMathHIn  = AL::Math::transformFromRotX(179.0f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = 0.1f;
  pMathHIn.r2_c4 = 0.2f;
  pMathHIn.r3_c4 = 0.3f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.10000000000000f, 0.47134729927687f, -0.30832434343239f, 3.12413936106983f, 0.0f, 0.0f), 1.0e-5f));

  // X10
  pMathHIn  = AL::Math::transformFromRotX(181.0f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = 0.1f;
  pMathHIn.r2_c4 = 0.2f;
  pMathHIn.r3_c4 = 0.3f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.10000000000000f, -0.46589450904408f, 0.31650352878158f, -3.12413936106983f, 0.0f, 0.0f), 1.0e-5f));

  // X11
  pMathHIn  = AL::Math::transformFromRotX(179.99f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, +3.14141812066260f, 0.0f, 0.0f), 1.0e-5f)); // devrait etre -

  // X12
  pMathHIn  = AL::Math::transformFromRotX(180.01f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, +3.14141812066260f, 0.0f, 0.0f), 1.0e-5f));

  // X13
  pMathHIn  = AL::Math::transformFromRotX(180.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 3.14159265358979f, 0.0f, 0.0f), 1.0e-5f));

  // X14
  pMathHIn  = AL::Math::transformFromRotX(-179.99f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, +3.14141812066260f, 0.0f, 0.0f), 1.0e-5f)); // devrait etre -

  // X15
  pMathHIn  = AL::Math::transformFromRotX(-180.01f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, +3.14141812066260f, 0.0f, 0.0f), 1.0e-5f));

  // X16
  pMathHIn  = AL::Math::transformFromRotX(-180.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, +3.14159265358979f, 0.0f, 0.0f), 1.0e-5f));

  // X17
  pMathHIn  = AL::Math::transformFromRotX(+180.0f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = 0.1f;
  pMathHIn.r2_c4 = 0.2f;
  pMathHIn.r3_c4 = 0.3f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.10000000000000f, +0.47123884392623f, -0.31415928915466f, +3.14159247705847f, 0.0f, 0.0f), 1.0e-5f));

  // X18
  pMathHIn = AL::Math::transformFromRotX(+0.1f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = 0.1f;
  pMathHIn.r2_c4 = 0.2f;
  pMathHIn.r3_c4 = 0.3f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.10000000000000f, 0.20026174861823f, 0.29982539092044f, 0.00174532925199f, 0.0f, 0.0f), 1.0e-5f));

  // X19
  pMathHIn  = AL::Math::transformFromRotX(-0.1f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = 0.1f;
  pMathHIn.r2_c4 = 0.2f;
  pMathHIn.r3_c4 = 0.3f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.10000000000000f, 0.19973814984263f, 0.30017445677084f, -0.00174532925199f, 0.0f, 0.0f), 1.0e-5f));

  // X20
  pMathHIn  = AL::Math::transformFromRotX(+1.0f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = 0.1f;
  pMathHIn.r2_c4 = 0.2f;
  pMathHIn.r3_c4 = 0.3f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.10000000000000f, 0.20261291689522f, 0.29824705527385f, 0.01745329251994f, 0.0f, 0.0f), 1.0e-5f));

  // X21
  pMathHIn  = AL::Math::transformFromRotX(-1.0f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = 0.1f;
  pMathHIn.r2_c4 = 0.2f;
  pMathHIn.r3_c4 = 0.3f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.10000000000000f, 0.19737692913924f, 0.30173771377784f, -0.01745329251994f, 0.0f, 0.0f), 1.0e-5f));

  // Y01
  pMathHIn  = AL::Math::transformFromRotY(10.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, 0.1745329f, 0.0f), 1.0e-5f));

  // Y02
  pMathHIn  = AL::Math::transformFromRotY(150.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, 2.61799387799149f, 0.0f), 1.0e-5f));

  // Y03
  pMathHIn  = AL::Math::transformFromRotY(179.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, 3.12413936106983f, 0.0f), 1.0e-5f));

  // Y04
  pMathHIn  = AL::Math::transformFromRotY(181.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, -3.12413936106983f, 0.0f), 1.0e-5f));

  // Y05
  pMathHIn  = AL::Math::transformFromRotY(-179.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, -3.12413936106983f, 0.0f), 1.0e-5f));

  // Y06
  pMathHIn  = AL::Math::transformFromRotY(-181.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, 3.12413936106983f, 0.0f), 1.0e-5f));

  // Y07
  pMathHIn  = AL::Math::transformFromRotY(90.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, 1.57079632679490f, 0.0f), 1.0e-5f));

  // Y08
  pMathHIn  = AL::Math::transformFromRotY(10.0f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = 0.1f;
  pMathHIn.r2_c4 = 0.2f;
  pMathHIn.r3_c4 = 0.3f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.07356608439926f, 0.20000000000000f, 0.30796471579749f, 0.0f, 0.17453292519943f, 0.0f), 1.0e-5f));

  // Y09
  pMathHIn  = AL::Math::transformFromRotY(179.0f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = 0.1f;
  pMathHIn.r2_c4 = 0.2f;
  pMathHIn.r3_c4 = 0.3f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(-0.46725770660228f, 0.20000000000000f, 0.16029656072809f, 0.0f, 3.12413936106983f, 0.0f), 1.0e-5f));

  // Y10
  pMathHIn  = AL::Math::transformFromRotY(181.0f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = 0.1f;
  pMathHIn.r2_c4 = 0.2f;
  pMathHIn.r3_c4 = 0.3f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.46998410171867f, 0.20000000000000f, -0.15211737537889f, 0.0f, -3.12413936106983f, 0.0f), 1.0e-5f));

  // Y11
  pMathHIn  = AL::Math::transformFromRotY(179.99f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, +3.14141812066260f, 0.0f), 1.0e-5f)); // devrait etre -

  // Y12
  pMathHIn  = AL::Math::transformFromRotY(180.01f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, +3.14141812066260f, 0.0f), 1.0e-5f));

  // Y13
  pMathHIn  = AL::Math::transformFromRotY(180.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, 3.14159265358979f, 0.0f), 1.0e-5f));

  // Y14
  pMathHIn  = AL::Math::transformFromRotY(-179.99f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, +3.14141812066260f, 0.0f), 1.0e-5f)); // devrait etre -

  // Y15
  pMathHIn  = AL::Math::transformFromRotY(-180.01f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, +3.14141812066260f, 0.0f), 1.0e-5f));

  // Y16
  pMathHIn  = AL::Math::transformFromRotY(-180.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, +3.14159265358979f, 0.0f), 1.0e-5f));

  // Y17
  pMathHIn  = AL::Math::transformFromRotY(+180.0f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = 0.1f;
  pMathHIn.r2_c4 = 0.2f;
  pMathHIn.r3_c4 = 0.3f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(-0.47092190248947f, 0.20000000000000f, 0.15803061932648f, 0.0f, 3.14159265358979f, 0.0f), 1.0e-3f));

  // Y18
  pMathHIn  = AL::Math::transformFromRotY(+0.1f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = -0.2f;
  pMathHIn.r2_c4 = -0.3f;
  pMathHIn.r3_c4 = -0.1f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(-0.19991268276783f, -0.30000000000000f, -0.10017450754041f, 0.0f, 0.00174532925199f, 0.0f), 1.0e-5f));

  // Y19
  pMathHIn  = AL::Math::transformFromRotY(-0.1f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = -0.2f;
  pMathHIn.r2_c4 = -0.3f;
  pMathHIn.r3_c4 = -0.1f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(-0.20008721569303f, -0.30000000000000f, -0.09982544169001f, 0.0f, -0.00174532925199f, 0.0f), 1.0e-5f));

  // Y20
  pMathHIn  = AL::Math::transformFromRotY(+1.0f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = -0.2f;
  pMathHIn.r2_c4 = -0.3f;
  pMathHIn.r3_c4 = -0.1f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(-0.19912225839123f, -0.30000000000000f, -0.10174279076061f, 0.0f, 0.01745329251994f, 0.0f), 1.0e-5f));

  // Y21
  pMathHIn  = AL::Math::transformFromRotY(-1.0f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = -0.2f;
  pMathHIn.r2_c4 = -0.3f;
  pMathHIn.r3_c4 = -0.1f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(-0.20086758764323f, -0.30000000000000f, -0.09825213225662f, 0.0f, -0.01745329251994f, 0.0f), 1.0e-5f));

  // Z01
  pMathHIn  = AL::Math::transformFromRotZ(10.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1745329f), 1.0e-5f));

  // Z02
  pMathHIn  = AL::Math::transformFromRotZ(150.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.61799387799149f), 1.0e-5f));

  // Z03
  pMathHIn  = AL::Math::transformFromRotZ(179.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3.12413936106983f), 1.0e-5f));

  // Z04
  pMathHIn  = AL::Math::transformFromRotZ(181.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -3.12413936106983f), 1.0e-5f));

  // Z05
  pMathHIn  = AL::Math::transformFromRotZ(-179.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -3.12413936106983f), 1.0e-5f));

  // Z06
  pMathHIn  = AL::Math::transformFromRotZ(-181.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3.12413936106983f), 1.0e-5f));

  // Z07
  pMathHIn  = AL::Math::transformFromRotZ(90.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.57079632679490f), 1.0e-5f));

  // Z08
  pMathHIn  = AL::Math::transformFromRotZ(10.0f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = 0.1f;
  pMathHIn.r2_c4 = 0.2f;
  pMathHIn.r3_c4 = 0.3f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.11719931569912f, 0.19076540009837f, 0.30000000000000f, 0.0f, 0.0f, 0.17453292519943f), 1.0e-5f));

  // Z09
  pMathHIn  = AL::Math::transformFromRotZ(179.0f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = 0.1f;
  pMathHIn.r2_c4 = 0.2f;
  pMathHIn.r3_c4 = 0.3f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.31377713366518f, -0.15348057293709f, 0.30000000000000f, 0.0f, 0.0f, 3.12413936106983f), 1.0e-5f));

  // Z10
  pMathHIn  = AL::Math::transformFromRotZ(181.0f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = 0.1f;
  pMathHIn.r2_c4 = 0.2f;
  pMathHIn.r3_c4 = 0.3f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(-0.31105073854878f, 0.15893336316989f, 0.30000000000000f, 0.0f, 0.0f, -3.12413936106983f), 1.0e-5f));

  // Z11
  pMathHIn  = AL::Math::transformFromRotZ(179.99f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, +3.14141812066260f), 1.0e-5f)); // devrait etre -

  // Z12
  pMathHIn  = AL::Math::transformFromRotZ(180.01f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, +3.14141812066260f), 1.0e-5f));

  // Z13
  pMathHIn  = AL::Math::transformFromRotZ(180.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3.14159265358979f), 1.0e-5f));

  // Z14
  pMathHIn  = AL::Math::transformFromRotZ(-179.99f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, +3.14141812066260f), 1.0e-5f)); // devrait etre -

  // Z15
  pMathHIn  = AL::Math::transformFromRotZ(-180.01f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, +3.14141812066260f), 1.0e-5f));

  // Z16
  pMathHIn  = AL::Math::transformFromRotZ(-180.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, +3.14159265358979f), 1.0e-5f));

  // Z17
  pMathHIn  = AL::Math::transformFromRotZ(+180.0f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = 0.1f;
  pMathHIn.r2_c4 = 0.2f;
  pMathHIn.r3_c4 = 0.3f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.31447626090798f, -0.15644564158149f, 0.30000000000000f, 0.0f, 0.0f, 3.14159265358979f), 1.0e-3f));

  // Z18
  pMathHIn  = AL::Math::transformFromRotZ(+0.1f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = -0.2f;
  pMathHIn.r2_c4 = -0.3f;
  pMathHIn.r3_c4 = -0.1f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(-0.20026174861823f, -0.29982539092044f, -0.10000000000000f, 0.0f, 0.0f, 0.00174532925199f), 1.0e-5f));

  // Z19
  pMathHIn  = AL::Math::transformFromRotZ(-0.1f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = -0.2f;
  pMathHIn.r2_c4 = -0.3f;
  pMathHIn.r3_c4 = -0.1f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(-0.19973814984263f, -0.30017445677084f, -0.10000000000000f, 0.0f, 0.0f, -0.00174532925199f), 1.0e-5f));

  // Z20
  pMathHIn  = AL::Math::transformFromRotZ(+1.0f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = -0.2f;
  pMathHIn.r2_c4 = -0.3f;
  pMathHIn.r3_c4 = -0.1f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(-0.20261291689522f, -0.29824705527385f, -0.10000000000000f, 0.0f, 0.0f, 0.01745329251994f), 1.0e-5f));

  // Z21
  pMathHIn  = AL::Math::transformFromRotZ(-1.0f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = -0.2f;
  pMathHIn.r2_c4 = -0.3f;
  pMathHIn.r3_c4 = -0.1f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(-0.19737692913924f, -0.30173771377784f, -0.10000000000000f, 0.0f, 0.0f, -0.01745329251994f), 1.0e-5f));

  // 00
  pMathHIn  = AL::Math::transformFromRotX(0.0f);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f), 1.0e-5f));

  // 01
  pMathHIn  = AL::Math::transformFromRotX(0.0f);
  pMathHIn.r1_c4 = 0.1f;
  pMathHIn.r2_c4 = 0.2f;
  pMathHIn.r3_c4 = 0.3f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.1f, 0.2f, 0.3f, 0.0f, 0.0f, 0.0f), 1.0e-5f));

  // 02
  pMathHIn  = AL::Math::transformFromRotZ(110.0f*AL::Math::TO_RAD)*
      AL::Math::transformFromRotX(-30.0f*AL::Math::TO_RAD)*
      AL::Math::transformFromRotY(-150.0f*AL::Math::TO_RAD);
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, -2.19530551593940f, 1.78449667326725f, -1.05305343403968f), 1.0e-5f));

  // 03
  pMathHIn  = AL::Math::transformFromRotZ(110.0f*AL::Math::TO_RAD)*
      AL::Math::transformFromRotX(-30.0f*AL::Math::TO_RAD)*
      AL::Math::transformFromRotY(-150.0f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = 0.1f;
  pMathHIn.r2_c4 = 0.2f;
  pMathHIn.r3_c4 = 0.3f;
  pMathVOut = AL::Math::Velocity6D();
  AL::Math::transformLogarithmInPlace(pMathHIn, pMathVOut);
  EXPECT_TRUE(pMathVOut.isNear(AL::Math::Velocity6D(-0.32467607279532f, -0.28979289026856f, 0.35532477264459f, -2.19530551593940f, 1.78449667326725f, -1.05305343403968f), 1.0e-5f));

} // end TransformLogarithmInPlace


TEST(ALTransformHelpersTest, velocityExponentialInPlace)
{
  // ************ velocityExponentialInPlace ************
  AL::Math::Velocity6D pMathVIn  = AL::Math::Velocity6D();
  AL::Math::Transform  pMathHOut = AL::Math::Transform();
  AL::Math::Transform  pMathHSol = AL::Math::Transform();

  AL::Math::velocityExponentialInPlace(pMathVIn, pMathHOut);
  EXPECT_TRUE(pMathHOut.isNear(AL::Math::Transform()));

  pMathVIn  = AL::Math::Velocity6D(1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f);
  pMathHOut = AL::Math::Transform();
  pMathHSol = AL::Math::Transform();

  pMathHSol.r1_c1 = 0.22629564095021f;
  pMathHSol.r1_c2 = -0.18300791965762f;
  pMathHSol.r1_c3 = 0.95671227870741f;

  pMathHSol.r2_c1 = 0.95671227870741f;
  pMathHSol.r2_c2 = 0.22629564095021f;
  pMathHSol.r2_c3 = -0.18300791965762f;

  pMathHSol.r3_c1 = -0.18300791965762f;
  pMathHSol.r3_c2 = 0.95671227870741f;
  pMathHSol.r3_c3 = 0.22629564095021f;

  pMathHSol.r1_c4 = 1.0f;
  pMathHSol.r2_c4 = 1.0f;
  pMathHSol.r3_c4 = 1.0f;

  AL::Math::velocityExponentialInPlace(pMathVIn, pMathHOut);
  EXPECT_TRUE(pMathHOut.isNear(pMathHSol));

  pMathVIn  = AL::Math::Velocity6D(1.0f, 0.4f, -0.2f, -0.5f, 0.6f, -0.7f);
  pMathHOut = AL::Math::Transform();
  pMathHSol = AL::Math::Transform();

  pMathHSol.r1_c1 = 0.61255758028090f;
  pMathHSol.r1_c2 = 0.44179855168103f;
  pMathHSol.r1_c3 = 0.65542905838310f;

  pMathHSol.r2_c1 = -0.71528731854157f;
  pMathHSol.r2_c2 = 0.66269718753866f;
  pMathHSol.r2_c3 = 0.22180281684855f;

  pMathHSol.r3_c1 = -0.33635883037913f;
  pMathHSol.r3_c2 = -0.60468709045331f;
  pMathHSol.r3_c3 = 0.72195308702511f;

  pMathHSol.r1_c4 = 0.90888443200578f;
  pMathHSol.r2_c4 = -0.04541275047598f;
  pMathHSol.r3_c4 = -0.51669980898354f;

  AL::Math::velocityExponentialInPlace(pMathVIn, pMathHOut);
  EXPECT_TRUE(pMathHOut.isNear(pMathHSol));

} // end VelocityExponentialInPlace

TEST(ALTransformHelpersTest, changeRepereVelocity6D)
{

}

TEST(ALTransformHelpersTest, changeReperePosition6D)
{

}

TEST(ALTransformHelpersTest, changeReperePosition3D)
{
  AL::Math::Transform pTransf = AL::Math::Transform(1.0f, 2.0f, 3.0f);
  AL::Math::Position3D pPos3D = AL::Math::Position3D(10.0f, 20.0f, 30.0f);
  AL::Math::Position3D pPosIn = pTransf * pPos3D;
  AL::Math::Position3D pPosOut = AL::Math::Position3D(11.0f, 22.0f, 33.0f);
  EXPECT_TRUE(pPosIn.isNear(pPosOut, 0.0001f));
}

TEST(ALTransformHelpersTest, changeRepereTransposePosition3D)
{

}

TEST(ALTransformHelpersTest, changeRepereTransform)
{
}

TEST(ALTransformHelpersTest, changeRepereTransposeTransform)
{
}

TEST(ALTransformHelpersTest, changeRepereTransposeVelocity6D)
{
}

TEST(ALTransformHelpersTest, changeRepereTransposePosition6D)
{
}

TEST(ALTransformHelpersTest, transformMeanInPlace)
{
  // ************ TransformMeanInPlace ************

  // 0
  AL::Math::Transform pMathHIn1 = AL::Math::Transform();
  AL::Math::Transform pMathHIn2 = AL::Math::Transform();
  AL::Math::Transform pMathHOut = AL::Math::Transform();

  AL::Math::transformMeanInPlace(pMathHIn1, pMathHIn2, 0.0f, pMathHOut);
  EXPECT_TRUE(pMathHIn1.isNear(pMathHOut));

  // 1
  pMathHIn1 = AL::Math::transformFromRotX(10.0f*AL::Math::TO_RAD);
  pMathHIn2 = AL::Math::transformFromRotX(30.0f*AL::Math::TO_RAD);

  AL::Math::transformMeanInPlace(pMathHIn1, pMathHIn2, 0.5f, pMathHOut);
  EXPECT_TRUE(AL::Math::transformFromRotX(20.0f*AL::Math::TO_RAD).isNear(pMathHOut, 5.0e-4f));

  // 2
  pMathHIn1 = AL::Math::transformFromRotY(10.0f*AL::Math::TO_RAD);
  pMathHIn2 = AL::Math::transformFromRotY(30.0f*AL::Math::TO_RAD);

  AL::Math::transformMeanInPlace(pMathHIn1, pMathHIn2, 0.5f, pMathHOut);
  EXPECT_TRUE(AL::Math::transformFromRotY(20.0f*AL::Math::TO_RAD).isNear(pMathHOut, 5.0e-4f));

  // 3
  pMathHIn1 = AL::Math::transformFromRotZ(10.0f*AL::Math::TO_RAD);
  pMathHIn2 = AL::Math::transformFromRotZ(30.0f*AL::Math::TO_RAD);

  AL::Math::transformMeanInPlace(pMathHIn1, pMathHIn2, 0.5f, pMathHOut);
  EXPECT_TRUE(AL::Math::transformFromRotZ(20.0f*AL::Math::TO_RAD).isNear(pMathHOut, 5.0e-4f));

  // 4
  pMathHIn1 = AL::Math::transformFromRotZ(10.0f*AL::Math::TO_RAD);
  pMathHIn2 = AL::Math::transformFromRotZ(30.0f*AL::Math::TO_RAD);
  pMathHOut = AL::Math::Transform();
  ASSERT_THROW(AL::Math::transformMeanInPlace(pMathHIn1, pMathHIn2, 1.1f, pMathHOut),
               std::runtime_error);

  ASSERT_THROW(AL::Math::transformMeanInPlace(pMathHIn1, pMathHIn2, -0.1f, pMathHOut),
               std::runtime_error);
} // end transformMean

TEST(ALTransformHelpersTest, transformFromPosition3DInPlace)
{
}

TEST(ALTransformHelpersTest, rotationToTransform)
{
}

TEST(ALTransformHelpersTest, rotationFromTransform)
{
}

TEST(ALTransformHelpersTest, transformToPosition3D)
{
}

TEST(ALTransformHelpersTest, position6DFromTransform)
{
}

TEST(ALTransformHelpersTest, transformFromPose2D)
{
}

TEST(ALTransformHelpersTest, pose2DFromTransform)
{
}

TEST(ALTransformHelpersTest, transformFromRotation3D)
{
}

TEST(ALTransformHelpersTest, transformFromPosition6D)
{
}

TEST(ALTransformHelpersTest, position6DFromTransformDiff)
{
  AL::Math::Transform pHCurrent;
  AL::Math::Transform pHTarget;
  AL::Math::Position6D pResult;

  AL::Math::Position6D pExpected;

  // TODO hight priority
  AL::Math::position6DFromTransformDiffInPlace(pHCurrent, pHTarget, pResult);

  EXPECT_TRUE(pResult.isNear(pExpected));
}

TEST(ALTransformHelpersTest, position3DFromTransformInPlace)
{
}

TEST(ALTransformHelpersTest, position3DFromTransform)
{
}

TEST(ALTransformHelpersTest, rotationPosition3DToTransform)
{
}

TEST(ALTransformHelpersTest, rotation3DFromTransform)
{
}

TEST(ALTransformHelpersTest, rotation2DFromTransformZ)
{
}


TEST(ALTransformHelpersTest, transformFromRotVec)
{
}

TEST(ALTransformHelpersTest, axisRotationProjection)
{
}


TEST(ALTransformHelpersTest, transformAxisRotationProjectionInPlace)
{
  // ************ axisRotationProjectionInPlace ************

  // 01
  AL::Math::Rotation   pMathRIn = AL::Math::Rotation();
  AL::Math::Position3D pMathPIn = AL::Math::Position3D();

  ASSERT_THROW(AL::Math::axisRotationProjectionInPlace(pMathPIn, pMathRIn), std::runtime_error);

  // 02
  AL::Math::Transform  pMathHIn  = AL::Math::Transform();
  AL::Math::Transform  pMathHOut = AL::Math::Transform();
  pMathPIn  = AL::Math::Position3D(1.0f, 0.0f, 0.0f);

  AL::Math::axisRotationProjectionInPlace(pMathPIn, pMathHIn);
  EXPECT_TRUE(pMathHIn.isNear(AL::Math::Transform()));

  // 03
  pMathHIn  = AL::Math::transformFromRotX(10.0f*AL::Math::TO_RAD);
  pMathPIn  = AL::Math::Position3D(0.0f, 0.0f, 1.0f);

  AL::Math::axisRotationProjectionInPlace(pMathPIn, pMathHIn);
  EXPECT_TRUE(pMathHIn.isNear(AL::Math::Transform()));

  // 04
  pMathHIn  = AL::Math::transformFromRotY(10.0f*AL::Math::TO_RAD)*
      AL::Math::transformFromRotX(10.0f*AL::Math::TO_RAD);
  pMathPIn  = AL::Math::Position3D(0.0f, 1.0f, 0.0f);

  AL::Math::axisRotationProjectionInPlace(pMathPIn, pMathHIn);
  EXPECT_TRUE(pMathHIn.isNear(AL::Math::transformFromRotY(10.0f*AL::Math::TO_RAD)));

  // 05
  pMathHIn  = AL::Math::transformFromRotZ(110.0f*AL::Math::TO_RAD)*
      AL::Math::transformFromRotX(-30.0f*AL::Math::TO_RAD)*
      AL::Math::transformFromRotY(-150.0f*AL::Math::TO_RAD);
  pMathHIn.r1_c4 = 10.0f;
  pMathHIn.r2_c4 = -5.0f;
  pMathHIn.r3_c4 = 12.0f;

  pMathPIn  = AL::Math::Position3D(0.5f, 0.5f, 0.0f);

  pMathHOut.r1_c1 = 0.67130774833459f;
  pMathHOut.r1_c2 = 0.32869225166541f;
  pMathHOut.r1_c3 = -0.66430504972634f;
  pMathHOut.r1_c4 = 10.0f;

  pMathHOut.r2_c1 = 0.32869225166541f;
  pMathHOut.r2_c2 = 0.67130774833459f;
  pMathHOut.r2_c3 = 0.66430504972634f;
  pMathHOut.r2_c4 = -5.0f;

  pMathHOut.r3_c1 = 0.66430504972634f;
  pMathHOut.r3_c2 = -0.66430504972634f;
  pMathHOut.r3_c3 = 0.34261549666919f;
  pMathHOut.r3_c4 = 12.0f;

  AL::Math::axisRotationProjectionInPlace(pMathPIn, pMathHIn);
  EXPECT_TRUE(pMathHIn.isNear(pMathHOut));

  // 06
  pMathHIn  = AL::Math::transformFromRotZ(110.0f*AL::Math::TO_RAD)*
      AL::Math::transformFromRotX(-30.0f*AL::Math::TO_RAD)*
      AL::Math::transformFromRotY(-150.0f*AL::Math::TO_RAD);
  pMathPIn  = AL::Math::Position3D(1.0f, 0.0f, 0.0f);

  pMathHOut = AL::Math::Transform();

  pMathHOut.r1_c1 = 1.0f;
  pMathHOut.r1_c2 = 0.0f;
  pMathHOut.r1_c3 = 0.0f;

  pMathHOut.r2_c1 = 0.0f;
  pMathHOut.r2_c2 = -0.91900358951769f;
  pMathHOut.r2_c3 = -0.39424916290792f;

  pMathHOut.r3_c1 = 0.0f;
  pMathHOut.r3_c2 = 0.39424916290792f;
  pMathHOut.r3_c3 = -0.91900358951769f;

  AL::Math::axisRotationProjectionInPlace(pMathPIn, pMathHIn);
  EXPECT_TRUE(pMathHIn.isNear(pMathHOut));

  // 07
  pMathHIn  = AL::Math::transformFromRotZ(110.0f*AL::Math::TO_RAD)*
      AL::Math::transformFromRotX(-30.0f*AL::Math::TO_RAD)*
      AL::Math::transformFromRotY(-150.0f*AL::Math::TO_RAD);
  pMathPIn  = AL::Math::Position3D(0.0f, 1.0f, 0.0f);

  pMathHOut.r1_c1 = -0.14011259929748f;
  pMathHOut.r1_c2 = 0.0f;
  pMathHOut.r1_c3 = 0.99013557633190f;

  pMathHOut.r2_c1 = 0.0f;
  pMathHOut.r2_c2 = 1.0f;
  pMathHOut.r2_c3 = 0.0f;

  pMathHOut.r3_c1 = -0.99013557633190f;
  pMathHOut.r3_c2 = 0.0f;
  pMathHOut.r3_c3 = -0.14011259929748f;

  AL::Math::axisRotationProjectionInPlace(pMathPIn, pMathHIn);
  EXPECT_TRUE(pMathHIn.isNear(pMathHOut));

  // 08
  pMathHIn  = AL::Math::transformFromRotZ(110.0f*AL::Math::TO_RAD)*
      AL::Math::transformFromRotX(-30.0f*AL::Math::TO_RAD)*
      AL::Math::transformFromRotY(-150.0f*AL::Math::TO_RAD);
  pMathPIn  = AL::Math::Position3D(0.0f, 0.0f, 1.0f);

  pMathHOut.r1_c1 = +0.34202014332567f;
  pMathHOut.r1_c2 = +0.93969262078591f;
  pMathHOut.r1_c3 = 0.0f;

  pMathHOut.r2_c1 = -0.93969262078591f;
  pMathHOut.r2_c2 = +0.34202014332567f;
  pMathHOut.r2_c3 = 0.0f;

  pMathHOut.r3_c1 = 0.0f;
  pMathHOut.r3_c2 = 0.0f;
  pMathHOut.r3_c3 = 1.0f;

  AL::Math::axisRotationProjectionInPlace(pMathPIn, pMathHIn);
#ifdef __x86_64__
  // FIXME: this does not work on 64 bits.
  // Use double instead of float?
  // Increase epsilon ?
  // Check what happens around singularities?
#else
  EXPECT_TRUE(pMathHIn.isNear(pMathHOut));
#endif

} // end transform AxisRotationProjectionInPlace

TEST(ALTransformHelpersTest, rotationAxisRotationProjectionInPlace)
{
  AL::Math::Rotation pMathROut;
  // ************ AxisRotationProjectionInPlace ************

  // 01
  AL::Math::Rotation   pMathRIn = AL::Math::Rotation();
  AL::Math::Position3D pMathPIn = AL::Math::Position3D();

  ASSERT_THROW(axisRotationProjectionInPlace(pMathPIn, pMathRIn), std::runtime_error);

  // 02
  pMathRIn  = AL::Math::Rotation();
  pMathPIn  = AL::Math::Position3D(1.0f, 0.0f, 0.0f);

  AL::Math::axisRotationProjectionInPlace(pMathPIn, pMathRIn);
  EXPECT_TRUE(pMathRIn.isNear(AL::Math::Rotation()));

  // 03
  pMathRIn  = AL::Math::rotationFromRotX(10.0f*AL::Math::TO_RAD);
  pMathPIn  = AL::Math::Position3D(0.0f, 0.0f, 1.0f);

  AL::Math::axisRotationProjectionInPlace(pMathPIn, pMathRIn);
  EXPECT_TRUE(pMathRIn.isNear(AL::Math::Rotation()));

  // 04
  pMathRIn  = AL::Math::rotationFromRotY(10.0f*AL::Math::TO_RAD)*
      AL::Math::rotationFromRotX(10.0f*AL::Math::TO_RAD);
  pMathPIn  = AL::Math::Position3D(0.0f, 1.0f, 0.0f);

  AL::Math::axisRotationProjectionInPlace(pMathPIn, pMathRIn);
  EXPECT_TRUE(pMathRIn.isNear(AL::Math::rotationFromRotY(10.0f*AL::Math::TO_RAD)));

  // 05
  pMathRIn  = AL::Math::rotationFromRotZ(110.0f*AL::Math::TO_RAD)*
      AL::Math::rotationFromRotX(-30.0f*AL::Math::TO_RAD)*
      AL::Math::rotationFromRotY(-150.0f*AL::Math::TO_RAD);
  pMathPIn  = AL::Math::Position3D(0.5f, 0.5f, 0.0f);

  pMathROut.r1_c1 = 0.67130774833459f;
  pMathROut.r1_c2 = 0.32869225166541f;
  pMathROut.r1_c3 = -0.66430504972634f;

  pMathROut.r2_c1 = 0.32869225166541f;
  pMathROut.r2_c2 = 0.67130774833459f;
  pMathROut.r2_c3 = 0.66430504972634f;

  pMathROut.r3_c1 = 0.66430504972634f;
  pMathROut.r3_c2 = -0.66430504972634f;
  pMathROut.r3_c3 = 0.34261549666919f;

  AL::Math::axisRotationProjectionInPlace(pMathPIn, pMathRIn);
  EXPECT_TRUE(pMathRIn.isNear(pMathROut));

  // 06
  pMathRIn  = AL::Math::rotationFromRotZ(110.0f*AL::Math::TO_RAD)*
      AL::Math::rotationFromRotX(-30.0f*AL::Math::TO_RAD)*
      AL::Math::rotationFromRotY(-150.0f*AL::Math::TO_RAD);
  pMathPIn  = AL::Math::Position3D(1.0f, 0.0f, 0.0f);

  pMathROut.r1_c1 = 1.0f;
  pMathROut.r1_c2 = 0.0f;
  pMathROut.r1_c3 = 0.0f;

  pMathROut.r2_c1 = 0.0f;
  pMathROut.r2_c2 = -0.91900358951769f;
  pMathROut.r2_c3 = -0.39424916290792f;

  pMathROut.r3_c1 = 0.0f;
  pMathROut.r3_c2 = 0.39424916290792f;
  pMathROut.r3_c3 = -0.91900358951769f;

  AL::Math::axisRotationProjectionInPlace(pMathPIn, pMathRIn);
  EXPECT_TRUE(pMathRIn.isNear(pMathROut));

  // 07
  pMathRIn  = AL::Math::rotationFromRotZ(110.0f*AL::Math::TO_RAD)*
      AL::Math::rotationFromRotX(-30.0f*AL::Math::TO_RAD)*
      AL::Math::rotationFromRotY(-150.0f*AL::Math::TO_RAD);
  pMathPIn  = AL::Math::Position3D(0.0f, 1.0f, 0.0f);

  pMathROut.r1_c1 = -0.14011259929748f;
  pMathROut.r1_c2 = 0.0f;
  pMathROut.r1_c3 = 0.99013557633190f;

  pMathROut.r2_c1 = 0.0f;
  pMathROut.r2_c2 = 1.0f;
  pMathROut.r2_c3 = 0.0f;

  pMathROut.r3_c1 = -0.99013557633190f;
  pMathROut.r3_c2 = 0.0f;
  pMathROut.r3_c3 = -0.14011259929748f;

  AL::Math::axisRotationProjectionInPlace(pMathPIn, pMathRIn);
  EXPECT_TRUE(pMathRIn.isNear(pMathROut));

  // 08
  pMathRIn  = AL::Math::rotationFromRotZ(110.0f*AL::Math::TO_RAD)*
      AL::Math::rotationFromRotX(-30.0f*AL::Math::TO_RAD)*
      AL::Math::rotationFromRotY(-150.0f*AL::Math::TO_RAD);
  pMathPIn  = AL::Math::Position3D(0.0f, 0.0f, 1.0f);

  pMathROut.r1_c1 = +0.34202014332567f;
  pMathROut.r1_c2 = +0.93969262078591f;
  pMathROut.r1_c3 = 0.0f;

  pMathROut.r2_c1 = -0.93969262078591f;
  pMathROut.r2_c2 = +0.34202014332567f;
  pMathROut.r2_c3 = 0.0f;

  pMathROut.r3_c1 = 0.0f;
  pMathROut.r3_c2 = 0.0f;
  pMathROut.r3_c3 = 1.0f;

  AL::Math::axisRotationProjectionInPlace(pMathPIn, pMathRIn);
#ifdef __x86_64__
  // FIXME: does not work on 64 bits.
#else
  EXPECT_TRUE(pMathRIn.isNear(pMathROut));
#endif

} // end rotationAxisRotationProjectionInPlace


TEST(ALControlExplosionTest, test0)
{
  // Inititalisation
  float Kp = 30.0f;
  float dt = 0.02f;

  AL::Math::Transform  fHTrunkWorld = AL::Math::Transform(0.1f, 0.1f, 0.2f);
  AL::Math::Transform  fHTrunkWorldDes;
  AL::Math::Position6D fDeltaTrunkWorldPosition;
  AL::Math::Velocity6D fVTrunkWorldDes;
  AL::Math::Velocity6D fVTrunkWorldConsigne;
  AL::Math::Velocity6D fVTrunkTorsoConsigne;

  // Main purpose: for an updated fHTrunkWorldDes, compute a control in order to
  // fHTrunkWorld = fHTrunkWorldDes

  for (unsigned int i=0; i<200; i++)
  {
    // Update fHTrunkWorldDes
    fHTrunkWorldDes = AL::Math::transformFromRotZ(2.0f*i*AL::Math::TO_RAD)*
        AL::Math::transformFromRotX(-0.2f*i*AL::Math::TO_RAD)*
        AL::Math::transformFromRotY(i*AL::Math::TO_RAD);
    fHTrunkWorldDes.r1_c4 = 0.1f+i*0.1f;
    fHTrunkWorldDes.r2_c4 = 0.1f-i*0.05f;
    fHTrunkWorldDes.r3_c4 = 0.2f+i*0.2f;

    for (unsigned j=0; j<1; j++)
    {
      AL::Math::position6DFromTransformDiffInPlace(
            fHTrunkWorld,
            fHTrunkWorldDes,
            fDeltaTrunkWorldPosition);

      // Compute BODY VELOCITY DES
      fVTrunkWorldConsigne = fVTrunkWorldDes + Kp * fDeltaTrunkWorldPosition;

      AL::Math::changeReferenceTransposeVelocity6D(
            fHTrunkWorld,
            fVTrunkWorldConsigne,
            fVTrunkTorsoConsigne);

      AL::Math::Transform fDelta_H_root = AL::Math::velocityExponential(dt*fVTrunkTorsoConsigne);
      fHTrunkWorld = fHTrunkWorld * fDelta_H_root;
    }
  }

  // On ne met plus a jour fHTrunkWorldDes
  // fHTrunkWorld doit converger vers fHTrunkWorldDes
  for (unsigned j=0; j<100; j++)
  {
    AL::Math::position6DFromTransformDiffInPlace(
          fHTrunkWorld,
          fHTrunkWorldDes,
          fDeltaTrunkWorldPosition);

    // Compute BODY VELOCITY DES
    fVTrunkWorldConsigne = fVTrunkWorldDes + Kp * fDeltaTrunkWorldPosition;

    AL::Math::changeReferenceTransposeVelocity6D(
          fHTrunkWorld,
          fVTrunkWorldConsigne,
          fVTrunkTorsoConsigne);

    AL::Math::Transform fDelta_H_root = AL::Math::velocityExponential(dt*fVTrunkTorsoConsigne);
    fHTrunkWorld = fHTrunkWorld * fDelta_H_root;
  }
}

TEST(ALTransformHelpersTest, orthogonalSpace)
{
  AL::Math::Position3D pAxis;
  AL::Math::Transform  pH;

  // test 0
  pAxis = AL::Math::Position3D(1.0f, 0.0f, 0.0f);
  pH = AL::Math::orthogonalSpace(pAxis);
  EXPECT_NEAR(pH.determinant(), 1.0f, 0.001f);

  // test 1
  pAxis = AL::Math::Position3D(-0.5f, 0.0f, 0.0f);
  pH = AL::Math::orthogonalSpace(pAxis);
  EXPECT_NEAR(pH.determinant(), 1.0f, 0.001f);

  // test 2
  pAxis = AL::Math::Position3D(0.0f, 1.0f, 0.0f);
  pH = AL::Math::orthogonalSpace(pAxis);
  EXPECT_NEAR(pH.determinant(), 1.0f, 0.001f);

  // test 3
  pAxis = AL::Math::Position3D(0.0f, -0.5f, 0.0f);
  pH = AL::Math::orthogonalSpace(pAxis);
  EXPECT_NEAR(pH.determinant(), 1.0f, 0.001f);

  // test 4
  pAxis = AL::Math::Position3D(0.0f, 0.0f, 1.0f);
  pH = AL::Math::orthogonalSpace(pAxis);
  EXPECT_NEAR(pH.determinant(), 1.0f, 0.001f);

  // test 5
  pAxis = AL::Math::Position3D(0.0f, 0.0f, -0.5f);
  pH = AL::Math::orthogonalSpace(pAxis);
  EXPECT_NEAR(pH.determinant(), 1.0f, 0.001f);

  // test 6
  pAxis = AL::Math::Position3D(1.0f, 1.0f, 1.0f);
  pH = AL::Math::orthogonalSpace(pAxis);
  EXPECT_NEAR(pH.determinant(), 1.0f, 0.001f);

  // test 7
  pAxis = AL::Math::Position3D(-0.5f, -0.5f, -0.5f);
  pH = AL::Math::orthogonalSpace(pAxis);
  EXPECT_NEAR(pH.determinant(), 1.0f, 0.001f);

  // test 8
  pAxis = AL::Math::Position3D(0.01f, -0.5f, -0.5f);
  pH = AL::Math::orthogonalSpace(pAxis);
  EXPECT_NEAR(pH.determinant(), 1.0f, 0.001f);
}


TEST(ALTransformHelpersTest, transformFromQuaternion)
{
  AL::Math::Quaternion pQua;
  AL::Math::Transform  pT;

//  std::cout << "transformFromQuaternion: identity quaternion" << std::endl;
  pT   = AL::Math::Transform();
  pQua = AL::Math::Quaternion();
  pT   = AL::Math::transformFromQuaternion(pQua);
  EXPECT_TRUE(pT.isNear(AL::Math::Transform(), 0.0001f));

//  std::cout << "transformFromQuaternion: 180 deg turn around X axis" << std::endl;
  pT   = AL::Math::Transform();
  pQua = AL::Math::Quaternion(0.0f, 1.0f, 0.0f, 0.0f);
  pT   = AL::Math::transformFromQuaternion(pQua);
  EXPECT_TRUE(pT.isNear(AL::Math::Transform::fromRotX(180.0f*AL::Math::TO_RAD), 0.0001f));

//  std::cout << "transformFromQuaternion: 180 deg turn around Y axis" << std::endl;
  pT   = AL::Math::Transform();
  pQua = AL::Math::Quaternion(0.0f, 0.0f, 1.0f, 0.0f);
  pT   = AL::Math::transformFromQuaternion(pQua);
  EXPECT_TRUE(pT.isNear(AL::Math::Transform::fromRotY(180.0f*AL::Math::TO_RAD), 0.0001f));

//  std::cout << "transformFromQuaternion: 180 deg turn around Z axis" << std::endl;
  pT   = AL::Math::Transform();
  pQua = AL::Math::Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
  pT   = AL::Math::transformFromQuaternion(pQua);
  EXPECT_TRUE(pT.isNear(AL::Math::Transform::fromRotZ(180.0f*AL::Math::TO_RAD), 0.0001f));

//  std::cout << "transformFromQuaternion: 90 deg turn around X axis" << std::endl;
  pT   = AL::Math::Transform();
  pQua = AL::Math::Quaternion(sqrtf(0.5f), sqrtf(0.5f), 0.0f, 0.0f);
  pT   = AL::Math::transformFromQuaternion(pQua);
  EXPECT_TRUE(pT.isNear(AL::Math::Transform::fromRotX(90.0f*AL::Math::TO_RAD), 0.0001f));

//  std::cout << "transformFromQuaternion: 90 deg turn around Y axis" << std::endl;
  pT   = AL::Math::Transform();
  pQua = AL::Math::Quaternion(sqrtf(0.5f), 0.0f, sqrtf(0.5f), 0.0f);
  pT   = AL::Math::transformFromQuaternion(pQua);
  EXPECT_TRUE(pT.isNear(AL::Math::Transform::fromRotY(90.0f*AL::Math::TO_RAD), 0.0001f));

//  std::cout << "transformFromQuaternion: 90 deg turn around Z axis" << std::endl;
  pT   = AL::Math::Transform();
  pQua = AL::Math::Quaternion(sqrtf(0.5f), 0.0f, 0.0f, sqrtf(0.5f));
  pT   = AL::Math::transformFromQuaternion(pQua);
  EXPECT_TRUE(pT.isNear(AL::Math::Transform::fromRotZ(90.0f*AL::Math::TO_RAD), 0.0001f));

//  std::cout << "transformFromQuaternion: -90 deg turn around X axis" << std::endl;
  pT   = AL::Math::Transform();
  pQua = AL::Math::Quaternion(sqrtf(0.5f), -sqrtf(0.5f), 0.0f, 0.0f);
  pT   = AL::Math::transformFromQuaternion(pQua);
  EXPECT_TRUE(pT.isNear(AL::Math::Transform::fromRotX(-90.0f*AL::Math::TO_RAD), 0.0001f));

//  std::cout << "transformFromQuaternion: -90 deg turn around Y axis" << std::endl;
  pT   = AL::Math::Transform();
  pQua = AL::Math::Quaternion(sqrtf(0.5f), 0.0f, -sqrtf(0.5f), 0.0f);
  pT   = AL::Math::transformFromQuaternion(pQua);
  EXPECT_TRUE(pT.isNear(AL::Math::Transform::fromRotY(-90.0f*AL::Math::TO_RAD), 0.0001f));

//  std::cout << "transformFromQuaternion: -90 deg turn around Z axis" << std::endl;
  pT   = AL::Math::Transform();
  pQua = AL::Math::Quaternion(sqrtf(0.5f), 0.0f, 0.0f, -sqrtf(0.5f));
  pT   = AL::Math::transformFromQuaternion(pQua);
  EXPECT_TRUE(pT.isNear(AL::Math::Transform::fromRotZ(-90.0f*AL::Math::TO_RAD), 0.0001f));
}

TEST(ALTransformHelpersTest, quaternionFromTransform)
{
  AL::Math::Quaternion pQua;
  AL::Math::Transform  pT;
  //std::cout << "quaternionFromTransform: identity quaternion" << std::endl;
  pT   = AL::Math::Transform();
  pQua = AL::Math::Quaternion();
  pQua = AL::Math::quaternionFromTransform(pT);
  //std::cout << "Result  : " << pQua << std::endl;
  //std::cout << "Expected: " << AL::Math::Quaternion() << std::endl;
  EXPECT_TRUE(pQua.isNear(AL::Math::Quaternion(), 0.0001f));

  //std::cout << "quaternionFromTransform: 180 deg turn around X axis" << std::endl;
  pT   = AL::Math::Transform::fromRotX(180.0f*AL::Math::TO_RAD);
  pQua = AL::Math::Quaternion();
  pQua = AL::Math::quaternionFromTransform(pT);
  //std::cout << "Result  : " << pQua << std::endl;
  //std::cout << "Expected: " << AL::Math::Quaternion(0.0f, -1.0f, 0.0f, 0.0f) << std::endl;
  EXPECT_TRUE(pQua.isNear(AL::Math::Quaternion(0.0f, 1.0f, 0.0f, 0.0f), 0.0001f));

  //std::cout << "quaternionFromTransform: 180 deg turn around Y axis" << std::endl;
  pT   = AL::Math::Transform::fromRotY(180.0f*AL::Math::TO_RAD);
  pQua = AL::Math::Quaternion();
  pQua = AL::Math::quaternionFromTransform(pT);
  //std::cout << "Result  : " << pQua << std::endl;
  //std::cout << "Expected: " << AL::Math::Quaternion(0.0f, 0.0f, 1.0f, 0.0f) << std::endl;
  EXPECT_TRUE(pQua.isNear(AL::Math::Quaternion(0.0f, 0.0f, 1.0f, 0.0f), 0.0001f));

  //std::cout << "quaternionFromTransform: 180 deg turn around Z axis" << std::endl;
  pT   = AL::Math::Transform::fromRotZ(180.0f*AL::Math::TO_RAD);
  pQua = AL::Math::Quaternion();
  pQua = AL::Math::quaternionFromTransform(pT);
  //std::cout << "Result  : " << pQua << std::endl;
  //std::cout << "Expected: " << AL::Math::Quaternion(0.0f, 0.0f, 0.0f, 1.0f) << std::endl;
  EXPECT_TRUE(pQua.isNear(AL::Math::Quaternion(0.0f, 0.0f, 0.0f, 1.0f), 0.0001f));

  //std::cout << "quaternionFromTransform: 90 deg turn around X axis" << std::endl;
  pT   = AL::Math::Transform::fromRotX(90.0f*AL::Math::TO_RAD);
  pQua = AL::Math::Quaternion();
  pQua = AL::Math::quaternionFromTransform(pT);
  //std::cout << "Result  : " << pQua << std::endl;
  //std::cout << "Expected: " << AL::Math::Quaternion(sqrtf(0.5f), sqrtf(0.5f), 0.0f, 0.0f) << std::endl;
  EXPECT_TRUE(pQua.isNear(AL::Math::Quaternion(sqrtf(0.5f), sqrtf(0.5f), 0.0f, 0.0f), 0.0001f));

  //std::cout << "quaternionFromTransform: 90 deg turn around Y axis" << std::endl;
  pT   = AL::Math::Transform::fromRotY(90.0f*AL::Math::TO_RAD);
  pQua = AL::Math::Quaternion();
  pQua = AL::Math::quaternionFromTransform(pT);
  //std::cout << "Result  : " << pQua << std::endl;
  //std::cout << "Expected: " << AL::Math::Quaternion(sqrtf(0.5f), 0.0f, sqrtf(0.5f), 0.0f) << std::endl;
  EXPECT_TRUE(pQua.isNear(AL::Math::Quaternion(sqrtf(0.5f), 0.0f, sqrtf(0.5f), 0.0f), 0.0001f));

  //std::cout << "quaternionFromTransform: 90 deg turn around Z axis" << std::endl;
  pT   = AL::Math::Transform::fromRotZ(90.0f*AL::Math::TO_RAD);
  pQua = AL::Math::Quaternion();
  pQua = AL::Math::quaternionFromTransform(pT);
  //std::cout << "Result  : " << pQua << std::endl;
  //std::cout << "Expected: " << AL::Math::Quaternion(sqrtf(0.5f), 0.0f, 0.0f, sqrtf(0.5f)) << std::endl;
  EXPECT_TRUE(pQua.isNear(AL::Math::Quaternion(sqrtf(0.5f), 0.0f, 0.0f, sqrtf(0.5f)), 0.0001f));

  //std::cout << "quaternionFromTransform: -90 deg turn around X axis" << std::endl;
  pT   = AL::Math::Transform::fromRotX(-90.0f*AL::Math::TO_RAD);
  pQua = AL::Math::Quaternion();
  pQua = AL::Math::quaternionFromTransform(pT);
  //std::cout << "Result  : " << pQua << std::endl;
  //std::cout << "Expected: " << AL::Math::Quaternion(sqrtf(0.5f), -sqrtf(0.5f), 0.0f, 0.0f) << std::endl;
  EXPECT_TRUE(pQua.isNear(AL::Math::Quaternion(sqrtf(0.5f), -sqrtf(0.5f), 0.0f, 0.0f), 0.0001f));

  //std::cout << "quaternionFromTransform: -90 deg turn around Y axis" << std::endl;
  pT   = AL::Math::Transform::fromRotY(-90.0f*AL::Math::TO_RAD);
  pQua = AL::Math::Quaternion();
  pQua = AL::Math::quaternionFromTransform(pT);
  //std::cout << "Result  : " << pQua << std::endl;
  //std::cout << "Expected: " << AL::Math::Quaternion(sqrtf(0.5f), 0.0f, -sqrtf(0.5f), 0.0f) << std::endl;
  EXPECT_TRUE(pQua.isNear(AL::Math::Quaternion(sqrtf(0.5f), 0.0f, -sqrtf(0.5f), 0.0f), 0.0001f));

  //std::cout << "quaternionFromTransform: -90 deg turn around Z axis" << std::endl;
  pT   = AL::Math::Transform::fromRotZ(-90.0f*AL::Math::TO_RAD);
  pQua = AL::Math::Quaternion();
  pQua = AL::Math::quaternionFromTransform(pT);
  //std::cout << "Result  : " << pQua << std::endl;
  //std::cout << "Expected: " << AL::Math::Quaternion(sqrtf(0.5f), 0.0f, 0.0f, -sqrtf(0.5f)) << std::endl;
  EXPECT_TRUE(pQua.isNear(AL::Math::Quaternion(sqrtf(0.5f), 0.0f, 0.0f, -sqrtf(0.5f)), 0.0001f));

  // close to identity
  for (unsigned int i=0; i<150; i++)
  {
    float angle = 0.00001f/(static_cast<float>(i+1));

    pT   = AL::Math::Transform::fromRotX(angle);
    pQua = AL::Math::quaternionFromTransform(pT);
    //std::cout << "i: " << i << " angle: " << angle << " quaternion: " << pQua << std::endl;
    EXPECT_TRUE(pQua.isNear(AL::Math::Quaternion(), 0.0001f));
  }
}

TEST(ALTransformHelpersTest, quaternionVsTransform)
{
  AL::Math::Quaternion pQua;
  AL::Math::Transform  pTIn;
  AL::Math::Transform  pTOut;

  unsigned int nbX = 100;
  unsigned int nbY = 100;
  unsigned int nbZ = 100;

  for (unsigned int i=0; i<nbX; i++)
  {
    for (unsigned int j=0; j<nbY; j++)
    {
      for (unsigned int k=0; k<nbZ; k++)
      {
        float angleX = static_cast<float>(i)/(static_cast<float>(nbX))*AL::Math::_2_PI_;
        float angleY = static_cast<float>(j)/(static_cast<float>(nbY))*AL::Math::_2_PI_;
        float angleZ = static_cast<float>(k)/(static_cast<float>(nbZ))*AL::Math::_2_PI_;

        pTIn = AL::Math::Transform::fromRotX(angleX)*
            AL::Math::Transform::fromRotY(angleY)*
            AL::Math::Transform::fromRotZ(angleZ);

        pQua  = AL::Math::quaternionFromTransform(pTIn);
        pTOut = AL::Math::transformFromQuaternion(pQua);

         if (!pTIn.isNear(pTOut, 0.0005f))
         {
           std::cout << "[angleX, angleY, angleZ, i, j, k]: "
                     << angleX*AL::Math::TO_DEG << " "
                     << angleY*AL::Math::TO_DEG << " "
                     << angleZ*AL::Math::TO_DEG << " "
                     << i << " " << j << " " << k << std::endl;
           std::cout << std::endl;

           std::cout << "pQua : " << pQua.norm()         << std::endl << pQua  << std::endl;
           std::cout << "pTIn : " << pTIn.determinant()  << std::endl << pTIn  << std::endl;
           std::cout << "pTOut: " << pTOut.determinant() << std::endl << pTOut << std::endl;

           pQua  = AL::Math::quaternionFromTransform(pTIn);
         }

        EXPECT_TRUE(pTIn.isNear(pTOut, 0.0005f));

      }
    }
  }

//  std::cout << "Result  : " << pQua << std::endl;
//  std::cout << "Expected: " << AL::Math::Quaternion() << std::endl;
}

TEST(ALMathTransformHelpers, displacementFromTransform)
{
  AL::Math::Position3D translation1(1.f, 2.f, 3.f);
  AL::Math::Position3D translation2(-1.f, -2.f, -3.f);
  AL::Math::Position3D translation3(1.1f, 1.2f, 1.3f);

  AL::Math::Rotation rotation1 =
    AL::Math::Rotation::fromAngleDirection(AL::Math::PI_2, 1.f, 0.f,  0.f);
   AL::Math::Rotation rotation2 =
    AL::Math::Rotation::fromAngleDirection(-AL::Math::PI_2, 0.f, 0.f,  1.f);
  AL::Math::Rotation rotation3 =
    AL::Math::Rotation::fromAngleDirection(AL::Math::PI, 0.f, 1.f,  0.f);

  AL::Math::Transform transform1 = AL::Math::transformFromRotation(rotation1);
  transform1.r1_c4 = translation1.x;
  transform1.r2_c4 = translation1.y;
  transform1.r3_c4 = translation1.z;
  AL::Math::Transform transform2 = AL::Math::transformFromRotation(rotation2);
  transform2.r1_c4 = translation2.x;
  transform2.r2_c4 = translation2.y;
  transform2.r3_c4 = translation2.z;
  AL::Math::Transform transform3 = AL::Math::transformFromRotation(rotation3);
  transform3.r1_c4 = translation3.x;
  transform3.r2_c4 = translation3.y;
  transform3.r3_c4 = translation3.z;

  AL::Math::Displacement resDisp1 = displacementFromTransform(transform1);
  AL::Math::Displacement resDisp2 = displacementFromTransform(transform2);
  AL::Math::Displacement resDisp3 = displacementFromTransform(transform3);

  AL::Math::Quaternion expQuat1(sqrtf(0.5f), sqrtf(.5f), 0.f, 0.f);
  AL::Math::Quaternion expQuat2(sqrtf(0.5f), 0.f, 0.f, -sqrtf(0.5f));
  AL::Math::Quaternion expQuat3(0.f, 0.f, 1.f, 0.f);

  AL::Math::Displacement expDisp1(translation1, expQuat1);
  AL::Math::Displacement expDisp2(translation2, expQuat2);
  AL::Math::Displacement expDisp3(translation3, expQuat3);

  ASSERT_TRUE(resDisp1.isNear(expDisp1));
  ASSERT_TRUE(resDisp2.isNear(expDisp2));
  ASSERT_TRUE(resDisp3.isNear(expDisp3));
}

TEST(ALMathTransformHelpers, transformFromDisplacement)
{
  AL::Math::Position3D translation1(1.f, 2.f, 3.f);
  AL::Math::Position3D translation2(-1.f, -2.f, -3.f);
  AL::Math::Position3D translation3(1.1f, 1.2f, 1.3f);

  AL::Math::Quaternion quaternion1(sqrtf(0.5f), sqrtf(.5f), 0.f, 0.f);
  AL::Math::Quaternion quaternion2(sqrtf(0.5f), 0.f, 0.f, -sqrtf(0.5f));
  AL::Math::Quaternion quaternion3(0.f, 0.f, 1.f, 0.f);

  AL::Math::Displacement disp1(translation1, quaternion1);
  AL::Math::Displacement disp2(translation2, quaternion2);
  AL::Math::Displacement disp3(translation3, quaternion3);

  AL::Math::Transform resTrans1 = transformFromDisplacement(disp1);
  AL::Math::Transform resTrans2 = transformFromDisplacement(disp2);
  AL::Math::Transform resTrans3 = transformFromDisplacement(disp3);

  AL::Math::Rotation expRot1 =
    AL::Math::Rotation::fromAngleDirection(AL::Math::PI_2, 1.f, 0.f,  0.f);
   AL::Math::Rotation expRot2 =
    AL::Math::Rotation::fromAngleDirection(-AL::Math::PI_2, 0.f, 0.f,  1.f);
  AL::Math::Rotation expRot3 =
    AL::Math::Rotation::fromAngleDirection(AL::Math::PI, 0.f, 1.f,  0.f);

  AL::Math::Transform expTrans1 = AL::Math::transformFromRotation(expRot1);
  expTrans1.r1_c4 = translation1.x;
  expTrans1.r2_c4 = translation1.y;
  expTrans1.r3_c4 = translation1.z;
  AL::Math::Transform expTrans2 = AL::Math::transformFromRotation(expRot2);
  expTrans2.r1_c4 = translation2.x;
  expTrans2.r2_c4 = translation2.y;
  expTrans2.r3_c4 = translation2.z;
  AL::Math::Transform expTrans3 = AL::Math::transformFromRotation(expRot3);
  expTrans3.r1_c4 = translation3.x;
  expTrans3.r2_c4 = translation3.y;
  expTrans3.r3_c4 = translation3.z;

  ASSERT_TRUE(resTrans1.isNear(expTrans1));
  ASSERT_TRUE(resTrans2.isNear(expTrans2));
  ASSERT_TRUE(resTrans3.isNear(expTrans3));
}
