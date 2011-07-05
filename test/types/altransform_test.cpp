/**
* @author Cyrille Collette - ccollette@aldebaran-robotics.com
* Aldebaran Robotics (c) 2009 All Rights Reserved
*
*/
#include <almath/types/altransform.h>
#include <almath/tools/almathio.h>
#include "../almathtestutils.h"

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
  compareTransform(pHIn1, pHSol, 0.0001f);

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

  compareTransform(pHIn3, pHSol, 0.0001f);
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
  EXPECT_NEAR(pH.norm(), sqrtf(3.0f), 0.0001f);
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

  compareTransform(pHIn.inverse(), pHSol, 0.0001f);
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

  compareTransform(pHIn, pHOut, 0.0001f);

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

  compareTransform(pHIn, pHOut, 0.0001f);

  // fromRotZ
  pHIn  = AL::Math::Transform::fromRotZ(0.5f);
  pHOut = AL::Math::Transform();
  pHOut.r1_c1 = 0.87758256189037f;
  pHOut.r1_c2 = -0.47942553860420f;
  pHOut.r2_c1 = 0.47942553860420f;
  pHOut.r2_c2 = 0.87758256189037f;
  compareTransform(pHIn, pHOut, 0.0001f);
}

TEST(ALTransformHelpersTest, fromRotXYZ2)
{
  // ************ Position6DFromTransform ************
  std::vector<float> pAngleList;
  AL::Math::Transform pHIn  = AL::Math::Transform();
  AL::Math::Transform pHOut = AL::Math::Transform();

  pAngleList.resize(11);
  pAngleList[0]  =     0.0f;
  pAngleList[1]  =   +10.0f*TO_RAD;
  pAngleList[2]  =   -10.0f*TO_RAD;
  pAngleList[3]  =  +90.00f*TO_RAD;
  pAngleList[4]  =  -90.00f*TO_RAD;
  pAngleList[5]  = +150.00f*TO_RAD;
  pAngleList[6]  = -150.00f*TO_RAD;
  pAngleList[7]  = +180.00f*TO_RAD;
  pAngleList[8]  = -180.00f*TO_RAD;
  pAngleList[9]  = +210.00f*TO_RAD;
  pAngleList[10] = -210.00f*TO_RAD;

  for (unsigned int i=0; i<pAngleList.size(); i++)
  {
    pHIn  = AL::Math::transformFromRotX(pAngleList[i]);
    pHOut = AL::Math::transformFromPosition6D(AL::Math::position6DFromTransform(pHIn));
    compareTransform(pHIn, pHOut);

    pHIn  = AL::Math::transformFromRotY(pAngleList[i]);
    pHOut = AL::Math::transformFromPosition6D(AL::Math::position6DFromTransform(pHIn));
    compareTransform(pHIn, pHOut);

    pHIn  = AL::Math::transformFromRotZ(pAngleList[i]);
    pHOut = AL::Math::transformFromPosition6D(AL::Math::position6DFromTransform(pHIn));
    compareTransform(pHIn, pHOut);
  }

  std::vector<std::vector<float> > pList;
  pList.resize(5);

  pAngleList.resize(3);

  // 0
  pAngleList[0] =   10.0f*TO_RAD;
  pAngleList[1] = -100.0f*TO_RAD;
  pAngleList[2] =   40.0f*TO_RAD;

  pList[0] = pAngleList;

  // 1
  pAngleList[0] = -180.0f*TO_RAD;
  pAngleList[1] = +150.0f*TO_RAD;
  pAngleList[2] =  270.0f*TO_RAD;

  pList[1] = pAngleList;

  // 2
  pAngleList[0] = -260.0f*TO_RAD;
  pAngleList[1] =  240.0f*TO_RAD;
  pAngleList[2] =  290.0f*TO_RAD;

  pList[2] = pAngleList;

  // 3
  pAngleList[0] = -10.0f*TO_RAD;
  pAngleList[1] = -10.0f*TO_RAD;
  pAngleList[2] = -10.0f*TO_RAD;

  pList[3] = pAngleList;

  // 4
  pAngleList[0] = +180.0f*TO_RAD;
  pAngleList[1] =  -90.0f*TO_RAD;
  pAngleList[2] = +180.0f*TO_RAD;

  pList[4] = pAngleList;

  for (unsigned int i=0; i<pList.size(); i++)
  {
    pHIn  = AL::Math::transformFromRotX(pList[i][0])*
        AL::Math::transformFromRotY(pList[i][1])*
        AL::Math::transformFromRotZ(pList[i][2]);
    pHOut = AL::Math::transformFromPosition6D(AL::Math::position6DFromTransform(pHIn));
    compareTransform(pHIn, pHOut);

    pHIn  = AL::Math::transformFromRotX(pList[i][2])*
        AL::Math::transformFromRotY(pList[i][0])*
        AL::Math::transformFromRotZ(pList[i][1]);
    pHOut = AL::Math::transformFromPosition6D(AL::Math::position6DFromTransform(pHIn));
    compareTransform(pHIn, pHOut);

    pHIn  = AL::Math::transformFromRotX(pList[i][1])*
        AL::Math::transformFromRotY(pList[i][2])*
        AL::Math::transformFromRotZ(pList[i][0]);
    pHOut = AL::Math::transformFromPosition6D(AL::Math::position6DFromTransform(pHIn));
    compareTransform(pHIn, pHOut);
  }

} // end transformFromRot


TEST(TransformTest, from3DRotation)
{
  AL::Math::Transform pHIn  = AL::Math::Transform::from3DRotation(0.0f, 0.0f, 0.0f);
  AL::Math::Transform pHOut = AL::Math::Transform();
  compareTransform(pHIn, pHOut, 0.00001f);

  // fromRotX
  pHIn  = AL::Math::Transform::from3DRotation(0.33f, 0.0f, 0.0f);
  pHOut = AL::Math::Transform::fromRotX(0.33f);
  compareTransform(pHIn, pHOut, 0.00001f);

  // fromRotY
  pHIn  = AL::Math::Transform::from3DRotation(0.0f, 0.33f, 0.0f);
  pHOut = AL::Math::Transform::fromRotY(0.33f);
  compareTransform(pHIn, pHOut, 0.00001f);

  // fromRotZ
  pHIn  = AL::Math::Transform::from3DRotation(0.0f, 0.0f, 0.33f);
  pHOut = AL::Math::Transform::fromRotZ(0.33f);
  compareTransform(pHIn, pHOut, 0.00001f);
}


TEST(TransformTest, fromPosition)
{
  AL::Math::Transform pHIn  = AL::Math::Transform::fromPosition(1.0f, 0.2f, 0.1f);
  AL::Math::Transform pHOut = AL::Math::Transform();
  pHOut.r1_c4 = 1.0f;
  pHOut.r2_c4 = 0.2f;
  pHOut.r3_c4 = 0.1f;
  compareTransform(pHIn, pHOut, 0.00001f);
}


TEST(TransformTest, fromPositionAndRotation)
{
  AL::Math::Transform pHIn  = AL::Math::Transform::fromPosition(1.0f, 0.2f, 0.1f, 0.0f, 0.0f, 0.0f);
  AL::Math::Transform pHOut = AL::Math::Transform();
  pHOut.r1_c4 = 1.0f;
  pHOut.r2_c4 = 0.2f;
  pHOut.r3_c4 = 0.1f;
  compareTransform(pHIn, pHOut, 0.00001f);

  pHIn  = AL::Math::Transform::fromPosition(1.0f, 0.2f, 0.1f, 0.2f, 0.0f, 0.0f);
  pHOut = AL::Math::Transform::fromRotX(0.2f);
  pHOut.r1_c4 = 1.0f;
  pHOut.r2_c4 = 0.2f;
  pHOut.r3_c4 = 0.1f;
  compareTransform(pHIn, pHOut, 0.00001f);

  pHIn  = AL::Math::Transform::fromPosition(1.0f, 0.2f, 0.1f, 0.0f, 0.2f, 0.0f);
  pHOut = AL::Math::Transform::fromRotY(0.2f);
  pHOut.r1_c4 = 1.0f;
  pHOut.r2_c4 = 0.2f;
  pHOut.r3_c4 = 0.1f;
  compareTransform(pHIn, pHOut, 0.00001f);

  pHIn  = AL::Math::Transform::fromPosition(1.0f, 0.2f, 0.1f, 0.0f, 0.0f, 0.2f);
  pHOut = AL::Math::Transform::fromRotZ(0.2f);
  pHOut.r1_c4 = 1.0f;
  pHOut.r2_c4 = 0.2f;
  pHOut.r3_c4 = 0.1f;
  compareTransform(pHIn, pHOut, 0.00001f);
}


TEST(TransformTest, diff)
{
  AL::Math::Transform pHIn1 = AL::Math::Transform::fromRotX(0.2f);
  AL::Math::Transform pHIn2 = AL::Math::Transform::fromRotX(-0.3f);
  AL::Math::Transform pHOut = AL::Math::Transform::fromRotX(-0.5f);
  compareTransform(pHIn1.diff(pHIn2), pHOut, 0.0001f);

  pHIn1 = AL::Math::Transform::fromRotY(0.2f);
  pHIn2 = AL::Math::Transform::fromRotY(-0.3f);
  pHOut = AL::Math::Transform::fromRotY(-0.5f);
  compareTransform(pHIn1.diff(pHIn2), pHOut, 0.0001f);

  pHIn1 = AL::Math::Transform::fromRotZ(0.2f);
  pHIn2 = AL::Math::Transform::fromRotZ(-0.3f);
  pHOut = AL::Math::Transform::fromRotZ(-0.5f);
  compareTransform(pHIn1.diff(pHIn2), pHOut, 0.0001f);
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
  EXPECT_NEAR(pHIn1.distance(pHIn2), sqrtf(3.0f), 0.0001f);
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
  compareTransform(AL::Math::transformInverse(AL::Math::Transform()), AL::Math::Transform());

  AL::Math::Transform pHIn  = AL::Math::Transform::fromRotX(0.5f);
  AL::Math::Transform pHOut = AL::Math::Transform::fromRotX(-0.5f);
  compareTransform(pHIn.inverse(), pHOut, 0.0001f);

  pHIn  = AL::Math::Transform::fromRotY(0.5f);
  pHOut = AL::Math::Transform::fromRotY(-0.5f);
  compareTransform(pHIn.inverse(), pHOut, 0.0001f);

  pHIn  = AL::Math::Transform::fromRotZ(0.5f);
  pHOut = AL::Math::Transform::fromRotZ(-0.5f);
  compareTransform(pHIn.inverse(), pHOut, 0.0001f);

  pHIn  = AL::Math::Transform::fromPosition(0.1f, 0.2f, 0.3f);
  pHOut = AL::Math::Transform::fromPosition(-0.1f, -0.2f, -0.3f);
  compareTransform(pHIn.inverse(), pHOut, 0.0001f);
}

