/**
 * @author Cyrille Collette - ccollette@aldebaran-robotics.com
 * Aldebaran Robotics (c) 2010 All Rights Reserved
 *
 */
#include <almath/types/alvelocity3d.h>
#include "../almathtestutils.h"


TEST(ALVelocity3DTest, variousOperator)
{
  //Velocity3D operator+ (a=b+c) (const Velocity3D& pVel2) const;
  AL::Math::Velocity3D pVIn1 = AL::Math::Velocity3D(0.5f, 0.3f, 0.1f);
  AL::Math::Velocity3D pVIn2 = AL::Math::Velocity3D(1.5f, 1.3f, 1.1f);
  AL::Math::Velocity3D pVIn3 = AL::Math::Velocity3D();
  pVIn3 = pVIn1 + pVIn2;

  AL::Math::Velocity3D pVOut = AL::Math::Velocity3D(2.0f, 1.6f, 1.2f);
  compareVelocity3D(pVIn3, pVOut, 0.0001f);

  //Velocity3D operator+ (a = +b) (const Velocity3D& pVel2) const;
  pVIn1 = AL::Math::Velocity3D(0.5f, -0.3f, 0.1f);
  pVIn3 = AL::Math::Velocity3D();
  pVIn3 = +pVIn1;

  pVOut = AL::Math::Velocity3D(0.5f, -0.3f, 0.1f);
  compareVelocity3D(pVIn3, pVOut, 0.0001f);

  //Velocity3D& operator+= (const Velocity3D& pVel2);
  pVIn1 = AL::Math::Velocity3D(0.5f, 0.3f, 0.1f);
  pVIn2 = AL::Math::Velocity3D(1.5f, 1.3f, 1.1f);
  pVIn1 += pVIn2;
  pVOut = AL::Math::Velocity3D(2.0f, 1.6f, 1.2f);
  compareVelocity3D(pVIn1, pVOut, 0.0001f);


  //Velocity3D operator- (a = b-c) (const Velocity3D& pVel2) const;
  pVIn1 = AL::Math::Velocity3D(0.5f, 0.3f, 0.1f);
  pVIn2 = AL::Math::Velocity3D(1.5f, 1.3f, 1.1f);
  pVIn3 = AL::Math::Velocity3D();
  pVIn3 = pVIn2 - pVIn1;
  pVOut = AL::Math::Velocity3D(1.0f, 1.0f, 1.0f);
  compareVelocity3D(pVIn3, pVOut, 0.0001f);


  //Velocity3D operator- (a = -b) (const Velocity3D& pVel2) const;
  pVIn1 = AL::Math::Velocity3D(0.5f, 0.3f, 0.1f);
  pVIn3 = AL::Math::Velocity3D();
  pVIn3 = -pVIn1;
  pVOut = AL::Math::Velocity3D(-0.5f, -0.3f, -0.1f);
  compareVelocity3D(pVIn3, pVOut, 0.0001f);

  //Velocity3D& operator-= (const Velocity3D& pVel2);
  pVIn1 = AL::Math::Velocity3D(0.5f, 0.3f, 0.1f);
  pVIn2 = AL::Math::Velocity3D(1.5f, 1.3f, 1.1f);
  pVIn1 -= pVIn2;
  pVOut = AL::Math::Velocity3D(-1.0f, -1.0f, -1.0f);
  compareVelocity3D(pVIn1, pVOut, 0.0001f);

  //Velocity3D operator* (const float pM) const;
  float K = 10.0f;
  pVIn1 = AL::Math::Velocity3D(0.5f, 0.3f, 0.1f);
  pVIn2 = AL::Math::Velocity3D();
  pVIn2 = pVIn1*K;
  pVOut = AL::Math::Velocity3D(5.0f, 3.0f, 1.0f);
  compareVelocity3D(pVIn2, pVOut, 0.0001f);

  //Velocity3D operator/ (const float pM) const;
  K = 10.0f;
  pVIn1 = AL::Math::Velocity3D(5.0f, 3.0f, 1.0f);
  pVIn2 = AL::Math::Velocity3D();
  pVIn2 = pVIn1/K;
  pVOut = AL::Math::Velocity3D(0.5f, 0.3f, 0.1f);
  compareVelocity3D(pVIn2, pVOut, 0.0001f);

  pVIn1 = AL::Math::Velocity3D(5.0f, 3.0f, 1.0f);
  ASSERT_THROW(pVIn1/0.0f, AL::ALError);

  //Velocity3D& operator*= (const float pM);
  K = 10.0f;
  pVIn1 = AL::Math::Velocity3D(0.5f, 0.3f, 0.1f);
  pVIn1 *= K;
  pVOut = AL::Math::Velocity3D(5.0f, 3.0f, 1.0f);
  compareVelocity3D(pVIn1, pVOut, 0.0001f);

  //Velocity3D& operator/= (const float pM);
  K = 10.0f;
  pVIn1 = AL::Math::Velocity3D(5.0f, 3.0f, 1.0f);
  pVIn1 /= K;
  pVOut = AL::Math::Velocity3D(0.5f, 0.3f, 0.1f);
  compareVelocity3D(pVIn1, pVOut, 0.0001f);

  pVIn1 = AL::Math::Velocity3D(5.0f, 3.0f, 1.0f);
  ASSERT_THROW(pVIn1/=0.0f, AL::ALError);
}

TEST(ALVelocity3DTest, isNear)
{
  AL::Math::Velocity3D pV = AL::Math::Velocity3D();

//bool isNear(
//            const Velocity3D& pVel,
//            const float&      pEpsilon=0.0001f) const;
}


//float norm () const;
TEST(ALVelocity3DTest, norm)
{
  AL::Math::Velocity3D pVel3D1 = AL::Math::Velocity3D();
  EXPECT_NEAR(AL::Math::norm(pVel3D1), 0.0f, kEpsilon);

  pVel3D1 = AL::Math::Velocity3D(1.0f, 0.0f, 0.0f);
  EXPECT_NEAR(AL::Math::norm(pVel3D1), 1.0f, kEpsilon);
}

//Velocity3D normalize() const;
TEST(ALVelocity3DTest, normalize)
{
  AL::Math::Velocity3D pVel3D1 = AL::Math::Velocity3D(0.5f, 0.0f, 0.0f);
  compareVelocity3D(AL::Math::normalize(pVel3D1), AL::Math::Velocity3D(1.0f, 0.0f, 0.0f));

  pVel3D1 = AL::Math::Velocity3D(1.0f, -1.0f, 0.0f);
  compareVelocity3D(AL::Math::normalize(pVel3D1), AL::Math::Velocity3D(0.70710678118655f, -0.70710678118655f, 0.0f));

  pVel3D1 = AL::Math::Velocity3D(0.0f, 0.0f, 0.0f);
  ASSERT_THROW(AL::Math::normalize(pVel3D1), AL::ALError);
}


TEST(ALVelocity3DTest, Divers)
{
  AL::Math::Velocity3D pVel3D1 = AL::Math::Velocity3D(+1.2f, 1.3f, 0.2f);
  AL::Math::Velocity3D pVel3D2 = AL::Math::Velocity3D(-0.5f, 0.2f, 0.4f);
  compareVelocity3D((pVel3D2-pVel3D1), AL::Math::Velocity3D(-1.7f, -1.1f, 0.2f));

  pVel3D1 = AL::Math::Velocity3D(+1.2f, 1.3f, 0.2f);
  pVel3D2 = AL::Math::Velocity3D(-0.5f, 0.2f, 0.4f);
  compareVelocity3D((pVel3D1+pVel3D2), AL::Math::Velocity3D(0.7f, 1.5f, 0.6f));

  pVel3D1 = AL::Math::Velocity3D(1.0f, 1.0f, 1.0f);
  compareVelocity3D((pVel3D1*2.0f), AL::Math::Velocity3D(2.0f, 2.0f, 2.0f));

  pVel3D1 = AL::Math::Velocity3D(1.0f, 1.0f, 1.0f);
  compareVelocity3D((2.0f*pVel3D1), AL::Math::Velocity3D(2.0f, 2.0f, 2.0f));

  pVel3D1 = AL::Math::Velocity3D(2.0f, 2.0f, 2.0f);
  compareVelocity3D((pVel3D1/2.0f), AL::Math::Velocity3D(1.0f, 1.0f, 1.0f));

  pVel3D1 = AL::Math::Velocity3D(2.0f, 2.0f, 2.0f);
  ASSERT_THROW((pVel3D1/0.0f), AL::ALError);
}
