/**
 * @author Cyrille Collette - ccollette@aldebaran-robotics.com
 * Aldebaran Robotics (c) 2009 All Rights Reserved
 *
 */
#include <almath/types/alposition3d.h>
#include "../almathtestutils.h"

#include <stdexcept>

AL::Math::Position3D pPos3D1 = AL::Math::Position3D();
AL::Math::Position3D pPos3D2 = AL::Math::Position3D();

TEST(ALPosition3DTest, Distance)
{
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
  //std::cout << "-------------- normalize 0 --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(0.0f, 0.0f, 0.0f);
  ASSERT_THROW(AL::Math::normalize(pPos3D1), std::runtime_error);


  //std::cout << "-------------- normalize 1 --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(0.5f, 0.0f, 0.0f);
  comparePosition3D(AL::Math::normalize(pPos3D1), AL::Math::Position3D(1.0f, 0.0f, 0.0f));


  //std::cout << "-------------- normalize 2 --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(1.0f, -1.0f, 0.0f);
  comparePosition3D(AL::Math::normalize(pPos3D1), AL::Math::Position3D(0.70710678118655f, -0.70710678118655f, 0.0f));
}


TEST(ALPosition3DTest, crossProduct)
{
  //std::cout << "-------------- crossProduct 0 --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D();
  pPos3D2 = AL::Math::Position3D();
  comparePosition3D(AL::Math::crossProduct(pPos3D1, pPos3D2), AL::Math::Position3D());


  //std::cout << "-------------- crossProduct 1 --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(+1.0f, 2.0f, 1.0f);
  pPos3D2 = AL::Math::Position3D(-0.5f, 0.2f, 0.2f);
  comparePosition3D(AL::Math::crossProduct(pPos3D1, pPos3D2), AL::Math::Position3D(0.2f, -0.7f, 1.2f));
}


TEST(ALPosition3DTest, Divers)
{
  //std::cout << "-------------- soustraction 0 (-a) --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(+1.2f, -1.3f, 0.2f);
  pPos3D2 = -pPos3D1;
  comparePosition3D(pPos3D2, AL::Math::Position3D(-1.2f, +1.3f, -0.2f));

  //std::cout << "-------------- soustraction 0 (a-b) --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(+1.2f, 1.3f, 0.2f);
  pPos3D2 = AL::Math::Position3D(-0.5f, 0.2f, 0.4f);
  comparePosition3D((pPos3D2-pPos3D1), AL::Math::Position3D(-1.7f, -1.1f, 0.2f));

  //std::cout << "-------------- soustraction 0 (a-=b) --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(-1.2f, +1.3f, -0.2f);
  pPos3D2 = AL::Math::Position3D(+1.2f, -1.3f, 0.2f);
  pPos3D2 -= pPos3D1;
  comparePosition3D(pPos3D2, AL::Math::Position3D(2.4f, -2.6f, 0.4f));

  //std::cout << "-------------- addition 0 (+a) --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(+1.2f, 1.3f, 0.3f);
  pPos3D2 = +pPos3D1;
  comparePosition3D(pPos3D2, AL::Math::Position3D(1.2f, 1.3f, 0.3f));

  //std::cout << "-------------- addition 0 (a+b) --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(+1.2f, 1.3f, 0.3f);
  pPos3D2 = AL::Math::Position3D(-0.5f, 0.2f, 0.2f);
  comparePosition3D((pPos3D1+pPos3D2), AL::Math::Position3D(0.7f, 1.5f, 0.5f));


  //std::cout << "-------------- addition 0 (a+=b) --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(+1.5f, 1.3f, 0.3f);
  pPos3D2 = AL::Math::Position3D(-0.5f, 0.2f, 0.2f);
  pPos3D2 += pPos3D1;
  comparePosition3D(pPos3D2, AL::Math::Position3D(1.0f, 1.5f, 0.5f));


  //std::cout << "-------------- multiplication 0 --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(1.0f, 1.0f, 1.0f);
  comparePosition3D((pPos3D1*2.0f), AL::Math::Position3D(2.0f, 2.0f, 2.0f));


  //std::cout << "-------------- multiplication 1 --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(1.0f, 1.0f, 1.0f);
  comparePosition3D((2.0f*pPos3D1), AL::Math::Position3D(2.0f, 2.0f, 2.0f));


  //std::cout << "-------------- division 0 --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(2.0f, 2.0f, 2.0f);
  comparePosition3D((pPos3D1/2.0f), AL::Math::Position3D(1.0f, 1.0f, 1.0f));


  //std::cout << "-------------- division 1 --------------" << std::endl;
  pPos3D1 = AL::Math::Position3D(2.0f, 2.0f, 2.0f);
  ASSERT_THROW((pPos3D1/0.0f), std::runtime_error);
}

