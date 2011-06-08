/**
 * @author Cyrille Collette - ccollette@aldebaran-robotics.com
 * Aldebaran Robotics (c) 2009 All Rights Reserved
 *
 */
#include <almath/types/alposition2d.h>
#include "../almathtestutils.h"

AL::Math::Position2D pPos2D1 = AL::Math::Position2D();
AL::Math::Position2D pPos2D2 = AL::Math::Position2D();

TEST(ALPosition2DTest, Distance)
{
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
  //pPos2D1 = AL::Math::Position2D(0.0f, 0.0f);
  //ASSERT_THROW(AL::Math::normalize(pPos2D1), AL::ALError);


  //std::cout << "-------------- normalize 1 --------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(0.5f, 0.0f);
  comparePosition2D(AL::Math::normalize(pPos2D1), AL::Math::Position2D(1.0f, 0.0f));


  //std::cout << "-------------- normalize 2 --------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(1.0f, -1.0f);
  comparePosition2D(AL::Math::normalize(pPos2D1), AL::Math::Position2D(0.70710678118655f, -0.70710678118655f));
}


TEST(ALPosition2DTest, crossProduct)
{
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
  comparePosition2D((pPos2D2-pPos2D1), AL::Math::Position2D(-1.7f, -1.1f));

  //std::cout << "-------------- soustraction 0 (-a) --------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(1.2f, -1.3f);
  pPos2D2 = AL::Math::Position2D();
  pPos2D2 = - pPos2D1;
  comparePosition2D(pPos2D2, AL::Math::Position2D(-1.2f, 1.3f));


  //std::cout << "-------------- soustraction 0 (-=a)--------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(1.2f, -1.5f);
  pPos2D2 = AL::Math::Position2D(-1.2f, +1.5f);
  pPos2D2 -= pPos2D1;
  comparePosition2D(pPos2D2, AL::Math::Position2D(-2.4f, +3.0f));


  //std::cout << "-------------- addition 0 (a+b)--------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(+1.2f, 1.3f);
  pPos2D2 = AL::Math::Position2D(-0.5f, 0.2f);
  comparePosition2D((pPos2D1+pPos2D2), AL::Math::Position2D(0.7f, 1.5f));


  //std::cout << "-------------- addition 0 (+a)--------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(1.2f, -1.5f);
  pPos2D2 = AL::Math::Position2D();
  pPos2D2 = + pPos2D1;
  comparePosition2D(pPos2D2, AL::Math::Position2D(1.2f, -1.5f));


  //std::cout << "-------------- addition 0 (+=a)--------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(1.2f, -1.5f);
  pPos2D2 = AL::Math::Position2D(1.2f, -1.5f);
  pPos2D2 += pPos2D1;
  comparePosition2D(pPos2D2, AL::Math::Position2D(2.4f, -3.0f));


  //std::cout << "-------------- multiplication 0 --------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(1.0f, 1.0f);
  comparePosition2D((pPos2D1*2.0f), AL::Math::Position2D(2.0f, 2.0f));


  //std::cout << "-------------- multiplication 1 --------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(1.0f, 1.0f);
  comparePosition2D((2.0f*pPos2D1), AL::Math::Position2D(2.0f, 2.0f));


  //std::cout << "-------------- division 0 --------------" << std::endl;
  pPos2D1 = AL::Math::Position2D(2.0f, 2.0f);
  comparePosition2D((pPos2D1/2.0f), AL::Math::Position2D(1.0f, 1.0f));


  //std::cout << "-------------- division 1 --------------" << std::endl;
  //pPos2D1 = AL::Math::Position2D(2.0f, 2.0f);
  //ASSERT_THROW((pPos2D1/0.0f), AL::ALError);
}

