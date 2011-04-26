/**
 * @author Cyrille Collette - ccollette@aldebaran-robotics.com
 * Aldebaran Robotics (c) 2010 All Rights Reserved
 *
 */
#include <almath/types/alvelocity6d.h>
#include "../almathtestutils.h"

AL::Math::Velocity6D pVel6D1 = AL::Math::Velocity6D();
AL::Math::Velocity6D pVel6D2 = AL::Math::Velocity6D();

TEST(ALVelocity6DTest, norm)
{
  //std::cout << "-------------- norm 0 --------------" << std::endl;
  pVel6D1 = AL::Math::Velocity6D();
  EXPECT_NEAR(AL::Math::norm(pVel6D1), 0.0f, kEpsilon);


  //std::cout << "-------------- norm 1 --------------" << std::endl;
  pVel6D1 = AL::Math::Velocity6D(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  EXPECT_NEAR(AL::Math::norm(pVel6D1), 1.0f, kEpsilon);


  ////std::cout << "-------------- norm 2 --------------" << std::endl;
  //pVel6D1 = AL::Math::Velocity6D(1.0f, -0.8f, 0.2f, 0.4f, 0.3f, -0.3f);
  //EXPECT_NEAR(AL::Math::norm(pVel6D1), 0.0f, kEpsilon); // TODO
}


TEST(ALVelocity6DTest, normalize)
{
  //std::cout << "-------------- normalize 0 --------------" << std::endl;
  //pVel6D1 = AL::Math::Velocity6D(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  //ASSERT_THROW(AL::Math::normalize(pVel6D1), AL::ALError);


  //std::cout << "-------------- normalize 1 --------------" << std::endl;
  pVel6D1 = AL::Math::Velocity6D(0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  compareVelocity6D(AL::Math::normalize(pVel6D1), AL::Math::Velocity6D(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));


  //std::cout << "-------------- normalize 2 --------------" << std::endl;
  pVel6D1 = AL::Math::Velocity6D(1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  compareVelocity6D(AL::Math::normalize(pVel6D1), AL::Math::Velocity6D(0.70710678118655f, -0.70710678118655f, 0.0f, 0.0f, 0.0f, 0.0f));
}


TEST(ALVelocity6DTest, Divers)
{
  //std::cout << "-------------- soustraction 0 --------------" << std::endl;
  pVel6D1 = AL::Math::Velocity6D(+1.2f, 1.3f, 0.2f, +0.3f, 10.0f, -10.0f);
  pVel6D2 = AL::Math::Velocity6D(-0.5f, 0.2f, 0.4f, -0.5f,  0.2f,  +0.4f);
  compareVelocity6D((pVel6D2-pVel6D1), AL::Math::Velocity6D(-1.7f, -1.1f, 0.2f, -0.8f, -9.8f, +10.4f));


  //std::cout << "-------------- addition 0 --------------" << std::endl;
  pVel6D1 = AL::Math::Velocity6D(+1.2f, 1.3f, 0.3f, +1.2f, 1.3f, 0.3f);
  pVel6D2 = AL::Math::Velocity6D(-0.5f, 0.2f, 0.2f, -0.5f, 0.2f, 0.2f);
  compareVelocity6D((pVel6D1+pVel6D2), AL::Math::Velocity6D(0.7f, 1.5f, 0.5f, 0.7f, 1.5f, 0.5f));


  //std::cout << "-------------- multiplication 0 --------------" << std::endl;
  pVel6D1 = AL::Math::Velocity6D(1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f);
  compareVelocity6D((pVel6D1*2.0f), AL::Math::Velocity6D(2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f));


  //std::cout << "-------------- multiplication 1 --------------" << std::endl;
  pVel6D1 = AL::Math::Velocity6D(1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f);
  compareVelocity6D((2.0f*pVel6D1), AL::Math::Velocity6D(2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f));


  //std::cout << "-------------- division 0 --------------" << std::endl;
  pVel6D1 = AL::Math::Velocity6D(2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f);
  compareVelocity6D((pVel6D1/2.0f), AL::Math::Velocity6D(1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f));


  //std::cout << "-------------- division 1 --------------" << std::endl;
  //pVel6D1 = AL::Math::Velocity6D(2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f);
  //ASSERT_THROW((pVel6D1/0.0f), AL::ALError);
}

