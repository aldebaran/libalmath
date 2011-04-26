/**
 * @author Cyrille Collette - ccollette@aldebaran-robotics.com
 * Aldebaran Robotics (c) 2009 All Rights Reserved
 *
 */

#include <almath/collisions/isinsidepolygon.h>
#include <almath/types/alposition2d.h>
#include "../almathtestutils.h"

TEST(isInsidePolygonTest, Log)
{

  AL::Math::Position2D pA = AL::Math::Position2D();
  AL::Math::Position2D pB = AL::Math::Position2D();
  AL::Math::Position2D pC = AL::Math::Position2D();
  AL::Math::Position2D pD = AL::Math::Position2D();

  AL::Math::Position2D com;
  std::vector<AL::Math::Position2D> support;
  support.resize(4);

  //std::cout << "***** xIsSegementIntersect 0 *****" << std::endl;
  pA = AL::Math::Position2D(-1.0f, 0.0f);
  pB = AL::Math::Position2D(1.0f, 0.0f);
  pC = AL::Math::Position2D(0.0f, -1.0f);
  pD = AL::Math::Position2D(0.0f, 1.0f);

  EXPECT_TRUE(xIsSegementIntersect(pA, pB, pC, pD));

  //std::cout << "***** xIsSegementIntersect 1 *****" << std::endl;
  pA = AL::Math::Position2D(2.0f, -1.0f);
  pB = AL::Math::Position2D(1.5f, 1.5f);
  pC = AL::Math::Position2D(-1.0f, -1.0f);
  pD = AL::Math::Position2D(2.0f, 1.0f);

  EXPECT_TRUE(xIsSegementIntersect(pA, pB, pC, pD));

  //std::cout << "***** isInSupportPolygon 0 *****" << std::endl;
  com = AL::Math::Position2D(0.0f, 0.0f);
  support[0] = AL::Math::Position2D(-0.1f, -0.1f);
  support[1] = AL::Math::Position2D(-0.1f, +0.1f);
  support[2] = AL::Math::Position2D(+0.1f, +0.1f);
  support[3] = AL::Math::Position2D(+0.1f, -0.1f);

  EXPECT_TRUE(isInSupportPolygon(com, support));

  //std::cout << "***** isInSupportPolygon 1 *****" << std::endl;
  com = AL::Math::Position2D(10.0f, 0.0f);
  support[0] = AL::Math::Position2D(-1.0f, -1.0f);
  support[1] = AL::Math::Position2D(-1.0f, +1.0f);
  support[2] = AL::Math::Position2D(+1.0f, +1.0f);
  support[3] = AL::Math::Position2D(+1.0f, -1.0f);

  EXPECT_FALSE(isInSupportPolygon(com, support));

  //std::cout << "***** isInSupportPolygon 2 *****" << std::endl;
  com = AL::Math::Position2D(-0.00109488f, -0.0499441f);
  support[0] = AL::Math::Position2D(0.0869909f, -0.0120001f);
  support[1] = AL::Math::Position2D(0.0869909f, -0.1f);
  support[2] = AL::Math::Position2D(-0.0560091f, -0.1f);
  support[3] = AL::Math::Position2D(-0.0560091f, -0.0120001f);

  EXPECT_TRUE(isInSupportPolygon(com, support));

  //std::cout << "***** isInSupportPolygon 3 *****" << std::endl;
  com = AL::Math::Position2D(-0.00109488f, -0.0499441f);
  support.resize(3);
  support[0] = AL::Math::Position2D(0.0869909f, -0.0120001f);
  support[1] = AL::Math::Position2D(-0.0560091f, -0.1f);
  support[2] = AL::Math::Position2D(-0.0560091f, -0.0120001f);

  EXPECT_TRUE(isInSupportPolygon(com, support));

}

