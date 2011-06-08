/**
 * @author Cyrille Collette - ccollette@aldebaran-robotics.com
 * Aldebaran Robotics (c) 2009 All Rights Reserved
 *
 */
#include <almath/interpolations/alinterpolationbezier.h>
#include "../almathtestutils.h"


TEST(ALInterpolationBezierTest, norm)
{
  EXPECT_NEAR(0.0f, 0.0f, kEpsilon);
}

