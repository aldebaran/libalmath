/**
 * @author Cyrille Collette - ccollette@aldebaran-robotics.com
 * Aldebaran Robotics (c) 2009 All Rights Reserved
 *
 */
#include <almath/types/alrotation3d.h>
#include "../almathtestutils.h"

TEST(ALRotation3DTest, norm)
{
  AL::Math::Rotation3D pRot = AL::Math::Rotation3D(0.0f, 0.0f, 0.0f);
  EXPECT_NEAR(AL::Math::norm(pRot), 0.0f, kEpsilon);
}

