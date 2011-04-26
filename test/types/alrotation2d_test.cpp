/**
 * @author Cyrille Collette - ccollette@aldebaran-robotics.com
 * Aldebaran Robotics (c) 2009 All Rights Reserved
 *
 */
#include <almath/types/alrotation2d.h>
#include "../almathtestutils.h"


//tmp.determinant
//tmp.isNear
//tmp.operator *
//tmp.operator *=
//tmp.transpose

TEST(ALRotation2DTest, determinant)
{
  AL::Math::Rotation2D tmp = AL::Math::Rotation2D();
  EXPECT_NEAR(tmp.determinant(), 1.0f, 0.001f);

  tmp = AL::Math::Rotation2D::fromAngle(0.5f);
  EXPECT_NEAR(tmp.determinant(), 1.0f, 0.001f);
}

TEST(ALRotation2DTest, isNear)
{
  AL::Math::Rotation2D tmp = AL::Math::Rotation2D();
  EXPECT_TRUE(tmp.isNear(AL::Math::Rotation2D(), 0.001f));

  tmp = AL::Math::Rotation2D::fromAngle(0.5f);
  EXPECT_FALSE(tmp.isNear(AL::Math::Rotation2D::fromAngle(0.4f), 0.001f));
}


TEST(ALRotation2DTest, variousOperator)
{
  AL::Math::Rotation2D sol;
  sol.r1_c1 = 0.82533561490968f;
  sol.r1_c2 = -0.56464247339504f;
  sol.r2_c1 = 0.56464247339504f;
  sol.r2_c2 = 0.82533561490968f;

  // operator *
  AL::Math::Rotation2D rot1 = AL::Math::Rotation2D::fromAngle(0.2f);
  AL::Math::Rotation2D rot2 = AL::Math::Rotation2D::fromAngle(0.4f);

  AL::Math::Rotation2D rot3 = rot1*rot2;

  compareRotation2D(rot3, sol, 0.0001f);

  // operator *=
  rot1 = AL::Math::Rotation2D::fromAngle(0.2f);
  rot2 = AL::Math::Rotation2D::fromAngle(0.4f);

  rot1 *= rot2;
  compareRotation2D(rot1, sol, 0.0001f);
}

TEST(ALRotation2DTest, transpose)
{
  AL::Math::Rotation2D rot = AL::Math::Rotation2D::fromAngle(0.2f);
  AL::Math::Rotation2D sol;

  sol.r1_c1 = rot.r1_c1;
  sol.r2_c2 = rot.r2_c2;
  sol.r1_c2 = rot.r2_c1;
  sol.r2_c1 = rot.r1_c2;

  compareRotation2D(rot.transpose(), sol, 0.0001f);
}
