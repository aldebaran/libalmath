#include <almath/types/aldisplacement.h>

#include <gtest/gtest.h>
#include <stdexcept>

namespace AL{

namespace Math{

TEST(ALDisplacementTest, isNear)
{
  Displacement test;
  Displacement test2;
  ASSERT_TRUE(test.isNear(test2));
  test.P = Position3D(1.f, 2.f, 3.f);
  test.Q = Quaternion(.5f, .5f, .5f, .5f);
  ASSERT_FALSE(test.isNear(test2));
}

TEST(ALDisplacementTest, Constructor)
{
  Displacement test;
  Position3D pose;
  ASSERT_TRUE(test.P.isNear(pose));
  Quaternion quat;
  ASSERT_TRUE(test.Q.isNear(quat));
  pose = Position3D(1.f, 2.f, 3.f);
  quat = Quaternion(.5f, .5f, .5f, .5f);
  test = Displacement(pose, quat);
  ASSERT_TRUE(test.P.isNear(pose));
  ASSERT_TRUE(test.Q.isNear(quat));
}

} // ends namespace Math

} // ends namespace AL
