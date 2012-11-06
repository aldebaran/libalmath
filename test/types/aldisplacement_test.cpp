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

} // ends namespace Math

} // ends namespace AL
