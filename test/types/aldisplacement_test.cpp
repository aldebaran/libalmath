#include <stdexcept>
#include <gtest/gtest.h>
#include <almath/tools/altransformhelpers.h>
#include <almath/types/aldisplacement.h>
#include <almath/types/alrotation.h>
#include <almath/types/altransform.h>

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

TEST(ALDisplacementTest, CompositionOp)
{
  Position3D translation(1.f, 2.f, 3.f);
  Rotation rotation = Rotation::fromAngleDirection(0.78f, 0.f, 1.f, 0.f);
  Transform transform = transformFromRotation(rotation);
  transform.r1_c4 += translation.x;
  transform.r2_c4 += translation.y;
  transform.r3_c4 += translation.z;
  Displacement disp(translation, quaternionFromTransform(transform));
  Displacement disp2 = disp;
  disp2 *= disp;
  Transform transform2 = transform;
  transform2 *= transform;
  Transform result = transformFromDisplacement(disp2);
  ASSERT_TRUE(result.isNear(transform2));
  disp2 = disp * disp;
  result = transformFromDisplacement(disp2);
  ASSERT_TRUE(result.isNear(transform2));
}


} // ends namespace Math

} // ends namespace AL
