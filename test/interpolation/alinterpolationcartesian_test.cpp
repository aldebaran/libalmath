/**
* @author Cyrille Collette - ccollette@aldebaran-robotics.com
* Aldebaran Robotics (c) 2009 All Rights Reserved
*
*/
#include <almath/interpolation/alinterpolationcartesian.h>
#include "../almathtestutils.h"

AL::Math::Interpolation::ALInterpolationCartesian interpolatorCartesian;

TEST(ALInterpolationCartesianTest, test0)
{
  std::vector<float>                 pTime;
  std::vector<AL::Math::Transform> pHi;
  std::vector<AL::Math::Velocity6D>  pVi;
  float                              pPeriod = 0.02f;
  AL::Math::TransformAndVelocity6D solCurrent;

  pTime.resize(2);
  pTime.at(0) = 0.0f;
  pTime.at(1) = 1.0f;

  pHi.resize(2);
  pHi.at(0) = AL::Math::Transform();
  pHi.at(1) = AL::Math::Transform();

  pVi.resize(2);
  pVi.at(0) = AL::Math::Velocity6D();
  pVi.at(1) = AL::Math::Velocity6D();

  interpolatorCartesian.Init(
    pTime,
    pHi,
    pVi,
    pPeriod);

  // t = 0.5
  interpolatorCartesian.getCurrentInterpolation(0.5f, solCurrent);

  compareTransform(solCurrent.H, AL::Math::Transform());
  compareVelocity6D(solCurrent.V, AL::Math::Velocity6D());
}

