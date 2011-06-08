/**
 * @author Cyrille Collette - ccollette@aldebaran-robotics.com
 * Aldebaran Robotics (c) 2009 All Rights Reserved
 *
 */
#include <almath/interpolations/alinterpolationarticular.h>
#include <almath/interpolations/alinterpolationutils.h>
#include "../almathtestutils.h"

std::vector<float> pTime;
std::vector<float> pPoint;
std::vector<float> pVelocity;
float pPeriod = 0.02f;
bool pIsHotStart = false;
std::vector<AL::Math::PositionAndVelocity> Solution;
AL::Math::Interpolation::ALInterpolationArticular interpolatorArticular;
AL::Math::PositionAndVelocity solCurrent;

TEST(ALInterpolationArticularTest, norm)
{

  //std::cout << "***** Exemple 0 *****" << std::endl;

  pTime.resize(2);
  pTime[0] = 0.0f;
  pTime[1] = 1.50f;

  pPoint.resize(2);
  pPoint[0] = 2.0f;
  pPoint[1] = 0.0f;

  pVelocity.resize(2);
  pVelocity[0] = +1.0f;
  pVelocity[1] = -1.0f;

  interpolatorArticular.Init(pTime, pPoint, pVelocity, pIsHotStart, pPeriod);

  // t = 0.5
  solCurrent = interpolatorArticular.getCurrentInterpolation(0.5f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(1.81481481481481f,  -1.44444444444444f));

  // t = 1.0
  solCurrent = interpolatorArticular.getCurrentInterpolation(1.0f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(0.85185185185185f,  -2.11111111111111f));

  //std::cout << "***** Exemple 1 *****" << std::endl;

  pTime.resize(2);
  pTime[0] = 1.0f;
  pTime[1] = 2.50f;

  pPoint.resize(2);
  pPoint[0] = 2.0f;
  pPoint[1] = 0.0f;

  pVelocity.resize(2);
  pVelocity[0] = +1.0f;
  pVelocity[1] = -1.0f;

  interpolatorArticular.Init(pTime, pPoint, pVelocity, pIsHotStart, pPeriod);

  // t = 1.5
  solCurrent = interpolatorArticular.getCurrentInterpolation(1.5f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(1.81481481481481f,  -1.44444444444444f));

  // t = 2.0
  solCurrent = interpolatorArticular.getCurrentInterpolation(2.0f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(0.85185185185185f,  -2.11111111111111f));


  //std::cout << "***** Exemple 2 *****" << std::endl;

  pTime.resize(2);
  pTime[0] = 0.0f;
  pTime[1] = 0.009f;

  pPoint.resize(2);
  pPoint[0] = 2.0f;
  pPoint[1] = 0.0f;

  pVelocity.resize(2);
  pVelocity[0] = +1.0f;
  pVelocity[1] = -1.0f;

  interpolatorArticular.Init(pTime, pPoint, pVelocity, pIsHotStart, pPeriod);

  // t = 0.0
  solCurrent = interpolatorArticular.getCurrentInterpolation(0.0f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(2.0f,  1.0f));

  // t = 0.02
  solCurrent = interpolatorArticular.getCurrentInterpolation(0.02f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(0.0f,  -1.0f));

  //std::cout << "***** Exemple 3 *****" << std::endl;

  pTime.resize(3);
  pTime[0] = 0.0f;
  pTime[1] = 0.2f;
  pTime[2] = 0.4f;

  pPoint.resize(3);
  pPoint[0] = 2.0f;
  pPoint[1] = 5.0f;
  pPoint[2] = 3.0f;

  pVelocity.resize(2);
  pVelocity[0] = +2.0f;
  pVelocity[1] = -1.0f;

  interpolatorArticular.Init(pTime, pPoint, pVelocity, pIsHotStart, pPeriod);

  Solution = interpolatorArticular.getAllInterpolation(pPeriod);

  // (t, q, dq) = 0.02000000000000f   2.11010000000000f   8.76500000000000f
  comparePositionAndVelocity(Solution[0], AL::Math::PositionAndVelocity(2.11010000000000f, 8.76500000000000f));

  // (t, q, dq) =    0.04000000000000f   2.34080000000000f  14.06000000000000f
  comparePositionAndVelocity(Solution[1], AL::Math::PositionAndVelocity(2.34080000000000f,  14.06000000000000f));

  // (t, q, dq) =    0.06000000000000f   2.66270000000000f  17.88500000000000f
  comparePositionAndVelocity(Solution[2], AL::Math::PositionAndVelocity(2.66270000000000f,  17.88500000000000f));

  // (t, q, dq) =    0.08000000000000f   3.04640000000000f  20.24000000000000f
  comparePositionAndVelocity(Solution[3], AL::Math::PositionAndVelocity(3.04640000000000f, 20.24000000000000f));

  // (t, q, dq) =    0.10000000000000f   3.46250000000000f  21.12500000000000f
  comparePositionAndVelocity(Solution[4], AL::Math::PositionAndVelocity(3.46250000000000f, 21.12500000000000f));

  // (t, q, dq) =    0.12000000000000f   3.88160000000000f  20.54000000000001f
  comparePositionAndVelocity(Solution[5], AL::Math::PositionAndVelocity(3.88160000000000f, 20.54000000000001f));

  // (t, q, dq) =    0.14000000000000f   4.27430000000000f  18.48500000000001f
  comparePositionAndVelocity(Solution[6], AL::Math::PositionAndVelocity(4.27430000000000f, 18.48500000000001f));

  // (t, q, dq) =    0.16000000000000f   4.61120000000000f  14.96000000000001f
  comparePositionAndVelocity(Solution[7], AL::Math::PositionAndVelocity(4.61120000000000f, 14.96000000000001f));

  // (t, q, dq) =    0.18000000000000f   4.86290000000000f   9.96500000000002f
  comparePositionAndVelocity(Solution[8], AL::Math::PositionAndVelocity(4.86290000000000f, 9.96500000000002f));

  // (t, q, dq) =    0.20000000000000f   5.00000000000000f   3.50000000000000f
  comparePositionAndVelocity(Solution[9], AL::Math::PositionAndVelocity(5.00000000000000f, 3.50000000000000f));

  // (t, q, dq) =    0.22000000000000f   5.00250000000000f  -3.02500000000001f
  comparePositionAndVelocity(Solution[10], AL::Math::PositionAndVelocity(5.00250000000000f, -3.02500000000001f));

  // (t, q, dq) =    0.24000000000000f   4.88800000000000f  -8.20000000000000f
  comparePositionAndVelocity(Solution[11], AL::Math::PositionAndVelocity(4.88800000000000f, -8.20000000000000f));

  // (t, q, dq) =    0.26000000000000f   4.68350000000000f -12.02500000000000f
  comparePositionAndVelocity(Solution[12], AL::Math::PositionAndVelocity(4.68350000000000f, -12.02500000000000f));

  // (t, q, dq) =    0.28000000000000f   4.41600000000000f -14.50000000000000f
  comparePositionAndVelocity(Solution[13], AL::Math::PositionAndVelocity(4.41600000000000f, -14.50000000000000f));

  // (t, q, dq) =    0.30000000000000f   4.11250000000000f -15.62500000000000f
  comparePositionAndVelocity(Solution[14], AL::Math::PositionAndVelocity(4.11250000000000f, -15.62500000000000f));

  // (t, q, dq) =    0.32000000000000f   3.80000000000000f -15.40000000000000f
  comparePositionAndVelocity(Solution[15], AL::Math::PositionAndVelocity(3.80000000000000f, -15.40000000000000f));

  // (t, q, dq) =    0.34000000000000f   3.50550000000000f -13.82500000000000f
  comparePositionAndVelocity(Solution[16], AL::Math::PositionAndVelocity(3.50550000000000f, -13.82500000000000f));

  // (t, q, dq) =    0.36000000000000f   3.25600000000000f -10.90000000000000f
  comparePositionAndVelocity(Solution[17], AL::Math::PositionAndVelocity(3.25600000000000f, -10.90000000000000f));

  // (t, q, dq) =    0.38000000000000f   3.07850000000000f  -6.62500000000001f
  comparePositionAndVelocity(Solution[18], AL::Math::PositionAndVelocity(3.07850000000000f, -6.624980000000000f));

  // (t, q, dq) =    0.40000000000000f   3.00000000000000f  -1.00000000000000f
  comparePositionAndVelocity(Solution[19], AL::Math::PositionAndVelocity(3.00000000000000f, -1.00000000000000f));

  //std::cout << "***** Exemple 4 *****" << std::endl;

  pTime.resize(7);
  pTime[0] = 0.0f;
  pTime[1] = 1.0f;
  pTime[2] = 2.0f;
  pTime[3] = 3.0f;
  pTime[4] = 4.0f;
  pTime[5] = 5.0f;
  pTime[6] = 6.0f;

  pPoint.resize(7);
  pPoint[0] = 1.0f;
  pPoint[1] = 0.0f;
  pPoint[2] = 0.0f;
  pPoint[3] = 1.0f;
  pPoint[4] = 2.0f;
  pPoint[5] = 2.0f;
  pPoint[6] = 1.0f;

  pVelocity.resize(2);
  pVelocity[0] = -0.2f;
  pVelocity[1] = -2.5f;

  interpolatorArticular.Init(pTime, pPoint, pVelocity, pIsHotStart, pPeriod);

  // t = 0.5
  solCurrent = interpolatorArticular.getCurrentInterpolation(0.5f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(0.58328525641026f,  -1.23342948717949f));

  // t = 1.0
  solCurrent = interpolatorArticular.getCurrentInterpolation(1.0f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(0.0f,  -0.86628205128205f));

  // t = 1.5
  solCurrent = interpolatorArticular.getCurrentInterpolation(1.5f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(-0.19142628205128f,   0.05028846153846f));

  // t = 2.0
  solCurrent = interpolatorArticular.getCurrentInterpolation(2.0f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(0.0f,   0.66512820512821f));

  // t = 2.5
  solCurrent = interpolatorArticular.getCurrentInterpolation(2.5f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(0.43241987179487f,   1.03227564102564f));

  // t = 3.0
  solCurrent = interpolatorArticular.getCurrentInterpolation(3.0f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(1.00000000000000f,   1.20576923076923f));

  // t = 3.5
  solCurrent = interpolatorArticular.getCurrentInterpolation(3.5f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(1.58674679487179f,   1.07060897435897f));

  // t = 4.0
  solCurrent = interpolatorArticular.getCurrentInterpolation(4.0f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(2.00000000000000f,   0.51179487179487f));

  // t = 4.5
  solCurrent = interpolatorArticular.getCurrentInterpolation(4.5f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(2.09559294871795f,  -0.06471153846154f));

  // t = 5.0
  solCurrent = interpolatorArticular.getCurrentInterpolation(5.0f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(2.00000000000000f,  -0.25294871794872f));

  // t = 5.5
  solCurrent = interpolatorArticular.getCurrentInterpolation(5.5f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(1.78088141025641f,  -0.81176282051282f));


  //std::cout << "***** Exemple 5 *****" << std::endl;

  pTime.resize(7);
  pTime[0] = 1.0f;
  pTime[1] = 2.0f;
  pTime[2] = 3.0f;
  pTime[3] = 4.0f;
  pTime[4] = 5.0f;
  pTime[5] = 6.0f;
  pTime[6] = 7.0f;

  pPoint.resize(7);
  pPoint[0] = 1.0f;
  pPoint[1] = 0.0f;
  pPoint[2] = 0.0f;
  pPoint[3] = 1.0f;
  pPoint[4] = 2.0f;
  pPoint[5] = 2.0f;
  pPoint[6] = 1.0f;

  pVelocity.resize(2);
  pVelocity[0] = -0.2f;
  pVelocity[1] = -2.5f;

  interpolatorArticular.Init(pTime, pPoint, pVelocity, pIsHotStart, pPeriod);

  // t = 1.5
  solCurrent = interpolatorArticular.getCurrentInterpolation(1.5f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(0.58328525641026f,  -1.23342948717949f));

  // t = 2.0
  solCurrent = interpolatorArticular.getCurrentInterpolation(2.0f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(0.0f,  -0.86628205128205f));

  // t = 2.5
  solCurrent = interpolatorArticular.getCurrentInterpolation(2.5f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(-0.19142628205128f,   0.05028846153846f));

  // t = 3.0
  solCurrent = interpolatorArticular.getCurrentInterpolation(3.0f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(0.0f,   0.66512820512821f));

  // t = 3.5
  solCurrent = interpolatorArticular.getCurrentInterpolation(3.5f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(0.43241987179487f,   1.03227564102564f));

  // t = 4.0
  solCurrent = interpolatorArticular.getCurrentInterpolation(4.0f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(1.00000000000000f,   1.20576923076923f));

  // t = 4.5
  solCurrent = interpolatorArticular.getCurrentInterpolation(4.5f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(1.58674679487179f,   1.07060897435897f));

  // t = 5.0
  solCurrent = interpolatorArticular.getCurrentInterpolation(5.0f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(2.00000000000000f,   0.51179487179487f));

  // t = 5.5
  solCurrent = interpolatorArticular.getCurrentInterpolation(5.5f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(2.09559294871795f,  -0.06471153846154f));

  // t = 6.0
  solCurrent = interpolatorArticular.getCurrentInterpolation(6.0f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(2.00000000000000f,  -0.25294871794872f));

  // t = 6.5
  solCurrent = interpolatorArticular.getCurrentInterpolation(6.5f);
  comparePositionAndVelocity(solCurrent, AL::Math::PositionAndVelocity(1.78088141025641f,  -0.81176282051282f));


  //std::cout << "***** Exemple 6 *****" << std::endl;

  float pPointInit      = 0.390119f;
  float pPointFinal     = 0.909297f;
  float pVelocityInit   = 0.827173f;
  float pVelocityFinal  = 0.0f;
  float pVelocityMaxAbs = 0.826797f;

  float pTimeFinal = AL::Math::getTimeFinalJoint(
    pPointInit,
    pPointFinal,
    pVelocityInit,
    pVelocityFinal,
    pVelocityMaxAbs,
    pPeriod);

  EXPECT_NEAR(pTimeFinal, 0.94f, 0.001f);

}

