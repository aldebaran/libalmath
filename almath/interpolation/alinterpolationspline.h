/**
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved \n
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALINTERPOLATIONSPLINE_H_
#define _LIB_ALMATH_ALMATH_ALINTERPOLATIONSPLINE_H_

#include <vector>

namespace AL
{
  namespace Math
  {
    namespace Interpolation
    {
      namespace Spline
      {
        /**
        * Spline Interpolation between three or more float (3th Order Polynomial)
        * @return            : Interpolation Samples in a vector of float
        * @param pTime       : Vector of Time in second of the Interpolation
        * @param pValue      : Initial Value
        * @param pSampleTime : Sampling step in seconds
        */
        std::vector<float> Interpolate(
          std::vector<float> pTime,
          std::vector<float> pValue,
          float              pSampleTime);

        /**
        * Spline Interpolation between three or more float (3th Order Polynomial)
        * @return                      : Interpolation Samples in a vector of float
        * @param pTime                 : Vector of Time in second of the Interpolation
        * @param pValue                : Initial Value
        * @param pJointVelocityLimit   : The Joint velocity limit in rad/s
        * @param pSampleTime           : Sampling step in seconds
        */
        std::vector<float> Interpolate(
          std::vector<float> pTime,
          std::vector<float> pValue,
          float              pJointVelocityLimit,
          float              pSampleTime);
      }
    }
  }
}

#endif  // _LIB_ALMATH_ALMATH_ALINTERPOLATIONSPLINE_H_
