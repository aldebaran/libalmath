/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALINTERPOLATIONSMOOTH_H_
#define _LIB_ALMATH_ALMATH_ALINTERPOLATIONSMOOTH_H_

#include <vector>

namespace AL
{
  namespace Math
  {
    namespace Interpolation
    {
      namespace Smooth
      {
        /**
        * Smooth Interpolation between two values (3th Order Polynomial)
        * @return                : Interpolation Samples in a vector of float
        * @param pDuration       : Time in second of the Interpolation
        * @param pStartVal       : Initial Value
        * @param pEndVal         : Target Value
        * @param pMaxAngleChange : The Joint velocity limit in rad/TCycle
        * @param pSampleTime     : The sampling step
        */
        std::vector<float> Interpolate(
          float       pDuration,
          const float pStartVal,
          const float pEndVal,
          const float pMaxAngleChange,
          const float pSampleTime);

        /**
        * Smooth Interpolation between two vector of float (3th Order Polynomial)
        * @return            : Interpolation Samples in a vector of float
        * @param pDuration   : Time in second of the Interpolation
        * @param pStartVal   : Vector of Initial Value
        * @param pEndVal     : Vector of Target Value
        * @param pSampleTime : The sampling step
        */
        std::vector<std::vector<float> > Interpolate(
          float                     pDuration,
          const std::vector<float>& pStartVal,
          const std::vector<float>& pEndVal,
          const float               pSampleTime);

        /**
        * Smooth Interpolation between two vector of float (3th Order Polynomial)
        * @return                : Interpolation Samples in a vector of float
        * @param pDuration       : Time in second of the Interpolation
        * @param pStartVal       : Vector of Initial Value
        * @param pEndVal         : Vector of Target Value
        * @param pMaxAngleChange : Vector of Joint velocity limit in rad/TCycle
        * @param pSampleTime     : The sampling step
        */
        std::vector<std::vector<float> > Interpolate(
          float                     pDuration,
          const std::vector<float>& pStartVal,
          const std::vector<float>& pEndVal,
          const std::vector<float>& pMaxAngleChange,
          float                     pSampleTime);

        /**
        * Smooth Interpolation between two Position2D (3th Order Polynomial)
        * @return            : Interpolation Samples in a vector of float
        * @param pNbSample   : Number of sample of the Interpolation
        * @param pStartVal   : Initial Value
        * @param pEndVal     : Target Value
        * @param pSampleTime : The sampling step
        */
        template <typename T>
        std::vector<T> Interpolate(
          int   pNbSample,
          T     pStartVal,
          T     pEndVal,
          const float pSampleTime);

        /**
        * Smooth Interpolation between two Ts (3th Order Polynomial)
        * @return            : Interpolation Samples in a vector of float
        * @param pTime       : Time in second of the Interpolation
        * @param pStartVal   : Initial Value
        * @param pEndVal     : Target Value
        * @param pSampleTime : The sampling step
        */
        template <typename T>
        std::vector<T> Interpolate(
          float       pTime,
          T           pStartVal,
          T           pEndVal,
          const float pSampleTime);


        /**
        * Smooth Interpolation between two float (5th Order Polynomial)
        * @param pNbSample     : Number of sample of the Interpolation
        * @param pStartVal     : Initial Value
        * @param pEndVal       : Target Value
        * @param *pResultArray : A pre Allocated array for interpolation result
        */
        void Interpolate(
          int         pNbSample,
          const float pStartVal,
          const float pEndVal,
          const float pSampleTime,
          float*      pResultArray);

      }
    }
  }
}

#endif  // _LIB_ALMATH_ALMATH_ALINTERPOLATIONSMOOTH_H_
