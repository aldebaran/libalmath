/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALINTERPOLATIONLINEAR_H_
#define _LIB_ALMATH_ALMATH_ALINTERPOLATIONLINEAR_H_

#include <vector>

namespace AL
{
  namespace Math
  {
    namespace Interpolation
    {
      namespace Linear
      {
        /**
        * Linearly Interpolation between two floats
        * @return      : Interpolation Samples in a vector of float
        * @param pDuration      : Duration in second of the Interpolation
        * @param pStartVal    : Initial Value
        * @param pEndVal      : Target Value
        * @param pMaxAngleChange : The Joint velocity limit in rad/TCycle
        * @param pSampleDuration : The interpolation step in seconds
        */
        std::vector<float> Interpolate(
          float pDuration,
          float pStartVal,
          float pEndVal,
          float pMaxAngleChange,
          float pSampleDuration);

        /**
        * Linearly Interpolation between two floats
        * @return    : Interpolation Samples in a vector of float
        * @param pNumSamples  : Number of sample of the Interpolation
        * @param pStartVal  : Initial Value
        * @param pEndVal    : Target Value
        */
        std::vector<float> Interpolate(
          int   pNumSamples,
          float pStartVal,
          float pEndVal);

        /**
        * Linearly Interpolation between two float
        * @return      : Interpolation Samples in a vector of float
        * @param pNumSamples      : Number of sample of the Interpolation
        * @param pStartVal      : Initial Value
        * @param pEndVal      : Target Value
        * @param pMaxAngleChange  : The Joint velocity limit in rad/TCycle
        */
        std::vector<float> Interpolate(
            int   pNumSamples,
            float pStartVal,
            float pEndVal,
            float pMaxAngleChange);

        /**
        * Linearly Interpolation between two vector of float
        * @return    : Interpolation Samples in a vector of Vector of float
        * @param pDuration    : Duration in second of the Interpolation
        * @param pStartVal  : Vector of Target Value
        * @param pEndVal    : Vector of Target Value
        * @param pSampleDuration : The interpolation step in seconds
        */
        std::vector<std::vector<float> > Interpolate(
            float              pDuration,
            std::vector<float> pStartVal,
            std::vector<float> pEndVal,
            float              pSampleDuration);

        /**
        * Linearly Interpolation between two vector of float
        * @return      : Interpolation Samples in a vector of vector of float
        * @param pDuration      : Duration in second of the Interpolation
        * @param pStartVal    : Vector of Initial Value
        * @param pEndVal      : Vector of Target Value
        * @param pMaxAngleChange : Vector of Joint velocity limit in rad/TCycle
        * @param pSampleDuration : The interpolation step in seconds
        */
        std::vector<std::vector<float> > Interpolate(
            float              pDuration,
            std::vector<float> pStartVal,
            std::vector<float> pEndVal,
            std::vector<float> pMaxAngleChange,
            float              pSampleDuration);

        /**
        * Linearly Interpolation between two T
        * @return        : Interpolation Samples in a vector of float
        * @param pDuration    : Duration in second of the Interpolation
        * @param pStartVal  : Initial Value
        * @param pEndVal    : Target Value
        * @param pSampleDuration : The interpolation step in seconds
        */
        template <typename T>
        std::vector<T> Interpolate(
            float pDuration,
            T     pStartVal,
            T     pEndVal,
            float pSampleDuration);
      }
    }
  }
}


#endif  // _LIB_ALMATH_ALMATH_ALINTERPOLATIONLINEAR_H_
