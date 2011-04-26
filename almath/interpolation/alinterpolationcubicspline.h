/**
* @author David GOUAILLIER
* Aldebaran Robotics (c) 2009 All Rights Reserved
*/
//#pragma once
#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALINTERPOLATIONCUBICSPLINE_H_
#define _LIB_ALMATH_ALMATH_ALINTERPOLATIONCUBICSPLINE_H_

#include <iostream>

namespace AL
{
  namespace Math
  {
    namespace Interpolation
    {

      class ALInterpolationCubicSpline
      {
      public:
        /**
        * Constructor.
        */
        ALInterpolationCubicSpline(
          const unsigned int pNbPoints = 0,
          const unsigned int pSampleDuration = 0);

        /**
        * Destructor.
        */
        virtual ~ALInterpolationCubicSpline ();

        /**
        * Initialize and Compute all interpolation parameters
        */
        void init(
          const float* pPoints,
          unsigned int pQpRatio);

        /**
        * Compute result interpolation at given point (int ms)
        */
        float getInterpolation(const unsigned int pDesiredTime);

      protected:
        unsigned int fNbPoints;  // Nb of interpolation control points
        unsigned int fSampleDuration;
        float* fTime;       // Array with time control
        float* fa;          // interpolation Coefficient
        float* fb;          // interpolation Coefficient
        float* fc;          // interpolation Coefficient
        float* fd;          // interpolation Coefficient
      };

    } // end namespace Interpolation
  } // end namespace Math
} // end namespace AL
#endif  // _LIB_ALMATH_ALMATH_ALINTERPOLATIONCUBICSPLINE_H_
