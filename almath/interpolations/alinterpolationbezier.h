/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALINTERPOLATIONBEZIER_H_
#define _LIB_ALMATH_ALMATH_ALINTERPOLATIONBEZIER_H_

#include <almath/types/alposition3d.h>
#include <almath/interpolations/alinterpolationtypes.h>

namespace AL
{
  namespace Math
  {
    namespace Interpolation
    {

      class ALInterpolationBezier
      {
        public:

        /// <summary> Default constructor. </summary>
        ALInterpolationBezier(void);

        /**
        * Constrained 2-dimensionnal cubic Bezier interpolation.
        *
        * Bezier curves are parametric curves defined by 4 control points,
        * named P0, P1, P2 and P3. This function interpolates a 2D curve
        * linearly along its 's' parameter.
        * This curve is constrained so that its first derivative along x is
        * always positive. That way the curve can be considered as a function
        * y=f(x) and used to control some output value along time.
        * The output is a series of y values given at regular x intervals.
        *
        * To get a better understanding of Bezier curve and the meaning of the
        * four control points, see http://en.wikipedia.org/wiki/B%C3%A9zier_curve#Cubic_B.C3.A9zier_curves
        *
        * @param pP0  First  Bezier control point.
        * @param pP1  Second Bezier control point.
        * @param pP2  Third  Bezier control point.
        * @param pP3  Fourth Bezier control point.
        * @param pdt  Length of the interval along X between two successive outputs.
        */
        std::vector<float> Interpolate(
          float      pDuration,
          const Key& pFromKey,
          const Key& pToKey,
          float      pStartValue,
          float      pMinValue,
          float      pMaxValue,
          float      pValueIncrementLimit,
          float      pSampleDuration);

        std::vector<float> Interpolate(
          const AL::Math::Position2D& pP0,
          const AL::Math::Position2D& pP1,
          const AL::Math::Position2D& pP2,
          const AL::Math::Position2D& pP3,
          float                       pSampleDuration);

        void Init(
          float      pDuration,
          const Key& pFromKey,
          const Key& pToKey,
          float      pStartValue,
          float      pMinValue,
          float      pMaxValue,
          float      pValueIncrementLimit,
          float      pSampleDuration);


        float getCurrentBezierPosition(const float& tDes);


        // Apres initialisation, recuperer le nombre de sample pour un certain
        // echantillonnage
        unsigned int getNumberSample(void)
        {
          return fNbSamples;
        };


        unsigned int getGlobalIndex(void)
        {
          return fGlobalIndex;
        };

        bool isFinished(void);

        // new
        void setIsFinished(bool tmpFinished)
        {
          fIsFinished = tmpFinished;
        }

        float getNextPosition(void);

        /// <summary> Finaliser. </summary>
        virtual ~ALInterpolationBezier(void);

        float* getResult(void);

      protected:

        void xInit(
          const AL::Math::Position2D& pP0,
          const AL::Math::Position2D& pP1,
          const AL::Math::Position2D& pP2,
          const AL::Math::Position2D& pP3);

        void xFilterSolution(float& pIn);

        void xComputeNumberSample(const float& dt);

        // a partir d'un temps absolue dans l'interpolation,
        // calculer le parametre de bezier entre [0,1]
        float xGetCurrentParameter(const float& tCurrent);

        // a partir d'un parametre de bezier entre [0,1],
        // calculer le timeBezier et positionBezier correspondant
        void xGetBezierPosition(const float& tCurrent);

        float xGetBezierTime(const float& tCurrent);

        AL::Math::Position2D fP0;
        AL::Math::Position2D fP1;
        AL::Math::Position2D fP2;
        AL::Math::Position2D fP3;

        AL::Math::Position3D fNewton;

        float fa, fb, fc, fd;               // coeff polynome
        float f00, f01, f10, f11;           // Bezier polynoms values at t parameter
        float fs, fs2, fs3;                 // fs is the parameter along the curve
        float fBezierTime, fBezierPosition; // x and y values on the curve at s parameter
        float fPreviousBezierTime;          // x value on the curve at previous s parameter
        float fPreviousBezierPosition;      // y value on the curve at previous s parameter
        float fyT;                          // value of the curve at x = t
        float fprevYT;                      // value of the curve at previous tStep x = t-pdt
        float fCurrentTime;

        float fLastFilter;
        float fSampleDuration;

        float ft2, ft3;

        float fMinValue, fMaxValue;
        float fValueIncrementLimit;

        float fLastParameter;

        unsigned int fNoBoucle;
        unsigned int fNoSolution;

        unsigned int fNbSamples;
        unsigned int fIndex;     // Iterator on t values to avoid imprecisions accumulation

        unsigned int fGlobalIndex;

        bool fIsFinished;
      };

    }
  }
}

#endif  // _LIB_ALMATH_ALMATH_ALINTERPOLATIONBEZIER_H_

