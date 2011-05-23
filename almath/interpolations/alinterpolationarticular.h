/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALINTERPOLATIONARTICULAR_H_
#define _LIB_ALMATH_ALMATH_ALINTERPOLATIONARTICULAR_H_

#include <vector>
#include <almath/types/alposition2d.h>
#include <almath/types/alpositionandvelocity.h>

namespace AL
{
  namespace Math
  {
    namespace Interpolation
    {
      class ALInterpolationArticular
      {
      public:
        /// <summary> Default constructor. </summary>
        ALInterpolationArticular(void);

        /// <summary> Initialize interpolation </summary>
        void Init(
          const std::vector<float>& pTime,
          const std::vector<float>& pPoint,
          const float&              pPeriod);

        /// <summary> Initialize interpolation </summary>
        void Init(
          const float& pTimeInit,
          const float& pTimeFinal,
          const float& pPointInit,
          const float& pPointFinal,
          const float& pVelocityInit,
          const float& pVelocityFinal,
          const float& pVelocityMaxAbs,
          const bool&  pChangeTimeFinal,
          const float& pPeriod);

        /// <summary> Initialize interpolation </summary>
        void Init(
          const float& pTimeInit,
          const float& pTimeFinal,
          const float& pPointInit,
          const float& pPointFinal,
          const float& pVelocityInit,
          const float& pVelocityFinal,
          const float& pPeriod);

        /// <summary> Main Initialize interpolation </summary>
        void Init(
          const std::vector<float>& pTime,
          const std::vector<float>& pPoint,
          const std::vector<float>& pVelocity,
          const bool&               pIsHotStart,
          const float&              pPeriod);

        // give current Position / Velocity angle at current time
        void getCurrentInterpolation(
          const float&                     time,
          AL::Math::PositionAndVelocity& pOut);

        AL::Math::PositionAndVelocity getCurrentInterpolation(const float& time);

        // give all Position / Velocity for all interpolation
        std::vector<AL::Math::PositionAndVelocity> getAllInterpolation(const float& dt_step);

        bool isFinished(const float& time);

        // new
        void setIsFinished(bool tmpFinished)
        {
          fIsFinished = tmpFinished;
        }

        inline const unsigned int getNumTimesCalled() const
        {
          return fNumTimesCalled;
        }

        /// <summary> Finaliser. </summary>
        virtual ~ALInterpolationArticular(void);

      protected:
        unsigned int  fNbEchantillon;
        unsigned int  fNbPoints;
        unsigned int  fNbAllInterpolation;
        unsigned int  fNumTimesCalled;
        float         fVelocityInit;
        float         fVelocityFinal;
        float         ft;
        float         fTmp;
        float         fPeriod; // period thread cycle
        bool          fIsFinished;
        float*  fTime;
        float*  fPoint;
        float*  fh;
        float*  fd;
        float*  fm;
        float*  fa;
        float*  fb;
        float*  fc;
        float*  fv;
        float*  fs1;
        float*  fs2;
        float*  fs3;
        float*  fs4;

        //void xChangeFinalTime(float& pFinalTime);
        void xCreateFloat(const unsigned int& nEchantillon);
        void xDestroyFloat(void);
      };
    }
  }
}

#endif  // _LIB_ALMATH_ALMATH_ALINTERPOLATIONARTICULAR_H_

