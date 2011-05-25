/**
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved \n
*/

#include <almath/interpolation/alinterpolationbezier.h>

#include "math.h"
#include <limits>
#include <cstdlib>
#include <algorithm>

#include <exception>
#include <almath/tools/alpolynomialsolver.h>

using std::vector;

namespace AL
{
  namespace Math
  {
    namespace Interpolation
    {

      // begin constructeur
      ALInterpolationBezier::ALInterpolationBezier():
          fa(0.0f),
          fb(0.0f),
          fc(0.0f),
          fd(0.0f),
          f00(0.0f),
          f01(0.0f),
          f10(0.0f),
          f11(0.0f),
          fs(0.0f),
          fs2(0.0f),
          fs3(0.0f),
          fBezierTime(0.0f),
          fBezierPosition(0.0f),
          fPreviousBezierTime(0.0f),
          fPreviousBezierPosition(0.0f),
          fyT(0.0f),
          fprevYT(0.0f),
          fCurrentTime(0.0f),
          fLastFilter(0.0f),
          fSampleDuration(0.0f),
          ft2(0.0f),
          ft3(0.0f),
          fMinValue(-100000.0f),
          fMaxValue(100000.0f),
          fValueIncrementLimit(100000.0f),
          fLastParameter(0.0f),
          fNoBoucle(0),
          fNoSolution(0),
          fNbSamples(0),
          fIndex(0),
          fGlobalIndex(1),
          fIsFinished(false){}


      void ALInterpolationBezier::xInit(
        const Position2D& pP0,
        const Position2D& pP1,
        const Position2D& pP2,
        const Position2D& pP3)
      {

        if (pP3.x < pP0.x)
        {
          throw std::invalid_argument(
            "ALMath: ALInterpolationBezier "
            "Last control point must have higher abscissa than the first one.");
        }

        // Init save the bezier control point
        fP0 = pP0;
        fP1 = pP1;
        fP2 = pP2;
        fP3 = pP3;

        fa = -pP0.x + 3.0f*pP1.x - 3.0f*pP2.x + pP3.x;
        fb = 3.0f*pP0.x - 6.0f*pP1.x + 3.0f*pP2.x;
        fc = -3.0f*pP0.x + 3.0f*pP1.x;

        fBezierTime             = fP0.x;
        fPreviousBezierTime     = fP0.x;
        fPreviousBezierPosition = fP0.y;
        fprevYT                 = fP0.y;

        fIndex = 1;

        fGlobalIndex = 1;

        fNoBoucle = 1;
        fNoSolution = 0;

        fLastFilter = fP0.y;

        fLastParameter = 0.0f;

        fIsFinished = false;

        // mettre a jour le nombre de sample
        xComputeNumberSample(fSampleDuration);
      } // end xInit


      void ALInterpolationBezier::Init(
        float      pDuration,
        const Key& pFromKey,
        const Key& pToKey,
        float      pStartValue,
        float      pMinValue,
        float      pMaxValue,
        float      pValueIncrementLimit,
        float      pSampleDuration)
      {

        if (pDuration <= 0.0f)
        {
          throw std::invalid_argument(
            "ALMath: ALInterpolationBezier Duration must be strictly positive.");
        }

        fSampleDuration      = pSampleDuration;
        fValueIncrementLimit = pValueIncrementLimit;
        fMinValue            = pMinValue;
        fMaxValue            = pMaxValue;

        Position2D P0, P1, P2, P3;
        P0.x = 0.0f;
        P0.y = pFromKey.fValue;
        P3.x = pDuration;
        P3.y = pToKey.fValue;

        Tangent left, right;
        left = pFromKey.fRightTangent;
        right = pToKey.fLeftTangent;
        if (left.fType == CONSTANT)
        {
          P1   = P0;
          P3.y = P0.y;
          P2   = P3;
        }
        else
        {
          if (left.fType == LINEAR)
          {
            P1 = P0;
          }
          else
          {
            Position2D offset(left.fOffset.x, left.fOffset.y);
            P1 = P0 + offset;
          }
          if (right.fType == LINEAR)
          {
            P2 = P3;
          }
          else
          {
            Position2D offset(right.fOffset.x, right.fOffset.y);
            P2 = P3 + offset;
          }
        }

        xInit(P0, P1, P2, P3);
      } // end Init


      std::vector<float> ALInterpolationBezier::Interpolate(
        const AL::Math::Position2D& pP0,
        const AL::Math::Position2D& pP1,
        const AL::Math::Position2D& pP2,
        const AL::Math::Position2D& pP3,
        float                       pSampleDuration)
      {

        fSampleDuration      = pSampleDuration;
        fValueIncrementLimit = +1000.0f;
        fMinValue            = -1000.0f;
        fMaxValue            = +1000.0f;

        xInit(pP0, pP1, pP2, pP3);

        fNbSamples = getNumberSample(); // fNbSamples mise a jour dans Init

        //here we MUST use pdt since it defines the expected size of returnInterpol
        std::vector<float> result; //return y values, at regular t samples
        result.resize(fNbSamples); // reserve
        for (unsigned int i=0; i<fNbSamples; i++)
        {
          result[i] = getNextPosition();
        }

        return result;
      } // end Interpolate


      std::vector<float> ALInterpolationBezier::Interpolate(
        float       pDuration,
        const Key&  pFromKey,
        const Key&  pToKey,
        float       pStartValue,
        float       pMinValue,
        float       pMaxValue,
        float       pValueIncrementLimit,
        float       pSampleDuration)
      {

        Init(
          pDuration,
          pFromKey,
          pToKey,
          pStartValue,
          pMinValue,
          pMaxValue,
          pValueIncrementLimit,
          pSampleDuration);

        fNbSamples = getNumberSample(); // fNbSamples mise a jour dans Init

        //here we MUST use pdt since it defines the expected size of returnInterpol
        std::vector<float> result; //return y values, at regular t samples
        result.resize(fNbSamples);
        for (unsigned int i=0; i<fNbSamples; i++)
        {
          result[i] = getNextPosition();
        }

        return result;
      } // end Interpolate


      bool ALInterpolationBezier::isFinished(void)
      {
        if (fIsFinished ||
          (fGlobalIndex > fNbSamples) )
        {
          return true;
        }
        else
        {
          return false;
        }
      } // end isFinished


      float ALInterpolationBezier::getNextPosition(void)
      {
        float sol = 0.0f;
        if (fGlobalIndex == fNbSamples )
        {
          sol = fP3.y;
        }
        else
        {
          sol = getCurrentBezierPosition(((float) fGlobalIndex)*fSampleDuration);
        }
        xFilterSolution(sol);

        fGlobalIndex ++;
        return sol;
      } // end getNextPosition



      float ALInterpolationBezier::getCurrentBezierPosition(const float& tDes)
      {
        fd = -tDes;
        float pBezierParameter = xGetCurrentParameter(tDes);

        if ( (pBezierParameter>=(0.0f)) && (pBezierParameter<=(1.0f)) )
        {
          xGetBezierPosition(pBezierParameter);
          xFilterSolution(fBezierPosition);

          return fBezierPosition;
        }
        else
        {
          if (fGlobalIndex == 1)
          {
            return fP0.y;
          }
          else if ( fGlobalIndex == fNbSamples )
          {
            return fP3.y;
          }
          else
          {
            return 0.0f;
          }
        }
      } // end getCurrentBezierPosition


      void ALInterpolationBezier::xComputeNumberSample(const float& pdt)
      {
        if (pdt <= 0.0f)
        {
          throw std::invalid_argument(
            "ALMath: ALInterpolationBezier Sample interval must be strictly positive.");
        }

        //Note JB Desmottes 18-05-09 : We use here pdt to determine the
        //  number of samples for s but this parameter could be anything else.

        fNbSamples = (unsigned)floorf((fP3.x - fP0.x) / pdt + 0.5f);
      } // end xComputeNumberSample


      float ALInterpolationBezier::xGetBezierTime(const float& pBezierParameter)
      {
        fs  = pBezierParameter;
        fs2 = fs*fs;
        fs3 = fs2*fs;

        f00 = - fs3 + 3.0f*fs2 - 3.0f*fs + 1.0f;
        f10 =   3.0f*fs3 - 6.0f*fs2 + 3.0f*fs;
        f11 = - 3.0f*fs3 + 3.0f*fs2;
        f01 =   fs3;

        return f00*fP0.x + f10*fP1.x + f11*fP2.x + f01*fP3.x;
      } // end getBezierTime


      void ALInterpolationBezier::xGetBezierPosition(const float& pBezierParameter)
      {
        fs  = pBezierParameter;
        fs2 = fs*fs;
        fs3 = fs2*fs;

        f00 = - fs3 + 3.0f*fs2 - 3.0f*fs + 1.0f;
        f10 =   3.0f*fs3 - 6.0f*fs2 + 3.0f*fs  ;
        f11 = - 3.0f*fs3 + 3.0f*fs2            ;
        f01 =   fs3                            ;

        fBezierTime     = f00*fP0.x + f10*fP1.x + f11*fP2.x + f01*fP3.x;
        fBezierPosition = f00*fP0.y + f10*fP1.y + f11*fP2.y + f01*fP3.y;
      } // end getBezierPosition


      float ALInterpolationBezier::xGetCurrentParameter(const float& tCurrent)
      {
        float pCurrentParameter = 0.0f;
        AL::Math::Complex solPolynome[3];

        unsigned int nbSol = AL::Math::Troisiemedegre(fa, fb, fc, fd, solPolynome);
        unsigned int nbSolUsefull = 0;

        std::vector<float> allPertinentSolution;
        for (unsigned int i = 0; i<nbSol; i++)
        {
          if (solPolynome[i].im == 0.0f)
          {
            if ( (solPolynome[i].re >= 0.0f)&&(solPolynome[i].re <= 1.0f) )
            {
              pCurrentParameter = solPolynome[i].re;
              allPertinentSolution.push_back(solPolynome[i].re);
              nbSolUsefull = nbSolUsefull + 1;
            }
          }
        }

        if (nbSolUsefull == 0)
        {
          nbSol = AL::Math::Troisiemedegre(0.0f, fb, fc, fd, solPolynome);
          std::vector<float> allPertinentSolution;
          for (unsigned int i = 0; i<nbSol; i++)
          {
            if (solPolynome[i].im == 0.0f)
            {
              if ( (solPolynome[i].re >= 0.0f)&&(solPolynome[i].re <= 1.0f) )
              {
                pCurrentParameter = solPolynome[i].re;
                allPertinentSolution.push_back(solPolynome[i].re);
                nbSolUsefull = nbSolUsefull + 1;
              }
            }
          }
        }

        if (nbSolUsefull > 1)
        {
          std::sort(allPertinentSolution.begin(), allPertinentSolution.end());
          unsigned int noIndex = 0;
          while ((allPertinentSolution[noIndex] <fLastParameter) &&
            (noIndex<allPertinentSolution.size()))
          {
            noIndex++;
          }
          pCurrentParameter = allPertinentSolution[noIndex];
        }

        fLastParameter = pCurrentParameter;
        return pCurrentParameter;
      } // end xGetCurrentParameter


      void ALInterpolationBezier::xFilterSolution(float& pIn)
      {
        pIn = std::min(pIn, fMaxValue);
        pIn = std::max(pIn, fMinValue);
        pIn = std::min(pIn, fLastFilter + fValueIncrementLimit);
        pIn = std::max(pIn, fLastFilter - fValueIncrementLimit);

        fLastFilter = pIn;
      } // end xFilterSolution


      ALInterpolationBezier::~ALInterpolationBezier()
      {
      } // end destructeur


    } //namespace Interpolation
  }
}


