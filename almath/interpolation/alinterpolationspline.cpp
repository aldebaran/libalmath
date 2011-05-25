/**
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved \n
*/

#include <almath/interpolation/alinterpolationspline.h>

#include "math.h"
#include <limits>
#include <cstdlib>

#include <exception>
#include <almath/tools/almath.h>

using std::vector;

namespace AL
{
  namespace Math
  {
    namespace Interpolation
    {
      namespace Spline
      {

        vector<float> Interpolate(
          vector<float> pTime,
          vector<float> pValue,
          float         pSampleTime)
        {
          vector<float> returnInterpol;

          if ( pTime.size() != pValue.size() )
          {
            throw std::invalid_argument(
              "ALMath: ALInterpolationSpline"
              "No equal size of vector of time and value for Spline Interpolator.");
            return returnInterpol;
          }

          if (pTime.size() < 3)
          {
            throw std::invalid_argument(
              "ALMath: ALInterpolationSpline"
              "Not enough points (min=3) for Spline Interpolator.");
            return returnInterpol;
          }

          unsigned short NbSamples = pTime.size();

          // Compute Tangent
          vector<float> Tangent;
          Tangent.reserve(pTime.size());
          Tangent.push_back(0.0f); // Init Tangent always 0.0
          for(unsigned short i=1; i<NbSamples-1; i++)
          {
            if (
              (Sign((pValue.at(i+1)-pValue.at(i))/(pTime.at(i+1)-pTime.at(i))) != Sign((pValue.at(i)-pValue.at(i-1))/(pTime.at(i)-pTime.at(i-1)))) ||
              (pValue.at(i) == pValue.at(i+1)) ||
              (pValue.at(i) == pValue.at(i-1)))
            {
              Tangent.push_back(0.0f);
            }
            else
            {
              //Tangent.push_back( (pValue.at(i+1)-pValue.at(i-1))/2.0f);
              //david correction 03/2009 -> stagiaire manu choregraphe
              Tangent.push_back( (pValue.at(i+1)-pValue.at(i-1))/(pTime.at(i+1)-pTime.at(i-1)));
            }
          }
          Tangent.push_back(0.0f);

          float Delta;
          float Alpha;
          float Beta;
          float Tau;

          for(unsigned short i=0; i<NbSamples-1; i++)
          {
            Delta = (pValue.at(i+1)-pValue.at(i))/(pTime.at(i+1)-pTime.at(i));
            Alpha = Tangent.at(i)/Delta;
            Beta = Tangent.at(i+1)/Delta;
            if ( (powf(Alpha,2.0f)+powf(Beta,2.0f)) > 9.0f )
            {
              Tau = 3.0f* powf((powf(Alpha,2.0f)+powf(Beta,2.0f)),-0.5f);
              Tangent.at(i) = Tau*Alpha*Delta;
              Tangent.at(i+1) = Tau*Beta*Delta;
            }
          }

          float h;
          float t, t2, t3;
          float h00, h10, h01, h11;
          unsigned short j = 0;
          for (unsigned short i=0; i<(((pTime.at(NbSamples-1)-pTime.at(0))/pSampleTime)); i++)
          {
            if ( (pSampleTime*i+pTime.at(0)) > pTime.at(j+1) )
            {
              j++;
            }
            h = (pTime.at(j+1) - pTime.at(j));
            t = ((pSampleTime*i+pTime.at(0))- pTime.at(j))/h;
            t2 = t*t;
            t3 = t2*t;
            h00 = 2*t3 - 3*t2 +1;
            h10 = t3 - 2*t2 + t;
            h01 = -2*t3 + 3*t2;
            h11 = t3 - t2;
            returnInterpol.push_back(
                h00*pValue.at(j) +
                h10*h*Tangent.at(j) +
                h01*pValue.at(j+1) +
                h11*h*Tangent.at(j+1) );
          }
          return returnInterpol;
        }

        vector<float> Interpolate(
          vector<float> pTime,
          vector<float> pValue,
          float         pMaxAngleChange,
          float         pSampleTime)
        {
          vector<float> returnInterpol;

          if (pTime.size() != pValue.size())
          {
            throw std::invalid_argument(
              "ALMath: ALInterpolationSpline"
              "No equal size of vector of time and value for Spline Interpolator.");
            return returnInterpol;
          }

          if (pTime.size() < 3)
          {
            throw std::invalid_argument(
              "ALMath: ALInterpolationSpline"
              "Not enough points (min=3) for Spline Interpolator.");
            return returnInterpol;
          }

          unsigned short NbSamples = pTime.size();

          // Check Speed
          // It's a big approximation throw linear but work, NO ?
          float SecurityCoefficient = 1.5f;
          for(unsigned short i=0; i<NbSamples-1; i++)
          {
            if (fabsf(pValue.at(i+1)-pValue.at(i))/(pTime.at(i+1)-pTime.at(i)) >
                pMaxAngleChange*50.0f/SecurityCoefficient)
            {
              pValue.at(i+1) = Sign(pValue.at(i+1)-pValue.at(i))* pMaxAngleChange * 50.0f /
                               SecurityCoefficient * (pTime.at(i + 1) - pTime.at(i)) + pValue.at(i);
            }
          }

          // Compute Tangent
          vector<float> Tangent;
          Tangent.reserve(pTime.size());
          Tangent.push_back(0.0f); // Init Tangent always 0.0
          for(unsigned short i=1; i<NbSamples-1; i++)
          {
            if (pValue.at(i+1) == pValue.at(i))
            {
              Tangent.push_back(0.0f);
            }
            else
            {
              if (
                (Sign((pValue.at(i+1)-pValue.at(i))/(pTime.at(i+1)-pTime.at(i))) !=
                 Sign((pValue.at(i)-pValue.at(i-1))/(pTime.at(i)-pTime.at(i-1)))) ||
                (pValue.at(i) == pValue.at(i+1)) ||
                (pValue.at(i) == pValue.at(i-1)))
              {
                Tangent.push_back(0.0f);
              }
              else
              {
                //Tangent.push_back( (pValue.at(i+1)-pValue.at(i-1))/2.0f);
                //david correction 03/2009 -> stagiaire manu choregrpahe
                Tangent.push_back( (pValue.at(i+1)-pValue.at(i-1))/(pTime.at(i+1)-pTime.at(i-1)));
              }
            }
          }
          Tangent.push_back(0.0f); //Last Tangent always 0.0

          float Delta;
          float Alpha;
          float Beta;
          float Tau;

          for(unsigned short i=0; i<NbSamples-1; i++)
          {
            Delta = (pValue.at(i+1)-pValue.at(i))/(pTime.at(i+1)-pTime.at(i));
            Alpha = Tangent.at(i)/Delta;
            Beta = Tangent.at(i+1)/Delta;
            if ( (powf(Alpha,2.0f)+powf(Beta,2.0f)) > 9.0f )
            {
              Tau = 3.0f* powf((powf(Alpha,2.0f)+powf(Beta,2.0f)),-0.5f);
              Tangent.at(i) = Tau*Alpha*Delta;
              Tangent.at(i+1) = Tau*Beta*Delta;
            }
          }

          float h;
          float t, t2, t3;
          float h00, h10, h01, h11;
          unsigned short j = 0;
          for (unsigned short i = 0; i< ( ((pTime.at(NbSamples-1)-pTime.at(0))/pSampleTime)) ; i++)
          {
            if ( (pSampleTime*i+pTime.at(0)) > pTime.at(j+1) )
            {
              j++;
            }
            h = (pTime.at(j+1) - pTime.at(j));
            t = ((pSampleTime*i+pTime.at(0))- pTime.at(j))/h;
            t2 = t*t;
            t3 = t2*t;
            h00 = 2*t3 - 3*t2 +1;
            h10 = t3 - 2*t2 + t;
            h01 = -2*t3 + 3*t2;
            h11 = t3 - t2;
            returnInterpol.push_back(
                h00*pValue.at(j) +
                h10*h*Tangent.at(j) +
                h01*pValue.at(j+1) +
                h11*h*Tangent.at(j+1));
          }
          return returnInterpol;

        }

      }
    }
  }
}
