/**
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved \n
*/

#include <almath/interpolation/alinterpolationsmooth.h>

#include "math.h"
#include <limits>
#include <iostream>
#include <cstdlib>

#include <alcore/alerror.h>
#include <almath/tools/almath.h>

using std::vector;

namespace AL
{
  namespace Math
  {
    namespace Interpolation
    {
      namespace Smooth
      {

        vector<float> Interpolate(
          float       pDuration,
          const float pStartVal,
          const float pEndVal,
          const float pMaxAngleChange,
          const float pSampleTime)
        {
          vector<float> returnInterpolation;
          int NbSample;
          float tf3,tf4,tf5;
          float t,t3,t4,t5;
          float D;
          float a3,a4,a5; //Interpolation Coefficient

          //compute diff
          D = pEndVal - pStartVal;

          if (pDuration > 0.0f)
          {
            // "Normal" behaviour
            if (pDuration < pSampleTime)
            {
              std::cout << "WARNING : lowest Interpolation Time \nChange Time to the Robot Cycle " << std::endl;
              pDuration = pSampleTime;
            }

            float MaxVelocity = 15.0f*D/(8.0f*pDuration);
            if ( fabsf(MaxVelocity) > pMaxAngleChange/pSampleTime)
            {
              std::cout << "WARNING : Velocity Joint Limit reached" << std::endl;
              D = Sign(MaxVelocity)*(pMaxAngleChange/pSampleTime)*8.0f*pDuration/15.0f;
            }
          }
          else
          {
            // With speed behaviour
            pDuration = fabsf(15.0f*D/(8.0f*(-pDuration)*(pMaxAngleChange/pSampleTime)));
          }

          pDuration = ceilf(pDuration/pSampleTime)*pSampleTime;
          tf3 = pDuration*pDuration*pDuration;
          tf4 = tf3*pDuration;
          tf5 = tf4*pDuration;
          a3 = 10.0f/tf3*D;
          a4 = -15.0f/tf4*D;
          a5 = 6.0f/tf5*D;

          NbSample = (int)(pDuration/pSampleTime);

          returnInterpolation.reserve(NbSample);
          for (unsigned short i=0; i<NbSample; i++)
          {
            t = pSampleTime*(i+1);
            t3 = t*t*t;
            t4 = t3*t;
            t5 = t4*t;
            returnInterpolation.push_back(pStartVal+a3*t3+a4*t4+a5*t5);
          }
          return returnInterpolation;
        }


        vector<vector<float> > Interpolate(
          float                     pDuration,
          const std::vector<float>& pStartVal,
          const std::vector<float>& pEndVal,
          const float               pSampleTime)
        {
          std::vector<std::vector<float> > returnInterpolation;
          vector<float> tmpInterpolation;
          int NbSample;
          int sizeOfVector;
          float tf3,tf4,tf5;
          float t,t3,t4,t5;
          float D;
          vector<float> a3,a4,a5; //Interpolation Coefficient

          if (pStartVal.size() != pEndVal.size())
          {
            throw ALERROR(
              "ALMath",
              "ALInterpolationSmooth",
              "StartValues EndValues have different size.");
            return returnInterpolation;
          }
          else
          {
            sizeOfVector = pStartVal.size();
          }

          if (pDuration < pSampleTime)
          {
            std::cout << "WARNING : lowest Interpolation Time \nChange Time to the Robot Cycle " << std::endl;
            pDuration = pSampleTime;
          }

          pDuration = ceilf(pDuration/pSampleTime)*pSampleTime;
          tf3 = pDuration*pDuration*pDuration;
          tf4 = tf3*pDuration;
          tf5 = tf4*pDuration;

          for (unsigned short i=0; i<sizeOfVector; i++)
          {
            D = pEndVal[i]-pStartVal[i];
            a3.push_back(10.0f/tf3*D);
            a4.push_back(-15.0f/tf4*D);
            a5.push_back(6.0f/tf5*D);
          }

          NbSample = (int)(pDuration/pSampleTime);

          for (unsigned short i=0; i<NbSample; i++)
          {
            t = pSampleTime*(i+1);
            t3 = t*t*t;
            t4 = t3*t;
            t5 = t4*t;
            for (unsigned short j=0; j<sizeOfVector; j++)
            {
              tmpInterpolation.push_back(pStartVal[j] + a3[j]*t3 + a4[j]*t4 + a5[j]*t5);
            }
            returnInterpolation.push_back(tmpInterpolation);
            tmpInterpolation.clear();
          }

          return returnInterpolation;
        }


        vector<vector<float> > Interpolate(
          float                     pDuration,
          const std::vector<float>& pStartVal,
          const std::vector<float>& pEndVal,
          const std::vector<float>& pMaxAngleChange,
          const float               pSampleTime)
        {
          vector<vector<float> > returnInterpolation;
          vector<float> tmpInterpolation;
          float tmpDuration = 0;
          int NbSample = 0;
          int sizeOfVector;
          float tf3,tf4,tf5;
          float t,t3,t4,t5;
          vector<float> D;
          vector<float> a3,a4,a5;  //Interpolation Coefficient

          if (pStartVal.size() != pEndVal.size())
          {
            throw ALERROR(
              "ALMath",
              "ALInterpolationSmooth",
              "StartValues EndValues have different size.");
            return returnInterpolation;
          }
          else
          {
            sizeOfVector = pStartVal.size();
          }

          // Compute Diff
          for (unsigned short i=0; i<sizeOfVector; i++)
          {
            D.push_back(pEndVal[i] - pStartVal[i]);
          }

          if (pDuration > 0.0f)
          {
            // "Normal" behaviour
            if (pDuration < pSampleTime)
            {
              std::cout << "WARNING : lowest Interpolation Time \nChange Time to the Robot Cycle " << std::endl;
              pDuration = pSampleTime;
            }

            for (unsigned short i=0; i<sizeOfVector; i++)
            {
              float MaxVelocity = 15.0f*D.at(i)/(8.0f*pDuration);
              if (fabsf(MaxVelocity) > pMaxAngleChange[i]/pSampleTime)
              {
                std::cout << "WARNING : Velocity Joint Limit reached" << std::endl;
                D.at(i) = Sign(MaxVelocity)*(pMaxAngleChange[i]/pSampleTime)*8.0f*pDuration/15.0f;
              }
            }
          }
          else
          {
            // With speed behaviour
            float MaxDuration = 0.0f;
            for (unsigned short i=0; i<sizeOfVector; i++)
            {
              tmpDuration = 15.0f*D.at(i)/(8.0f*(-pDuration)*(pMaxAngleChange[i]/pSampleTime));
              if ( fabsf(tmpDuration) > MaxDuration)
              {
                MaxDuration = fabsf(tmpDuration);
              }
            }
            pDuration = MaxDuration;
          }

          pDuration = ceilf(pDuration/pSampleTime)*pSampleTime;
          tf3 = pDuration*pDuration*pDuration;
          tf4 = tf3*pDuration;
          tf5 = tf4*pDuration;

          for (unsigned short i=0; i<sizeOfVector; i++)
          {
            a3.push_back(10.0f/tf3*D.at(i));
            a4.push_back(-15.0f/tf4*D.at(i));
            a5.push_back(6.0f/tf5*D.at(i));
          }

          NbSample = (int)(pDuration/pSampleTime);

          for (unsigned short i=0; i<NbSample; i++)
          {
            t = pSampleTime*(i+1);
            t3 = t*t*t;
            t4 = t3*t;
            t5 = t4*t;
            for (unsigned short j=0; j<sizeOfVector; j++)
            {
              tmpInterpolation.push_back(pStartVal[j] + a3[j]*t3 + a4[j]*t4 + a5[j]*t5);
            }
            returnInterpolation.push_back(tmpInterpolation);
            tmpInterpolation.clear();
          }

          return returnInterpolation;
        }


        void Interpolate(
          int         pNbSample,
          const float pStartVal,
          const float pEndVal,
          const float pSampleTime,
          float*      pResultArray)
        {
          float duration;
          float tf3,tf4,tf5;
          float t,t3,t4,t5;
          float D;
          float a3,a4,a5; //Interpolation Coefficient

          if (pResultArray == NULL)
            return;

          if (pNbSample < 1)
          {
            std::cout << "WARNING : Number of Sample to small or Negative \nChange NbSample to 1 " << std::endl;
            pNbSample = 1;
          }

          // Compute Interpolation Coefficient
          D = pEndVal - pStartVal;
          duration = pNbSample*pSampleTime;
          tf3 = duration*duration*duration;
          tf4 = tf3*duration;
          tf5 = tf4*duration;
          a3 = 10.0f/tf3*D;
          a4 = -15.0f/tf4*D;
          a5 = 6.0f/tf5*D;

          for (unsigned short i=0; i<pNbSample; i++)
          {
            t = pSampleTime*(i+1);
            t3 = t*t*t;
            t4 = t3*t;
            t5 = t4*t;
            pResultArray[i] = pStartVal+a3*t3+a4*t4+a5*t5;
          }
        }

      } // end namespace Smooth
    } // end namespace Interpolation
  } // end namespace Math
} // end namespace AL

