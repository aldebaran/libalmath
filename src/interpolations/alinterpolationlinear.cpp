/**
* @author Jerome Vuarand
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved \n
*/

#include <almath/interpolations/alinterpolationlinear.h>

#include "math.h"
#include <limits>
#include <iostream>
#include <cstdlib>

#include <almath/tools/almath.h>

using std::vector;

namespace AL
{
  namespace Math
  {
    namespace Interpolation
    {
      namespace Linear
      {

        std::vector<float> Interpolate(
          float pDuration,
          float pStartVal,
          float pEndVal,
          float pMaxAngleChange,
          float pSampleDuration)
        {
          vector<float> returnInterpolation;
          int numSamples;
          float Speed;

          if (pDuration > 0.0f)
          {
            // "Normal" behaviour
            if (pDuration < pSampleDuration)
            {
              std::cout << "WARNING : lowest Interpolation Duration \nChange Duration to the Robot Cycle " << std::endl;
              pDuration = pSampleDuration;
            }
            numSamples = (int)floorf(pDuration/pSampleDuration);


            Speed = (pEndVal-pStartVal)/numSamples;
            if ( fabsf(Speed) > pMaxAngleChange )
            {
              Speed = Sign(Speed)*pMaxAngleChange;
              std::cout << "WARNING : Velocity Joint Limit reached" << std::endl;
            }
          }
          else
          {
            // With speed behaviour
            Speed = Sign(pEndVal-pStartVal)*(pMaxAngleChange*(-pDuration));
            numSamples = (int)ceilf((pEndVal-pStartVal)/(Speed));
            Speed = (pEndVal-pStartVal)/numSamples;
          }

          returnInterpolation.reserve(numSamples);
          for (unsigned short i=0; i<numSamples; i++)
          {
            returnInterpolation.push_back(pStartVal+(i+1)*Speed);
          }

          return returnInterpolation;
        }


        std::vector<float> Interpolate(
            int   pNumSamples,
            float pStartVal,
            float pEndVal)
        {
          std::vector<float> returnInterpolation;
          float valDiff;

          if (pNumSamples < 1)
          {
            std::cout << "WARNING : Number of Sample to small or Negative \nChange numSamples to 1 " << std::endl;
            pNumSamples = 1;
          }

          valDiff = pEndVal-pStartVal;

          returnInterpolation.reserve(pNumSamples);
          float valDiffOvernumSamples =  valDiff/pNumSamples;

          for (unsigned short i=0; i<pNumSamples; i++)
          {
            returnInterpolation.push_back(pStartVal+(i+1)*valDiffOvernumSamples);
          }

          return returnInterpolation;
        }


        std::vector<float> Interpolate(
            int pNumSamples,
            float pStartVal,
            float pEndVal,
            float pMaxAngleChange)
        {
          vector<float> returnInterpolation;
          float Speed;

          if (pNumSamples < 1)
          {
            std::cout << "WARNING : Number of Sample to small or Negative \nChange numSamples to 1 " << std::endl;
            pNumSamples = 1;
          }

          Speed = (pEndVal-pStartVal)/pNumSamples;
          if ( fabsf(Speed) > pMaxAngleChange )
          {
            Speed = Sign(Speed)*pMaxAngleChange;
            std::cout << "WARNING : Velocity Joint Limit reached" << std::endl;
          }

          returnInterpolation.reserve(pNumSamples);
          for (unsigned short i=0; i<pNumSamples; i++)
          {
            returnInterpolation.push_back(pStartVal+(i+1)*Speed);
          }

          return returnInterpolation;
        }


        std::vector<vector<float> > Interpolate(
            float              pDuration,
            std::vector<float> pStartVal,
            std::vector<float> pEndVal,
            float              pSampleDuration)
        {
          std::vector<vector<float> > returnInterpolation;
          std::vector<float> tmpInterpolation;
          int numSamples;
          int sizeOfVector;
          std::vector<float> valDiff;

          if (pStartVal.size() != pEndVal.size() )
          {
            // TODO : Make an ERROR
            return returnInterpolation;
          }
          else
          {
            sizeOfVector = pStartVal.size();
          }

          if (pDuration < pSampleDuration)
          {
            std::cout << "WARNING : lowest Interpolation Duration \nChange Duration to the Robot Cycle " << std::endl;
            pDuration = pSampleDuration;
          }
          numSamples = (int)floorf(pDuration/pSampleDuration);

          valDiff.reserve(sizeOfVector);
          for(unsigned short i=0; i<sizeOfVector; i++ )
          {
            valDiff.push_back(pEndVal[i]-pStartVal[i]);
          }

          returnInterpolation.reserve(numSamples);
          float oneOvernumSamples = 1.0f/numSamples;
          for(unsigned short i=0; i<numSamples; i++ )
          {
            tmpInterpolation.reserve(sizeOfVector);
            for(unsigned short j=0; j<sizeOfVector; j++ )
            {
              tmpInterpolation.push_back(pStartVal[j]+(i+1)*valDiff[j]*oneOvernumSamples);
            }
            returnInterpolation.push_back(tmpInterpolation);
            tmpInterpolation.clear();
          }

          return returnInterpolation;
        }


        std::vector<std::vector<float> > Interpolate(
            float              pDuration,
            std::vector<float> pStartVal,
            std::vector<float> pEndVal,
            std::vector<float> pMaxAngleChange,
            float              pSampleDuration)
        {
          std::vector<vector<float> > returnInterpolation;
          std::vector<float> tmpInterpolation;
          int tmpNumSamples = 0;
          int numSamples = 0;
          int sizeOfVector;
          std::vector<float> Speed;

          if(pStartVal.size() != pEndVal.size() )
          {
            // TODO : Make an ERROR
            return returnInterpolation;
          }
          else
          {
            sizeOfVector = pStartVal.size();
          }

          if (pDuration > 0.0f)
          {
            // "Normal" behaviour
            if (pDuration < pSampleDuration)
            {
              std::cout << "WARNING : lowest Interpolation Duration \nChange Duration to the Robot Cycle " << std::endl;
              pDuration = pSampleDuration;
            }
            numSamples = (int)floorf(pDuration/pSampleDuration);

            Speed.reserve(sizeOfVector);
            for (unsigned short i=0; i<sizeOfVector; i++ )
            {
              Speed.push_back((pEndVal[i]-pStartVal[i])/numSamples);
              if ( fabsf(Speed[i]) > pMaxAngleChange[i] )
              {
                Speed[i] = Sign(Speed[i])*(pMaxAngleChange[i]-0.0001f);
                std::cout << "WARNING : Velocity Joint Limit reached" << std::endl;
              }
            }
          }
          else
          {
            // With speed behaviour
            for (unsigned short i=0; i<sizeOfVector; i++)
            {
              Speed.push_back(Sign(pEndVal[i]-pStartVal[i])*(pMaxAngleChange[i]*(-pDuration)));
              tmpNumSamples = (int)ceilf((pEndVal[i]-pStartVal[i])/(Speed[i]));
              if (tmpNumSamples>numSamples)
                numSamples = tmpNumSamples;
            }
            for(unsigned short i=0; i<sizeOfVector; i++)
            {
              Speed[i] = (pEndVal[i]-pStartVal[i])/numSamples;
            }
          }

          returnInterpolation.reserve(numSamples);
          for (unsigned short i=0; i<numSamples; i++)
          {
            tmpInterpolation.reserve(sizeOfVector);
            for(unsigned short j=0; j<sizeOfVector; j++)
            {
              tmpInterpolation.push_back(pStartVal[j]+(i+1)*Speed[j]);
            }
            returnInterpolation.push_back(tmpInterpolation);
            tmpInterpolation.clear();
          }

          return returnInterpolation;
        }


      }
    }
  }
}
