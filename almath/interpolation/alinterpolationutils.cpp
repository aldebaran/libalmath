/**
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved \n
*
*/

#include <almath/interpolation/alinterpolationutils.h>

#include "math.h"

#include <alcore/alerror.h>
#include <almath/types/alvelocity6d.h>
#include <almath/tools/altransformhelpers.h>

namespace AL
{
  namespace Math
  {

    void computeFinalTimeInterpolation(
      const float& pPeriod,
      float&       pFinalTime)
    {

      pFinalTime = pPeriod*floor(pFinalTime/pPeriod + 0.5f);
      if (pFinalTime < 0.00001f)
      {
        pFinalTime = pPeriod;
      }

      //pFinalTime = 0.02f*floor(50.0f*pFinalTime + 0.5f);
      //if (pFinalTime < 0.00001f)
      //{
      //  pFinalTime = 0.02f;
      //}
    }


    float getTimeFinalJoint(
      const float& pPointInit,
      const float& pPointFinal,
      const float& pVelocityInit,
      const float& pVelocityFinal,
      const float& pVelocityMaxAbs,
      const float& pPeriod)
    {
      float tFinal;

      if (pVelocityMaxAbs <= 0.0f)
      {
        throw ALERROR(
          "ALMath",
          "ALInterpolationArticular",
          "pVelocityMaxAbs must be strictly positive.");
      }

      float tmpVelocityInit  = pVelocityInit;

      if (tmpVelocityInit > pVelocityMaxAbs)
      {
        tmpVelocityInit = pVelocityMaxAbs;
      }

      if (tmpVelocityInit < -pVelocityMaxAbs)
      {
        tmpVelocityInit = -pVelocityMaxAbs;
      }

      float tmpVelocityFinal = pVelocityFinal;

      if (tmpVelocityFinal > pVelocityMaxAbs)
      {
        tmpVelocityFinal = pVelocityMaxAbs;
      }

      if (tmpVelocityFinal < -pVelocityMaxAbs)
      {
        tmpVelocityFinal = -pVelocityMaxAbs;
      }

      float VMax = pVelocityMaxAbs;

      // Compute the sign of VMax
      float s2 = 3.0f*(pPointFinal - pPointInit) - 2.0f*tmpVelocityInit - tmpVelocityFinal;
      float s3 = 2.0f*(pPointInit - pPointFinal) + tmpVelocityInit + tmpVelocityFinal;

      float tMax = -s2/(3.0f*s3);
      float VMaxSign  = tmpVelocityInit + 2.0f*s2*tMax + 3.0f*s3*powf(tMax, 2);

      if (VMaxSign < 0.0f)
      {
        VMax = -VMax;
      }
      // end compute sign of VMax

      float tempo = 3.0f*(tmpVelocityFinal + tmpVelocityInit)*VMax +
          powf(tmpVelocityFinal, 2) +
          tmpVelocityInit*tmpVelocityFinal + powf(tmpVelocityInit, 2);

      if ( (tmpVelocityInit == 0.0f) && (tmpVelocityFinal == 0.0f) )
      {
        tFinal = 3.0f*(pPointFinal - pPointInit)/(2.0f*VMax);
      }
      else if ( (tempo == 0.0f) && (tmpVelocityInit!=0.0f) )
      {
        tFinal = 3.0f*(pPointFinal - tmpVelocityInit)/tmpVelocityInit;
      }
      else
      {
        float tmpSqrt = 0.0f;
        if (VMaxSign > 0.0f)
        {
          tmpSqrt = powf(VMax,2) -
                    (tmpVelocityFinal + tmpVelocityInit)*VMax + tmpVelocityInit*tmpVelocityFinal;

          if (tmpSqrt > 0.0f)
          {
            tFinal = ( 3.0f*(pPointFinal-pPointInit)*(VMax + tmpVelocityFinal + tmpVelocityInit -
              sqrt(tmpSqrt)))/tempo;
          }
          else
          {
            tFinal = 3.0f*(pPointFinal - pPointInit)/(2.0f*VMax);
          }

        }
        else
        {
          tmpSqrt = powf(VMax, 2) -
                    (tmpVelocityFinal + tmpVelocityInit)*VMax + tmpVelocityInit*tmpVelocityFinal;

          if (tmpSqrt > 0.0f)
          {
            tFinal = ( 3.0f*(pPointFinal-pPointInit)*(
              sqrt(tmpSqrt) +
              VMax +
              tmpVelocityFinal +
              tmpVelocityInit ))/tempo;
          }
          else
          {
            tFinal = 3.0f*(pPointFinal - pPointInit)/(2.0f*VMax);
          }

        }
      }

      // CK: FIXME why do this interpolations not reach the desired angles? need a multiple of dt?
      // hack to make a positive time .... cyrille? ca marche comme ca
      if (tFinal < 0.0f)
      {
        tFinal = -tFinal;
      }

      AL::Math::computeFinalTimeInterpolation(pPeriod, tFinal);

      return tFinal;
    } // end getTimeFinal


      float getTimeFinalCartesian(
        const AL::Math::Transform& pHInit,
        const AL::Math::Transform& pHFinal,
        const float&                 pMaxVelocity,
        const float&                 pPeriod)
      {
        float tFinal = 0.0f;


        AL::Math::Velocity6D pVelocityIn  = AL::Math::Velocity6D();
        AL::Math::Velocity6D pVelocity = AL::Math::Velocity6D();

        AL::Math::TransformLogarithme(AL::Math::pinv(pHInit)*pHFinal, pVelocityIn);

        AL::Math::ChangeRepereVelocity6D(
          pHInit,         //const AL::Math::Transform& pH,
          pVelocityIn,    //const AL::Math::Velocity6D&  pVIn,
          pVelocity);  //AL::Math::Velocity6D&        pVOut


        // 0.15 m.s-1, 2*pi rad.s-1
        AL::Math::Velocity6D pVelocityMax = AL::Math::Velocity6D(
            0.15f,
            0.15f,
            0.15f,
            6.283185f,
            6.283185f,
            6.283185f);

        pVelocityMax = pMaxVelocity*pVelocityMax;

        if (fabsf(pVelocity.xd)/pVelocityMax.xd > tFinal)
        {
          tFinal = fabsf(pVelocity.xd)/pVelocityMax.xd;
        }

        if (fabsf(pVelocity.yd)/pVelocityMax.yd > tFinal)
        {
          tFinal = fabsf(pVelocity.yd)/pVelocityMax.yd;
        }

        if (fabsf(pVelocity.zd)/pVelocityMax.zd > tFinal)
        {
          tFinal = fabsf(pVelocity.zd)/pVelocityMax.zd;
        }

        if (fabsf(pVelocity.wxd)/pVelocityMax.wxd > tFinal)
        {
          tFinal = fabsf(pVelocity.wxd)/pVelocityMax.wxd;
        }

        if (fabsf(pVelocity.wyd)/pVelocityMax.wyd > tFinal)
        {
          tFinal = fabsf(pVelocity.wyd)/pVelocityMax.wyd;
        }

        if (fabsf(pVelocity.wzd)/pVelocityMax.wzd > tFinal)
        {
          tFinal = fabsf(pVelocity.wzd)/pVelocityMax.wzd;
        }

        AL::Math::computeFinalTimeInterpolation(
          pPeriod,
          tFinal);

        return tFinal;
      } // end xGetFinalTimeCartesianGoTo


  } // namespace Math
} // namespace AL

