/**
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved \n
*
* http://math.fullerton.edu/mathews/n2003/CubicSplinesMod.html
*/

#include <almath/interpolations/alinterpolationarticular.h>

#include "math.h"
#include <algorithm> // for copy
#include <limits>
#include <cstdlib>
#include <stdexcept>
#include <almath/interpolations/alinterpolationutils.h>

using std::vector;

namespace AL
{
  namespace Math
  {
    namespace Interpolation
    {

      //const double epsilon = 1e-5;
      const float epsilon = 0.00001f;

      // begin constructeur
      ALInterpolationArticular::ALInterpolationArticular():
        fNbEchantillon(5),
        fNbPoints(0),
        fNbAllInterpolation(0),
        fNumTimesCalled(0),
        fVelocityInit(0.0f),
        fVelocityFinal(0.0f),
        ft(0.0f),
        fTmp(0.0f),
        fPeriod(0.0f),
        fIsFinished(true),
        fTime(NULL),
        fPoint(NULL),
        fh(NULL),
        fd(NULL),
        fm(NULL),
        fa(NULL),
        fb(NULL),
        fc(NULL),
        fv(NULL),
        fs1(NULL),
        fs2(NULL),
        fs3(NULL),
        fs4(NULL)
      {
        xCreateFloat(fNbEchantillon);
      }

      // INIT With Velocity Saturation
      void ALInterpolationArticular::Init(
        const float& pTimeInit,
        const float& pTimeFinal,
        const float& pPointInit,
        const float& pPointFinal,
        const float& pVelocityInit,
        const float& pVelocityFinal,
        const float& pVelocityMaxAbs,
        const bool&  pChangeTimeFinal,
        const float& pPeriod)
      {
        if (pVelocityMaxAbs <= 0.0f)
        {
          throw std::invalid_argument(
            "ALMath: ALInterpolationArticular pVelocityMaxAbs must be strictly positive.");
        }

        if (pTimeFinal <= pTimeInit)
        {
          throw std::invalid_argument(
            "ALMath: ALInterpolationArticular pTimeFinal <= pTimeInit");
        }

        float TimeFinalTmp     = pTimeFinal;
        float PointFinalTmp    = pPointFinal;
        float VelocityInitTmp  = pVelocityInit;
        float VelocityFinalTmp = pVelocityFinal;

        if (pVelocityInit >= 0.0f)
        {
          if (pVelocityInit > pVelocityMaxAbs)
          {
            VelocityInitTmp = pVelocityMaxAbs;
          }
        }
        else
        {
          if (-pVelocityInit > pVelocityMaxAbs)
          {
            VelocityInitTmp = -pVelocityMaxAbs;
          }
        }

        if (pVelocityFinal >= 0.0f)
        {
          if (pVelocityFinal > pVelocityMaxAbs)
          {
            VelocityFinalTmp = pVelocityMaxAbs;
          }
        }
        else
        {
          if (-pVelocityFinal > pVelocityMaxAbs)
          {
            VelocityFinalTmp = -pVelocityMaxAbs;
          }
        }

        // Compute tMax
        float a    = pTimeInit;
        float b    = pTimeFinal;
        float x0   = pPointInit;
        float x1   = pPointFinal;
        float dx0  = VelocityInitTmp;
        float dx1  = VelocityFinalTmp;

        float iba  = 1.0f/(b-a);
        float iba2 = powf(iba,2);

        float s1   = iba2*( 6.0f*a*b*iba*(x0 - x1) + (b*(b+2.0f*a))*dx0 + (a*(2.0f*b+a))*dx1 );
        float s2   = iba2*( 3.0f*(b+a)*iba*(x1-x0) - (2.0f*b+a)*dx0 - (b+2.0f*a)*dx1 );
        float s3   = iba2*( 2.0f*iba*(x0 - x1) + dx0 + dx1 );

        float tMax = -s2/(3.0f*s3);

        if ( (tMax >= pTimeFinal) || (tMax <= pTimeInit) )
        {
          if ( fabsf(VelocityInitTmp) > fabsf(VelocityFinalTmp) )
          {
            tMax = pTimeInit;
          }
          else
          {
            tMax = pTimeFinal;
          }
        }

        // Compute VMax
        float VMax  = s1 + 2.0f*s2*tMax + 3.0f*s3*powf(tMax,2);

        float VelocityMaxTmp = 0.0f;

        if (VMax < 0.0f)
        {
          VelocityMaxTmp = -pVelocityMaxAbs;
        }
        else
        {
          VelocityMaxTmp = pVelocityMaxAbs;
        }

        float tmpSqrt = 0.0f;
        if ( fabsf(VelocityMaxTmp) < fabsf(VMax))
        {

          if (pChangeTimeFinal == true)
          {
            float tempo = 3.0f*(VelocityFinalTmp + VelocityInitTmp)*VelocityMaxTmp + powf(VelocityFinalTmp,2) +
              VelocityInitTmp*VelocityFinalTmp + powf(VelocityInitTmp,2);

            if ( (VelocityInitTmp == 0) && (VelocityFinalTmp == 0) )
            {
              TimeFinalTmp = (3.0f*pPointFinal - 3.0f*pPointInit + 2.0f*pTimeInit*VelocityMaxTmp)/(2.0f*VelocityMaxTmp);
            }
            else if ( tempo == 0.0f )
            {
              TimeFinalTmp = pTimeFinal;
            }
            else
            {
              if (VelocityMaxTmp > 0.0f)
              {
                tmpSqrt = fabsf(powf(VelocityMaxTmp, 2) -
                                (VelocityFinalTmp + VelocityInitTmp)*VelocityMaxTmp +
                                VelocityInitTmp*VelocityFinalTmp);

                TimeFinalTmp = ( 3.0f*(pPointFinal - pPointInit)*
                  (VelocityMaxTmp + VelocityFinalTmp + VelocityInitTmp -
                  sqrt(tmpSqrt) )+
                  3.0f*pTimeInit*(VelocityFinalTmp + VelocityInitTmp)*VelocityMaxTmp +
                  pTimeInit*(powf(VelocityFinalTmp,2) +
                  VelocityInitTmp*VelocityFinalTmp + powf(VelocityInitTmp,2)) )/tempo;
              }
              else
              {
                tmpSqrt = fabsf(powf(VelocityMaxTmp, 2) -
                                (VelocityFinalTmp + VelocityInitTmp)*VelocityMaxTmp +
                                VelocityInitTmp*VelocityFinalTmp);

                TimeFinalTmp = ( 3.0f*(pPointFinal - pPointInit)*
                  ( sqrt(tmpSqrt) +
                  VelocityMaxTmp + VelocityFinalTmp + VelocityInitTmp ) +
                  3.0f*pTimeInit*(VelocityFinalTmp + VelocityInitTmp)*VelocityMaxTmp +
                  pTimeInit*(powf(VelocityFinalTmp,2) + VelocityInitTmp*VelocityFinalTmp +
                             powf(VelocityInitTmp,2)) )/tempo;
              }
            }

            if ( (TimeFinalTmp < pTimeFinal)||(TimeFinalTmp <= pTimeInit) )
            {
              TimeFinalTmp = pTimeFinal;
            }

          }
          else // Change final position
          {
            if (VelocityMaxTmp > 0.0f)
            {
              tmpSqrt = fabsf(powf(VelocityMaxTmp,2) -
                              (VelocityFinalTmp + VelocityInitTmp)*VelocityMaxTmp +
                              VelocityInitTmp*VelocityFinalTmp);

              PointFinalTmp = pPointInit +
                0.3333333333f*(pTimeFinal - pTimeInit)*
                ( sqrt(tmpSqrt) +
                VelocityMaxTmp + VelocityFinalTmp + VelocityInitTmp );
            }
            else
            {
              tmpSqrt = fabsf(powf(VelocityMaxTmp,2) -
                              ( VelocityFinalTmp + VelocityInitTmp )*VelocityMaxTmp +
                              VelocityInitTmp*VelocityFinalTmp);

              PointFinalTmp = pPointInit +
                0.3333333333f*(pTimeFinal - pTimeInit)*
                ( - sqrt(tmpSqrt) +
                VelocityMaxTmp + VelocityFinalTmp + VelocityInitTmp );
            }
          }
        }

        std::vector<float> pTime;
        pTime.push_back(pTimeInit);
        pTime.push_back(TimeFinalTmp);

        std::vector<float> pPoint;
        pPoint.push_back(pPointInit);
        pPoint.push_back(PointFinalTmp);

        std::vector<float> pVelocity;
        pVelocity.push_back(VelocityInitTmp);
        pVelocity.push_back(VelocityFinalTmp);

        ALInterpolationArticular::Init(pTime, pPoint, pVelocity, false, pPeriod);
      }

      // Basic INIT
      void ALInterpolationArticular::Init(
        const float& pTimeInit,
        const float& pTimeFinal,
        const float& pPointInit,
        const float& pPointFinal,
        const float& pVelocityInit,
        const float& pVelocityFinal,
        const float& pPeriod)
      {
        std::vector<float> pTime;
        pTime.push_back(pTimeInit);
        pTime.push_back(pTimeFinal);

        std::vector<float> pPoint;
        pPoint.push_back(pPointInit);
        pPoint.push_back(pPointFinal);

        std::vector<float> pVelocity;
        pVelocity.push_back(pVelocityInit);
        pVelocity.push_back(pVelocityFinal);

        ALInterpolationArticular::Init(pTime, pPoint, pVelocity, false, pPeriod);
      }

      // INIT Without velocity
      void ALInterpolationArticular::Init(
        const std::vector<float>& pTime,
        const std::vector<float>& pPoint,
        const float&              pPeriod)
      {
        std::vector<float> pVelocity;
        pVelocity.push_back(0.0f); // why?
        pVelocity.push_back(0.0f);

        ALInterpolationArticular::Init(pTime, pPoint, pVelocity, false, pPeriod);
      }


      // MAIN INIT
      void ALInterpolationArticular::Init(
        const std::vector<float>& pTime,
        const std::vector<float>& pPoint,
        const std::vector<float>& pVelocity,
        const bool&               pIsHotStart,
        const float&              pPeriod)
      {
        if (pVelocity.size() != 2)
        {
          throw std::invalid_argument(
            "ALMath: ALInterpolationArticular pVelocity.size() != 2");
        }

        if (pTime.size() < 2)
        {
          throw std::invalid_argument(
            "ALMath: ALInterpolationArticular pTime.size() < 2");
        }

        if (pPoint.size() < 2)
        {
          throw std::invalid_argument(
            "ALMath: ALInterpolationArticular pPoint.size() < 2");
        }

        if (pTime.size() != pPoint.size())
        {
          throw std::invalid_argument(
            "ALMath: ALInterpolationArticular pTime.size() != pPoint.size()");
        }

        for (unsigned int i=0; i<pTime.size()-1; i++)
        {
          if (pTime[i] >= pTime[i+1])
          {
            throw std::invalid_argument(
              "ALMath: ALInterpolationArticular pTime[i] >= pTime[i+1]");
          }
        }

        fPeriod = pPeriod;

        fIsFinished = false;
        if (!pIsHotStart)
        {
          fNumTimesCalled = 0;
        }

        if ( pTime.size() > fNbEchantillon )
        {
          fNbEchantillon = pTime.size() + 1;
          xDestroyFloat();
          xCreateFloat(fNbEchantillon);
        }

        // Mise a zeros
        for (unsigned int i=0; i<fNbEchantillon; i++)
        {
          fTime[i]  = 0.0f;
          fPoint[i] = 0.0f;
          fh[i]     = 0.0f;
          fd[i]     = 0.0f;
          fm[i]     = 0.0f;
          fa[i]     = 0.0f;
          fb[i]     = 0.0f;
          fc[i]     = 0.0f;
          fv[i]     = 0.0f;
          fs1[i]    = 0.0f;
          fs2[i]    = 0.0f;
          fs3[i]    = 0.0f;
          fs4[i]    = 0.0f;
        }

        // sauvegarde Input
        fNbPoints = pTime.size() - 1;
        for (unsigned int i = 0 ; i < pTime.size(); i++)
        {
          fTime[i]  = pTime[i];
          fPoint[i] = pPoint[i];
        }

        AL::Math::computeFinalTimeInterpolation(fPeriod, fTime[fNbPoints]);

        fVelocityInit  = pVelocity[0];
        fVelocityFinal = pVelocity[1];

        if (fNbPoints == 1)
        {
          fa[0] = 0.0f;
          fb[0] = fTime[1] - fTime[0];

          float b = fTime[1] - fTime[0];
          float iba  = 1.0f/b;
          float iba2 = powf(iba, 2);

          fs1[0] = fPoint[0];

          fs2[0] = fVelocityInit;

          fs3[0] = iba2*( 3.0f*(fPoint[1] - fPoint[0]) - b*(2.0f*fVelocityInit + fVelocityFinal) );

          fs4[0] = iba2*( 2.0f*iba*(fPoint[0] - fPoint[1]) + fVelocityInit + fVelocityFinal );

          //float a2   = powf(fTime[0], 2);
          //float b2   = powf(fTime[1], 2);
          //float iba  = 1.0f/(fTime[1] - fTime[0]);
          //float iba2 = powf(iba, 2);

          //fs1[0] = iba2*( b2*(fTime[1] - 3.0f*fTime[0])*iba*fPoint[0] +
          //  a2*(3.0f*fTime[1] - fTime[0])*iba*fPoint[1] - fTime[0]*b2*fVelocityInit - a2*fTime[1]*fVelocityFinal );

          //fs2[0] = iba2*( 6.0f*fTime[0]*fTime[1]*iba*(fPoint[0] - fPoint[1]) +
          //  (fTime[1]*(fTime[1] + 2.0f*fTime[0]))*fVelocityInit + (fTime[0]*(2.0f*fTime[1] + fTime[0]))*fVelocityFinal );

          //fs3[0] = iba2*( 3.0f*(fTime[1] + fTime[0])*iba*(fPoint[1] - fPoint[0]) -
          //  (2.0f*fTime[1] + fTime[0])*fVelocityInit - (fTime[1] + 2.0f*fTime[0])*fVelocityFinal );

          //fs4[0] = iba2*( 2.0f*iba*(fPoint[0] - fPoint[1]) + fVelocityInit + fVelocityFinal );
        }
        else
        {
          // Differences
          fh[0] = fTime[1] - fTime[0];
          fd[0] = (fPoint[1] - fPoint[0])/fh[0];

          for (unsigned int i = 1 ; i < fNbPoints ; i++)
          {
            fh[i]   = fTime[i+1] - fTime[i];
            fd[i]   = ( fPoint[i+1] - fPoint[i] )/fh[i];
            fa[i-1] = fh[i];
            fb[i-1] = 2.0f*( fh[i-1] + fh[i] );
            fc[i-1] = fh[i];
            fv[i-1] = 6.0f*( fd[i] - fd[i-1] );
          }

          // Tridiagonal
          fb[0] = fb[0] - 0.5f*fh[0];
          fv[0] = fv[0] - 3.0f*(fd[0] - fVelocityInit);

          fb[fNbPoints-2] = fb[fNbPoints-2] - 0.5f*fh[fNbPoints-1];
          fv[fNbPoints-2] = fv[fNbPoints-2] - 3.0f*( fVelocityFinal - fd[fNbPoints-1] );

          if (fNbPoints>2)
          {
            for (unsigned int i = 1 ; i < (fNbPoints-1) ; i++)
            {
              fTmp  = fa[i-1]/fb[i-1];
              fb[i] = fb[i] - fTmp*fc[i-1];
              fv[i] = fv[i] - fTmp*fv[i-1];
            }
          }

          fm[fNbPoints-1] = fv[fNbPoints-2]/fb[fNbPoints-2];

          if (fNbPoints>2)
          {
            for (int i = (fNbPoints-3) ; i > -1 ; i--)
            {
              fm[i+1] = ( fv[i] - fc[i]*fm[i+2] )/fb[i];
            }
          }

          // Compute Coeff
          fm[0]         = 3.0f*( fd[0] - fVelocityInit )/fh[0] - 0.5f*fm[1];
          fm[fNbPoints] = 3.0f*( fVelocityFinal - fd[fNbPoints-1] )/fh[fNbPoints-1] -
                          0.5f*fm[fNbPoints-1];

          for (unsigned int i = 0 ; i < fNbPoints ; i++)
          {
            fs1[i] = fPoint[i]; //pPoint.at(i);
            fs2[i] = fd[i] - 0.16666666666667f*fh[i]*( 2.0f*fm[i] + fm[i+1] );
            fs3[i] = 0.5f*fm[i];
            fs4[i] = 0.16666666666667f*(fm[i+1] - fm[i])/fh[i];

          }
        }
      }// END MAIN INIT


      void ALInterpolationArticular::getCurrentInterpolation(
        const float&                     tCurrent,
        AL::Math::PositionAndVelocity& pOut)
      {
        fNumTimesCalled++;
        int i = 0;
        while ( fTime[i] <= (tCurrent-epsilon) )
        {
          i ++;
        }

        i = i-1;

        if (i == (int)fNbPoints)
        {
          i = i-1;
        }
        if (i < 0)
        {
          i = 0;
        }

        ft  = tCurrent - fTime[i];

        if (tCurrent >= fTime[fNbPoints] - epsilon)
        {
          pOut.q = fPoint[fNbPoints];
          pOut.dq = fVelocityFinal;
        }
        else
        {
          pOut.q  = ( ( fs4[i]*ft + fs3[i] ) * ft + fs2[i] ) * ft + fs1[i];
          pOut.dq = ( 3.0f*fs4[i]*ft + 2.0f*fs3[i] ) * ft + fs2[i];
        }

      } // end getCurrentInterpolation


      AL::Math::PositionAndVelocity ALInterpolationArticular::getCurrentInterpolation(
          const float& tCurrent)
      {
        PositionAndVelocity tSolution;
        ALInterpolationArticular::getCurrentInterpolation(tCurrent,tSolution);
        return tSolution;
      }


      // give all PositionAndVelocity for all interpolation
      std::vector<AL::Math::PositionAndVelocity> ALInterpolationArticular::getAllInterpolation(
          const float& dt_step)
      {
        std::vector<AL::Math::PositionAndVelocity> solution;

        //Round to nearest integer
        fNbAllInterpolation = 0;
        float i = dt_step;
        while (!isFinished(i))
        {
          fNbAllInterpolation ++;
          i = i + dt_step;
        }

        AL::Math::PositionAndVelocity qdqNull;
        solution.assign(fNbAllInterpolation, qdqNull); // n cellule de 6D vector

        // boucle temporelle d'interpolation
        i = dt_step;
        unsigned int jj = 0;
        fIsFinished = false; // sinon, il est mis a true par la precedente boucle isFinished
        while (!isFinished(i))
        {
          ALInterpolationArticular::getCurrentInterpolation(i, solution[jj]); // solution.at(jj)
          jj++;
          i = i + dt_step;
        }

        return solution;
      } // end getAllInterpolation


      bool ALInterpolationArticular::isFinished(const float& tCurrent)
      {
        if ( fIsFinished
          || (tCurrent >= (fTime[fNbPoints]+epsilon))
          || (tCurrent <= (fTime[0]-epsilon)) )
        {
          fIsFinished = true;
        }
        return fIsFinished;
      } // end isFinished


      ALInterpolationArticular::~ALInterpolationArticular()
      {
        xDestroyFloat();
      } // end destructeur


      void ALInterpolationArticular::xCreateFloat(const unsigned int& nEchantillon)
      {
        fTime  = new float[nEchantillon];
        fPoint = new float[nEchantillon];

        fh     = new float[nEchantillon];

        fd     = new float[nEchantillon];
        fm     = new float[nEchantillon];

        fa     = new float[nEchantillon];
        fb     = new float[nEchantillon];
        fc     = new float[nEchantillon];
        fv     = new float[nEchantillon];

        fs1    = new float[nEchantillon];
        fs2    = new float[nEchantillon];
        fs3    = new float[nEchantillon];
        fs4    = new float[nEchantillon];
      }

#define DEL_ARRAY_IF_NOT_NULL(x) if (x!=NULL){delete[] x; x = NULL;}

      void ALInterpolationArticular::xDestroyFloat(void)
      {

        DEL_ARRAY_IF_NOT_NULL(fTime);
        DEL_ARRAY_IF_NOT_NULL(fPoint);
        DEL_ARRAY_IF_NOT_NULL(fh);
        DEL_ARRAY_IF_NOT_NULL(fd);
        DEL_ARRAY_IF_NOT_NULL(fm);

        DEL_ARRAY_IF_NOT_NULL(fa);
        DEL_ARRAY_IF_NOT_NULL(fb);
        DEL_ARRAY_IF_NOT_NULL(fc);
        DEL_ARRAY_IF_NOT_NULL(fv);

        DEL_ARRAY_IF_NOT_NULL(fs1);
        DEL_ARRAY_IF_NOT_NULL(fs2);
        DEL_ARRAY_IF_NOT_NULL(fs3);
        DEL_ARRAY_IF_NOT_NULL(fs4);
      }

    }
  }
}

