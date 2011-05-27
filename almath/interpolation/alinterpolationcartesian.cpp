/**
* @author Cyrille Collette
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved
*
* Milos Zefran and Vizay Kumar
* Two Methods for Interpolating Rigid Body Motions
* IEEE International Conference on Robotics and Automation
* 1998 pp.2922-2927
*/

#include <almath/interpolation/alinterpolationcartesian.h>

#include <stdexcept>
#include <almath/interpolation/alinterpolationutils.h>
#include <almath/tools/almath.h>
#include <almath/tools/altransformhelpers.h>

namespace AL
{
  namespace Math
  {
    namespace Interpolation
    {
      const float epsilon = 0.00001f;

      ALInterpolationCartesian::ALInterpolationCartesian() :
          fSpace(0),
          fNbAllInterpolation(0),
          fNumTimesCalled(0),
          fIsFinished(true),
          fAxisMask(AL::Math::AXIS_MASK_ALL),
          fPeriod(0.0f){}


      void ALInterpolationCartesian::Init(
        const std::vector<float>&                pTime,
        const std::vector<AL::Math::Position6D>& pXi,
        const float&                             pPeriod)
      {
        fAxisMask = AXIS_MASK_ALL;
        std::vector<AL::Math::Transform> pHi;
        for (unsigned int i = 0 ; i < pXi.size() ; i++)
        {
          pHi.push_back(TransformFromPosition6D(pXi.at(i)));
        }

        std::vector<AL::Math::Velocity6D> pVi;
        AL::Math::Velocity6D pV_null = AL::Math::Velocity6D();
        pVi.assign(2,pV_null);

        ALInterpolationCartesian::Init(pTime, pHi, pVi, pPeriod);
      }


      void ALInterpolationCartesian::Init(
        const std::vector<float>&                pTime,
        const std::vector<AL::Math::Position6D>& pXi,
        const std::vector<AL::Math::Velocity6D>& pVi,
        AL::Math::AXIS_MASK                      pAxisMask,
        const float&                             pPeriod)
      {
        fAxisMask = pAxisMask;

        std::vector<AL::Math::Transform> pHi;
        for (unsigned int i = 0 ; i < pXi.size() ; i++)
        {
          pHi.push_back(AL::Math::TransformFromPosition6D(pXi.at(i)));
        }

        ALInterpolationCartesian::Init(pTime, pHi, pVi, pPeriod);
      }


      void ALInterpolationCartesian::Init(
        const std::vector<float>&                pTime,
        const std::vector<AL::Math::Position6D>& pXi,
        const std::vector<AL::Math::Velocity6D>& pVi,
        const float&                             pPeriod)
      {
        fAxisMask = AXIS_MASK_ALL;
        std::vector<AL::Math::Transform> pHi;
        for (unsigned int i = 0 ; i < pXi.size() ; i++)
        {
          pHi.push_back(AL::Math::TransformFromPosition6D(pXi.at(i)));
        }

        ALInterpolationCartesian::Init(pTime, pHi, pVi, pPeriod);
      }


      void ALInterpolationCartesian::Init(
        const std::vector<float>&                pTime,
        const std::vector<AL::Math::Position6D>& pXi,
        AL::Math::AXIS_MASK                      pAxisMask,
        const float&                             pPeriod)
      {
        fAxisMask = pAxisMask;

        std::vector<AL::Math::Transform> pHi;
        for (unsigned int i = 0; i < pXi.size(); i++)
        {
          pHi.push_back(AL::Math::TransformFromPosition6D(pXi.at(i)));
        }

        std::vector<AL::Math::Velocity6D> pVi;
        AL::Math::Velocity6D pV_null = AL::Math::Velocity6D();
        pVi.assign(2, pV_null);

        ALInterpolationCartesian::Init(pTime, pHi, pVi, pPeriod);
      }


      void ALInterpolationCartesian::Init(
        const std::vector<float>&                 pTime,
        const std::vector<AL::Math::Transform>& pHi,
        AL::Math::AXIS_MASK                       pAxisMask,
        const float&                              pPeriod)
      {
        fAxisMask = pAxisMask;

        std::vector<AL::Math::Velocity6D> pVi;
        AL::Math::Velocity6D pV_null = AL::Math::Velocity6D();
        pVi.assign(2, pV_null);

        ALInterpolationCartesian::Init(pTime, pHi, pVi, pPeriod);
      }


      void ALInterpolationCartesian::Init(
        const std::vector<float>&                 pTime,
        const std::vector<AL::Math::Transform>& pHi,
        const std::vector<AL::Math::Velocity6D>&  pVi,
        AL::Math::AXIS_MASK                       pAxisMask,
        const float&                              pPeriod)
      {
        fAxisMask = pAxisMask;

        ALInterpolationCartesian::Init(pTime, pHi, pVi, pPeriod);
      }


      void ALInterpolationCartesian::Init(
        const std::vector<float>&                 pTime,
        const std::vector<AL::Math::Transform>&   pHi,
        const float&                              pPeriod)
      {
        fAxisMask = AXIS_MASK_ALL;
        std::vector<AL::Math::Velocity6D> pVi;
        AL::Math::Velocity6D pV_null = AL::Math::Velocity6D();

        pVi.assign(2,pV_null);

        ALInterpolationCartesian::Init(pTime, pHi, pVi, pPeriod);
      }


      // MAIN
      void ALInterpolationCartesian::Init(
        const std::vector<float>&                 pTime,
        const std::vector<AL::Math::Transform>&   pHi,
        const std::vector<AL::Math::Velocity6D>&  pVi,
        const float&                              pPeriod)
      {
        fPeriod = pPeriod;
        fNumTimesCalled = 0;
        fIsFinished = false;
        if ( (pTime.size() != pHi.size() ) || (pVi.size() != 2) )
        {
          throw std::invalid_argument(
            "ALMath: ALInterpolationCartesian Vectors have not same dimensions.");
        }

        fTime = pTime;
        f_Hi  = pHi;

        // Bonne orientation vitesse initiale
        ChangeRepereVelocity6D(
        pinv(f_Hi.at(0)), // const AL::Math::Transform& pH,
        pVi.at(0),        // const AL::Math::Velocity6D&  pVIn,
        fVInit);          // AL::Math::Velocity6D&        pVOut

        // Bonne orientation vitesse finale
        ChangeRepereVelocity6D(
        pinv(f_Hi.at(0)), // const AL::Math::Transform& pH,
        pVi.at(1),        // const AL::Math::Velocity6D&  pVIn,
        fVEnd);           // AL::Math::Velocity6D&        pVOut

        unsigned int nb_pt = f_Hi.size();
        xSetInitTransform(pHi.at(0));
        xSetFinalTransform(pHi.at(nb_pt-1));

        AL::Math::computeFinalTimeInterpolation(fPeriod, fTime.at(nb_pt-1));
        //xChangeFinalTime(fTime.at(nb_pt-1));

        // Initialisation
        fDHi = f_Hi; //homogenous matrix

        fAlpha.assign(nb_pt, fVNull);  // n cellule de 6D vector
        fZi.assign(nb_pt, fVNull);     // n cellule de 6D vector

        f_Xi.assign(nb_pt, fVNull);    // n   cellule de 6D vector
        f_Bi.assign(nb_pt-1, fVNull);  // n-1 cellule de 6D vector
        f_Ci.assign(nb_pt, fVNull);    // n   cellule de 6D vector
        f_Di.assign(nb_pt-1, fVNull);  // n-1 cellule de 6D vector

        for (unsigned int i = 0 ; i < nb_pt ; i++)
        {
          fDHi.at(i) = TransformInverse(f_Hi.at(0))*f_Hi.at(i); // Hi1i*f_Hi.at(i);
          TransformLogarithme(fDHi.at(i), f_Xi.at(i));
        }

        // Step 1 -- hi = pTime(2:n) - pTime(1:n-1);
        std::vector<float> hi;
        for (unsigned int i = 0 ; i < nb_pt - 1 ; i++)
        {
          hi.push_back ( fTime.at(i+1) - fTime.at(i) );
        }

        // Step 2
        DiffLog(f_Xi.at(0), fVInit, fXid1); //pVi.at(0)
        DiffLog(f_Xi.at(nb_pt-1), fVEnd, fXidn); //pVi.at(1)

        fAlpha.at(0) = 3.0f * ( (f_Xi.at(1) - f_Xi.at(0))/hi.at(0) - fXid1 );

        // Step 3
        for (unsigned int i = 1 ; i < nb_pt - 1 ; i++)
        {
          fAlpha.at(i) = 3.0f * ( ( f_Xi.at(i+1) - f_Xi.at(i) ) / hi.at(i) - ( f_Xi.at(i) - f_Xi.at(i-1) ) / hi.at(i-1) );
        }

        fAlpha.at(nb_pt-1) = 3.0f * ( fXidn - (f_Xi.at(nb_pt-1) - f_Xi.at(nb_pt-2))/hi.at(nb_pt-2) );

        std::vector<float> fli;
        std::vector<float> fmui;
        fli.resize(nb_pt);
        fmui.resize(nb_pt-1);

        fli.at(0)  = 2.0f*hi.at(0);
        fmui.at(0) = 0.5f;
        fZi.at(0) = fAlpha.at(0)/fli.at(0);

        // Step 5
        for (unsigned int i = 1 ; i < nb_pt - 1 ; i++) // attention : melange de matrix et vector, commence pas pareil (0 ou 1)
        {
          fli.at(i) = 2.0f * ( fTime.at(i+1) - fTime.at(i-1) ) - hi.at(i-1) * fmui.at(i-1);
          fmui.at(i) = hi.at(i)/fli.at(i);
          fZi.at(i) = ( fAlpha.at(i) - hi.at(i-1) * fZi.at(i-1) ) / fli.at(i);
        }

        // Step 6
        fli.at(nb_pt-1) = hi.at(nb_pt-2) * ( 2.0f - fmui.at(nb_pt-2) );   // matlab : li(n) = hi(n-1)*(2-mui(n-1));
        fZi.at(nb_pt-1) = (fAlpha.at(nb_pt-1) - hi.at(nb_pt-2)*fZi.at(nb_pt-2) ) / fli.at(nb_pt-1); // matlab : fZi{n} = (fAlpha{n}-hi(n-1)*fZi{n-1})/li(n);
        f_Ci.at(nb_pt-1) = fZi.at(nb_pt-1); // matlab : Ci{n} = fZi{n};

        // Step 7 (k = n-1:-1:1) -- Ci a 1 terme, Bi et Di sont vides.
        for (int i = nb_pt-2 ; i > - 1 ; i--) // attention : melange de matrix et vector, commence pas pareil (0 ou 1)
        {
          f_Ci.at(i) = fZi.at(i) - fmui.at(i)*f_Ci.at(i+1);
          f_Bi.at(i) = ( f_Xi.at(i+1) - f_Xi.at(i) ) / hi.at(i) - 0.33333333333333f*hi.at(i)*( f_Ci.at(i+1) + 2.0f*f_Ci.at(i) );
          f_Di.at(i) = 0.333333333333333f*(f_Ci.at(i+1) - f_Ci.at(i)) / hi.at(i);
        }
      } // END Init MAIN

      bool ALInterpolationCartesian::isFinished(const float& time)
      {
        // DG add test if fTime size is null no initialization so return true
        if (fTime.empty())
        {
          return true;
          //fIsFinished = true;
          //fNumTimesCalled = 0;
        }

        if (fIsFinished
          || (time >= (fTime.back()+epsilon))
          || (time <= (fTime.front()-epsilon)) )
        {
          fIsFinished = true;
          //fNumTimesCalled = 0;
          return true;
        }

        return false;
      }

      void ALInterpolationCartesian::getCurrentInterpolation(
        const float&                      pTimeCurrent,
        AL::Math::TransformAndVelocity6D& pOut)
      {
        fNumTimesCalled++;
        unsigned int j = 0;
        // - epsilon devient + epsilon
        while ( (j<fTime.size()) && (fTime.at(j) <= (pTimeCurrent+epsilon)) )
        {
          j ++;
        }

        if (j!=0)
        {
          j = j - 1;
        }

        if (j == (((unsigned int)fTime.size())-1))
        {
          j = j-1;
        }
        if (j < 0)
        {
          j = 0;
        }

        float timeDif = pTimeCurrent - fTime.at(j);
        float timeDif_x2 = powf(timeDif, 2);

        fX  = f_Xi.at(j) +
          f_Bi.at(j) * timeDif +
          f_Ci.at(j) * timeDif_x2 +
          f_Di.at(j) * (timeDif_x2*timeDif);

        fdX = f_Bi.at(j) +
          (2.0f * timeDif) * f_Ci.at(j)  +
          (3.0f * timeDif_x2) * f_Di.at(j);

        pOut.H = f_Hi.at(0)*VelocityExponential(fX);
        InvDiffLog(fX, fdX, fVTmp);

        // NEW -- Orient correctement la vitesse
        ChangeRepereVelocity6D(
          f_Hi.at(0), //const AL::Math::Transform& pH,
          fVTmp,      //const AL::Math::Velocity6D&  pVIn,
          pOut.V);    //AL::Math::Velocity6D&        pVOut

        // DAVID
        fOut = pOut; // make a copy of result
      } // end getCurrentInterpolation


      AL::Math::TransformAndVelocity6D ALInterpolationCartesian::getCurrentInterpolation(const float& time)
      {
        AL::Math::TransformAndVelocity6D  fSolution;
        getCurrentInterpolation(time, fSolution);
        return fSolution;
      }

      // give all Transform / Velocity6D for all interpolation
      std::vector<AL::Math::TransformAndVelocity6D> ALInterpolationCartesian::getAllInterpolation(const float& dt_step)
      {
        std::vector<AL::Math::TransformAndVelocity6D> solution;

        //Round to nearest integer
        fNbAllInterpolation = 0;

        float i = dt_step;
        while (!isFinished(i))
        {
          fNbAllInterpolation ++;
          i = i + dt_step;
        }

        AL::Math::TransformAndVelocity6D HVNull;
        solution.assign(fNbAllInterpolation, HVNull); // n cellule de 6D vector

        // boucle temporelle d'interpolation
        i = dt_step;
        unsigned int jj = 0;
        while (!isFinished(i))
        {
          ALInterpolationCartesian::getCurrentInterpolation(i,solution.at(jj));
          jj++;
          i = i + dt_step;
        }

        return solution;
      } // end getAllInterpolation

      void ALInterpolationCartesian::xSetFinalTransform(const AL::Math::Transform& pHFinal)
      {
        fHFinal = pHFinal;
      }

      void ALInterpolationCartesian::xSetInitTransform(const AL::Math::Transform& pHInit)
      {
        fHInit = pHInit;
      }

      ALInterpolationCartesian::~ALInterpolationCartesian()
      {
        // add destructor code here.
      }


    }
  }
}


