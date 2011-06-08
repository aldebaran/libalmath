/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Milos Zefran and Vizay Kumar
** Two Methods for Interpolating Rigid Body Motions
** IEEE International Conference on Robotics and Automation
** 1998 pp.2922-2927
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALINTERPOLATIONCARTESIAN_H_
#define _LIB_ALMATH_ALMATH_ALINTERPOLATIONCARTESIAN_H_

#include <almath/types/altransformandvelocity6d.h>
#include <almath/types/alposition6d.h>
#include <almath/types/alaxismask.h>

namespace AL
{
  namespace Math
  {
    namespace Interpolation
    {

      class ALInterpolationCartesian // : public ALInterpolator
      {
      public:

        /**
        * Default Constructor.
        */
        ALInterpolationCartesian (void);

        /**
        * Initialize SE(3) Interpolation between two transforms
        * @param pTime        : Duration in second of the different stay point of the Interpolation
        * @param pPosition6D  : Position6D of the different stay point of the Interpolation
        * @param pPeriod      : Integration step usefull to guaranty final time interpolation
        */
        void Init(
          const std::vector<float>&                pTime,
          const std::vector<AL::Math::Position6D>& pPosition6D,
          const float&                             pPeriod);


        /**
        * Initialize SE(3) Interpolation between two transforms
        * @param pTime        : Duration in second of the different stay point of the Interpolation
        * @param pPosition6D  : Position6D of the different stay point of the Interpolation
        * @param pVelocity6D  : Velocity in Velocity6D of the first and last point passage
        * @param pAxisMask    : Axis controlled during SE(3) interpolation by control law
        * @param pPeriod      : Integration step usefull to guaranty final time interpolation
        */
        void Init(
          const std::vector<float>&                pTime,
          const std::vector<AL::Math::Position6D>& pPosition6D,
          const std::vector<AL::Math::Velocity6D>& pVelocity,
          AL::Math::AXIS_MASK                      pAxisMask,
          const float&                             pPeriod);


        /**
        * Initialize SE(3) Interpolation between two transforms
        * @param pTime        : Duration in second of the different stay point of the Interpolation
        * @param pPosition6D  : Position6D of the different stay point of the Interpolation
        * @param pVelocity6D  : Velocity in Velocity6D of the first and last point passage
        * @param pPeriod      : Integration step usefull to guaranty final time interpolation
        */
        void Init(
          const std::vector<float>&                pTime,
          const std::vector<AL::Math::Position6D>& pPosition6D,
          const std::vector<AL::Math::Velocity6D>& pVelocity6D,
          const float&                             pPeriod);


        /**
        * Initialize SE(3) Interpolation between two transforms
        * @param pTime        : Duration in second of the different stay point of the Interpolation
        * @param pPosition6D  : Position6D of the different stay point of the Interpolation
        * @param pAxisMask    : Axis controlled during SE(3) interpolation by control law
        * @param pPeriod      : Integration step usefull to guaranty final time interpolation
        */
        void Init(
          const std::vector<float>&                pTime,
          const std::vector<AL::Math::Position6D>& pPosition6D,
          AL::Math::AXIS_MASK                      pAxisMask,
          const float&                             pPeriod);


        /**
        * Initialize SE(3) Interpolation between two transforms
        * @param pTime        : Duration in second of the different stay point of the Interpolation
        * @param pTransform : ALTranform of the different stay point of the Interpolation
        * @param pAxisMask    : Axis controlled during SE(3) interpolation by control law
        * @param pPeriod      : Integration step usefull to guaranty final time interpolation
        */
        void Init(
          const std::vector<float>&                 pTime,
          const std::vector<AL::Math::Transform>& pTransform,
          AL::Math::AXIS_MASK                       pAxisMask,
          const float&                              pPeriod);


        /**
        * Initialize SE(3) Interpolation between two transforms
        * @param pTime        : Duration in second of the different stay point of the Interpolation
        * @param pTransform : ALTranform of the different stay point of the Interpolation
        * @param pPeriod      : Integration step usefull to guaranty final time interpolation
        */
        void Init(
          const std::vector<float>&                 pTime,
          const std::vector<AL::Math::Transform>& pTransform,
          const float&                              pPeriod);


        /**
        * Initialize SE(3) Interpolation between two transforms
        * @param pTime        : Duration in second of the different stay point of the Interpolation
        * @param pTransform : ALTranform of the different stay point of the Interpolation
        * @param pVelocity6D  : Velocity in Velocity6D of the first and last point passage
        * @param pAxisMask    : Axis controlled during SE(3) interpolation by control law
        * @param pPeriod      : Integration step usefull to guaranty final time interpolation
        */
        void Init(
          const std::vector<float>&                 pTime,
          const std::vector<AL::Math::Transform>& pTransform,
          const std::vector<AL::Math::Velocity6D>&  pVelocity6D,
          AL::Math::AXIS_MASK                       pAxisMask,
          const float&                              pPeriod);


        /**
        * Initialize SE(3) Interpolation between two transforms
        * @param pTime        : Duration in second of the different stay point of the Interpolation
        * @param pTransform : ALTranform of the different stay point of the Interpolation
        * @param pVelocity6D  : Velocity in Velocity6D of the first and last point passage
        * @param pPeriod      : Integration step usefull to guaranty final time interpolation
        */
        void Init(
          const std::vector<float>&                 pTime,
          const std::vector<AL::Math::Transform>& pTransform,
          const std::vector<AL::Math::Velocity6D>&  pVelocity6D,
          const float&                              pPeriod);

        // give current Transform / Velocity6D at current time
        void getCurrentInterpolation(
          const float&                      time,
          AL::Math::TransformAndVelocity6D& pOut);

        AL::Math::TransformAndVelocity6D getCurrentInterpolation(const float& time);

        // give all Transform / Velocity6D for all interpolation
        std::vector<AL::Math::TransformAndVelocity6D> getAllInterpolation(const float& dt_step);

        bool isFinished(const float& time);

        inline const unsigned int getNumTimesCalled() const
        {
          return fNumTimesCalled;
        }

        inline AXIS_MASK getAxisMask() const
        {
          return fAxisMask;
        }

        inline bool getIsFinished(void) const
        {
          return fIsFinished;
        }

        // new
        void setIsFinished(bool tmpFinished)
        {
          fIsFinished = tmpFinished;
        }

        inline AL::Math::Transform getFinalTransform(void) const
        {
          return fHFinal;
        }

        inline AL::Math::Transform getInitTransform(void) const
        {
          return fHInit;
        }

        // Space
        inline void setSpace(unsigned int pSpace)
        {
          fSpace = pSpace;
        }

        inline unsigned int getSpace(void)
        {
          return fSpace;
        }

        // Rotation orientation
        inline void setFixRepereInWorld(AL::Math::Transform& pHFix)
        {
           fFixRepereInWorld = pHFix;
        }

        inline AL::Math::Transform getFixRepereInWorld(void)
        {
           return fFixRepereInWorld;
        }

        // result
        inline AL::Math::TransformAndVelocity6D getLastSolution(void)
        {
           return fOut;
        }

        /**
        * Destructor.
        */
        virtual ~ALInterpolationCartesian ();

      protected:
        unsigned int                        fSpace;
        unsigned int                        fNbAllInterpolation;
        unsigned int                        fNumTimesCalled;
        bool                                fIsFinished;
        AXIS_MASK                           fAxisMask;
        float                               fPeriod;

        std::vector<float>  fTime;

        AL::Math::Transform                 fFixRepereInWorld;

        std::vector<AL::Math::Transform>    f_Hi;
        std::vector<AL::Math::Velocity6D>   f_Xi;
        std::vector<AL::Math::Velocity6D>   f_Bi;
        std::vector<AL::Math::Velocity6D>   f_Ci;
        std::vector<AL::Math::Velocity6D>   f_Di;

        AL::Math::Velocity6D                fX;
        AL::Math::Velocity6D                fdX;
        AL::Math::Velocity6D                fVNull;
        std::vector<AL::Math::Transform>    fDHi;
        std::vector<AL::Math::Velocity6D>   fAlpha;
        std::vector<AL::Math::Velocity6D>   fZi;
        AL::Math::Velocity6D                fXid1;
        AL::Math::Velocity6D                fXidn;

        AL::Math::Transform                 fHFinal;
        AL::Math::Transform                 fHInit;

        AL::Math::Velocity6D                fVTmp;
        AL::Math::Velocity6D                fVInit;
        AL::Math::Velocity6D                fVEnd;

        AL::Math::TransformAndVelocity6D    fOut; // the last resolution


        //void xChangeFinalTime(float& pFinalTime);
        void xSetFinalTransform(const AL::Math::Transform& pHFinal);
        void xSetInitTransform(const AL::Math::Transform& pHInit);
      };

    }
  }
}

#endif  // _LIB_ALMATH_ALMATH_ALINTERPOLATIONCARTESIAN_H_

