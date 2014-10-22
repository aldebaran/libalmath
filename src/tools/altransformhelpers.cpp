/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <cmath>

#include <almath/tools/altransformhelpers.h>
#include <stdexcept>
#include <almath/tools/altrigonometry.h>
#include <almath/tools/almathio.h>
#include <assert.h>

namespace AL {
  namespace Math {

    Transform transformFromRotationPosition3D(
        const Rotation& pRot,
        const float&    pX,
        const float&    pY,
        const float&    pZ)
    {
      Transform T = Transform();

      T.r1_c4 = pX;
      T.r2_c4 = pY;
      T.r3_c4 = pZ;

      T.r1_c1 = pRot.r1_c1;
      T.r1_c2 = pRot.r1_c2;
      T.r1_c3 = pRot.r1_c3;

      T.r2_c1 = pRot.r2_c1;
      T.r2_c2 = pRot.r2_c2;
      T.r2_c3 = pRot.r2_c3;

      T.r3_c1 = pRot.r3_c1;
      T.r3_c2 = pRot.r3_c2;
      T.r3_c3 = pRot.r3_c3;

      return T;
    }

    Transform transformFromRotationPosition3D(
        const Rotation&   pRot,
        const Position3D& pPos)
    {
      Transform T = Transform();

      T.r1_c4 = pPos.x;
      T.r2_c4 = pPos.y;
      T.r3_c4 = pPos.z;

      T.r1_c1 = pRot.r1_c1;
      T.r1_c2 = pRot.r1_c2;
      T.r1_c3 = pRot.r1_c3;

      T.r2_c1 = pRot.r2_c1;
      T.r2_c2 = pRot.r2_c2;
      T.r2_c3 = pRot.r2_c3;

      T.r3_c1 = pRot.r3_c1;
      T.r3_c2 = pRot.r3_c2;
      T.r3_c3 = pRot.r3_c3;

      return T;
    }


    void transformLogarithmInPlace(
        const Transform& pH,
        Velocity6D&      pVOut)
    {
      const float epsilon = 0.001f; // new

      // square root of sum of squares of the elements
      const float si = 0.5f*std::sqrt((pH.r3_c2 - pH.r2_c3)*(pH.r3_c2 - pH.r2_c3) +
                             (pH.r1_c3 - pH.r3_c1)*(pH.r1_c3 - pH.r3_c1) +
                             (pH.r2_c1 - pH.r1_c2)*(pH.r2_c1 - pH.r1_c2) );
      const float co = 0.5f*(pH.r1_c1 + pH.r2_c2 + pH.r3_c3 - 1.0f);

      const float angle = std::atan2(si, co);// arctan(sin/cos)

      float coeff  = 0.0f;
      float lambda = 0.0f;
      if (si < epsilon)
      {
        if (co > 1.0f - epsilon)
        {
          coeff = angle/( 2.0f*si + epsilon );
          pVOut.wxd = coeff * (pH.r3_c2 - pH.r2_c3);
          pVOut.wyd = coeff * (pH.r1_c3 - pH.r3_c1);
          pVOut.wzd = coeff * (pH.r2_c1 - pH.r1_c2);
        }
        else if (co < -1.0f + epsilon)
        {
          if (pH.r1_c1 > 1.0f - epsilon)
          {
            lambda    = 1.0f/(PI*PI);
            pVOut.wxd = angle;
            pVOut.wyd = 0.0f;
            pVOut.wzd = 0.0f;
            pVOut.xd  = pH.r1_c4;
            pVOut.yd  = (1.0f-lambda*angle*angle)*pH.r2_c4 + 0.5f*angle*pH.r3_c4;
            pVOut.zd  = -0.5f*angle*pH.r2_c4 + (1.0f-lambda*angle*angle)*pH.r3_c4;
            return;
          }
          else if (pH.r2_c2 > 1.0f - epsilon)
          {
            lambda    = 1.0f/(PI*PI);
            pVOut.wxd = 0.0f;
            pVOut.wyd = angle;
            pVOut.wzd = 0.0f;
            pVOut.xd  = (1.0f-lambda*angle*angle)*pH.r1_c4 - 0.5f*angle*pH.r3_c4;
            pVOut.yd  = pH.r2_c4;
            pVOut.zd  = 0.5f*angle*pH.r1_c4 + (1.0f-lambda*angle*angle)*pH.r3_c4;
            return;
          }
          else if (pH.r3_c3 > 1.0f - epsilon)
          {
            lambda    = 1.0f/(PI*PI);
            pVOut.wxd = 0.0f;
            pVOut.wyd = 0.0f;
            pVOut.wzd = angle;
            pVOut.xd  = (1.0f-lambda*angle*angle)*pH.r1_c4 + 0.5f*angle*pH.r2_c4;
            pVOut.yd  = -0.5f*angle*pH.r1_c4 + (1.0f-lambda*angle*angle)*pH.r2_c4;
            pVOut.zd  = pH.r3_c4;
            return;
          }
          else
          {
            //std::cout << "transformLogarithmInPlace : co cas non traite" << std::endl;
          }
        }
        else
        {
          //std::cout << "transformLogarithmInPlace : si cas non traite" << std::endl;
        }
      }
      else
      {
        coeff = angle/(2.0f*si); // was pow(2.0f,-52)
        pVOut.wxd = coeff * (pH.r3_c2 - pH.r2_c3);
        pVOut.wyd = coeff * (pH.r1_c3 - pH.r3_c1);
        pVOut.wzd = coeff * (pH.r2_c1 - pH.r1_c2);
      }

      const float coeff_2 = std::pow(coeff, 2);

      if (angle < epsilon) // 0.001f epsilon
      {
        lambda = 1.0f/12.0f; // 0.08333333333333f
      }
      else if ((angle > PI - epsilon)||(angle < -PI + epsilon))
      {
        lambda = 0.101f;
      }
      else
      {
        lambda = 0.5f*(2.0f*si - angle*(1.0f + co)) / (angle * angle * si);
      }

      pVOut.xd = pH.r2_c4*(
          coeff_2*(  pH.r1_c3 - pH.r3_c1 )*( pH.r3_c2 - pH.r2_c3 )*lambda -
          0.5f*coeff*( pH.r1_c2 - pH.r2_c1 )) +
          pH.r3_c4*( coeff_2*(  pH.r1_c2 - pH.r2_c1 )*( pH.r2_c3 - pH.r3_c2 )*lambda -
                     0.5f*coeff*( pH.r1_c3 - pH.r3_c1 )) +
          pH.r1_c4*( coeff_2*(( pH.r1_c3 - pH.r3_c1 )*( pH.r3_c1 - pH.r1_c3) +
                              ( pH.r1_c2 - pH.r2_c1 )*( pH.r2_c1 - pH.r1_c2 ))*lambda + 1.0f );

      pVOut.yd = pH.r2_c4*(
          coeff_2*(( pH.r2_c3 - pH.r3_c2 )*( pH.r3_c2 - pH.r2_c3 ) +
                   ( pH.r1_c2 - pH.r2_c1 )*( pH.r2_c1 - pH.r1_c2 ))*lambda + 1.0f) +
          pH.r1_c4*( coeff_2*( pH.r3_c1 - pH.r1_c3 )*( pH.r2_c3 - pH.r3_c2 )*lambda -
                     0.5f*coeff*( pH.r2_c1 - pH.r1_c2 )) +
          pH.r3_c4*( coeff_2*( pH.r2_c1 - pH.r1_c2 )*( pH.r1_c3 - pH.r3_c1 )*lambda -
                     0.5f*coeff*( pH.r2_c3 - pH.r3_c2 ));

      pVOut.zd = pH.r3_c4*(
          coeff_2*(( pH.r2_c3 - pH.r3_c2 )*( pH.r3_c2 - pH.r2_c3 ) +
                   ( pH.r1_c3 - pH.r3_c1 )*( pH.r3_c1 - pH.r1_c3 ))*lambda + 1.0f ) +
          pH.r1_c4*( coeff_2*( pH.r2_c1 - pH.r1_c2 )*( pH.r3_c2 - pH.r2_c3 )*lambda -
                     0.5f*coeff*( pH.r3_c1 - pH.r1_c3 )) +
          pH.r2_c4*( coeff_2*( pH.r1_c2 - pH.r2_c1 )*( pH.r3_c1 - pH.r1_c3 )*lambda -
                     0.5f*coeff*( pH.r3_c2 - pH.r2_c3 ));

    } // end transformLogarithmInPlace


    Velocity6D transformLogarithm(const Transform& pH)
    {
      Velocity6D pV;
      transformLogarithmInPlace(pH, pV);
      return pV;
    }


    void velocityExponentialInPlace(
        const AL::Math::Velocity6D& pM,
        AL::Math::Transform&        tM)
    {
      float t;
      // square root of sum of squares of the elements (w.norm_Frobenius())
      t = std::sqrt(pM.wxd*pM.wxd + pM.wyd*pM.wyd + pM.wzd*pM.wzd);

      float CC = 0.0f, SC = 0.0f, dSC = 0.0f;

      if (t >= 0.001f) // seuil
      {
        CC  = (1-std::cos(t)) / (t*t); // motionCos cardinal cosinus
        SC  = std::sin(t) / t;       // motionSin cardinal sinus
        dSC = (t-std::sin(t)) / std::pow(t, 3); // motionSin cardinal sinus derivative std::pow
      }
      else
      {
        CC  = 0.5f;
        SC  = 1.0f - t*t / 6.0f;
        dSC = 0.166666667f;
      }

      // Maxima
      tM.r1_c1 = 1.0f - CC*(pM.wzd*pM.wzd + pM.wyd*pM.wyd);
      tM.r1_c2 =   - SC*pM.wzd  + CC*pM.wxd*pM.wyd;
      tM.r1_c3 =     SC*pM.wyd  + CC*pM.wxd*pM.wzd;
      tM.r2_c1 =     SC*pM.wzd  + CC*pM.wxd*pM.wyd;
      tM.r2_c2 = 1.0f - CC*(pM.wxd*pM.wxd + pM.wzd*pM.wzd);
      tM.r2_c3 =   - SC*pM.wxd  + CC*pM.wyd*pM.wzd;
      tM.r3_c1 =   - SC*pM.wyd  + CC*pM.wxd*pM.wzd;
      tM.r3_c2 =     SC*pM.wxd  + CC*pM.wyd*pM.wzd;
      tM.r3_c3 = 1.0f - CC*(pM.wxd*pM.wxd + pM.wyd*pM.wyd);

      tM.r1_c4 = (SC + dSC*pM.wxd*pM.wxd)*pM.xd +
                 (-CC*pM.wzd + dSC*pM.wxd*pM.wyd)*pM.yd +
                 (+CC*pM.wyd + dSC*pM.wxd*pM.wzd)*pM.zd;

      tM.r2_c4 = (CC*pM.wzd + dSC*pM.wyd*pM.wxd)*pM.xd +
                 (SC + dSC*pM.wyd*pM.wyd)*pM.yd +
                 (-CC*pM.wxd + dSC*pM.wyd*pM.wzd)*pM.zd;

      tM.r3_c4 = (-CC*pM.wyd + dSC*pM.wzd*pM.wxd)*pM.xd +
                 (CC*pM.wxd + dSC*pM.wzd*pM.wyd)*pM.yd +
                 (SC + dSC*pM.wzd*pM.wzd)*pM.zd;
    }


    Transform velocityExponential(const Velocity6D& pM)
    {
      Transform tM;
      velocityExponentialInPlace(pM, tM);
      return tM;
    }


    void changeReferenceVelocity6D(
        const Transform&  pH,
        const Velocity6D& pVIn,
        Velocity6D&       pVOut)
    {
      pVOut.xd  = pH.r1_c1 * pVIn.xd  + pH.r1_c2 * pVIn.yd  + pH.r1_c3 * pVIn.zd;
      pVOut.yd  = pH.r2_c1 * pVIn.xd  + pH.r2_c2 * pVIn.yd  + pH.r2_c3 * pVIn.zd;
      pVOut.zd  = pH.r3_c1 * pVIn.xd  + pH.r3_c2 * pVIn.yd  + pH.r3_c3 * pVIn.zd;
      pVOut.wxd = pH.r1_c1 * pVIn.wxd + pH.r1_c2 * pVIn.wyd + pH.r1_c3 * pVIn.wzd;
      pVOut.wyd = pH.r2_c1 * pVIn.wxd + pH.r2_c2 * pVIn.wyd + pH.r2_c3 * pVIn.wzd;
      pVOut.wzd = pH.r3_c1 * pVIn.wxd + pH.r3_c2 * pVIn.wyd + pH.r3_c3 * pVIn.wzd;
    }


    void changeReferencePosition6D(
        const Transform&  pH,
        const Position6D& pPIn,
        Position6D&       pPOut)
    {
      pPOut.x  = pH.r1_c1 * pPIn.x  + pH.r1_c2 * pPIn.y  + pH.r1_c3 * pPIn.z;
      pPOut.y  = pH.r2_c1 * pPIn.x  + pH.r2_c2 * pPIn.y  + pH.r2_c3 * pPIn.z;
      pPOut.z  = pH.r3_c1 * pPIn.x  + pH.r3_c2 * pPIn.y  + pH.r3_c3 * pPIn.z;
      pPOut.wx = pH.r1_c1 * pPIn.wx + pH.r1_c2 * pPIn.wy + pH.r1_c3 * pPIn.wz;
      pPOut.wy = pH.r2_c1 * pPIn.wx + pH.r2_c2 * pPIn.wy + pH.r2_c3 * pPIn.wz;
      pPOut.wz = pH.r3_c1 * pPIn.wx + pH.r3_c2 * pPIn.wy + pH.r3_c3 * pPIn.wz;
    }


    void changeReferencePosition3DInPlace(
        const Transform& pH,
        Position3D&      pPosOut)
    {
      float x, y, z;
      x = pPosOut.x;
      y = pPosOut.y;
      z = pPosOut.z;

      pPosOut.x = pH.r1_c1 * x + pH.r1_c2 * y + pH.r1_c3 * z;
      pPosOut.y = pH.r2_c1 * x + pH.r2_c2 * y + pH.r2_c3 * z;
      pPosOut.z = pH.r3_c1 * x + pH.r3_c2 * y + pH.r3_c3 * z;
    }


    void changeReferenceTransposePosition3DInPlace(
        const Transform&  pH,
        Position3D&       pPosOut)
    {
      float x, y, z;
      x = pPosOut.x;
      y = pPosOut.y;
      z = pPosOut.z;

      pPosOut.x = pH.r1_c1 * x + pH.r2_c1 * y + pH.r3_c1 * z;
      pPosOut.y = pH.r1_c2 * x + pH.r2_c2 * y + pH.r3_c2 * z;
      pPosOut.z = pH.r1_c3 * x + pH.r2_c3 * y + pH.r3_c3 * z;
    }


    void changeReferencePosition3D(
        const Transform&  pH,
        const Position3D& pPosIn,
        Position3D&       pPosOut)
    {
      pPosOut.x = pH.r1_c1 * pPosIn.x + pH.r1_c2 * pPosIn.y + pH.r1_c3 * pPosIn.z;
      pPosOut.y = pH.r2_c1 * pPosIn.x + pH.r2_c2 * pPosIn.y + pH.r2_c3 * pPosIn.z;
      pPosOut.z = pH.r3_c1 * pPosIn.x + pH.r3_c2 * pPosIn.y + pH.r3_c3 * pPosIn.z;
    }


    void changeReferenceTransposePosition3D(
        const Transform&  pH,
        const Position3D& pPosIn,
        Position3D&       pPosOut)
    {
      pPosOut.x = pH.r1_c1 * pPosIn.x + pH.r2_c1 * pPosIn.y + pH.r3_c1 * pPosIn.z;
      pPosOut.y = pH.r1_c2 * pPosIn.x + pH.r2_c2 * pPosIn.y + pH.r3_c2 * pPosIn.z;
      pPosOut.z = pH.r1_c3 * pPosIn.x + pH.r2_c3 * pPosIn.y + pH.r3_c3 * pPosIn.z;
    }


    void changeReferenceTransform(
        const AL::Math::Transform&  pH,
        const AL::Math::Transform&  pHIn,
        AL::Math::Transform&        pHOut)
    {
      pHOut.r1_c1 = (pH.r1_c1 * pHIn.r1_c1) + (pH.r1_c2 * pHIn.r2_c1) + (pH.r1_c3 * pHIn.r3_c1);
      pHOut.r1_c2 = (pH.r1_c1 * pHIn.r1_c2) + (pH.r1_c2 * pHIn.r2_c2) + (pH.r1_c3 * pHIn.r3_c2);
      pHOut.r1_c3 = (pH.r1_c1 * pHIn.r1_c3) + (pH.r1_c2 * pHIn.r2_c3) + (pH.r1_c3 * pHIn.r3_c3);
      pHOut.r1_c4 = (pH.r1_c1 * pHIn.r1_c4) + (pH.r1_c2 * pHIn.r2_c4) + (pH.r1_c3 * pHIn.r3_c4);

      pHOut.r2_c1 = (pH.r2_c1 * pHIn.r1_c1) + (pH.r2_c2 * pHIn.r2_c1) + (pH.r2_c3 * pHIn.r3_c1);
      pHOut.r2_c2 = (pH.r2_c1 * pHIn.r1_c2) + (pH.r2_c2 * pHIn.r2_c2) + (pH.r2_c3 * pHIn.r3_c2);
      pHOut.r2_c3 = (pH.r2_c1 * pHIn.r1_c3) + (pH.r2_c2 * pHIn.r2_c3) + (pH.r2_c3 * pHIn.r3_c3);
      pHOut.r2_c4 = (pH.r2_c1 * pHIn.r1_c4) + (pH.r2_c2 * pHIn.r2_c4) + (pH.r2_c3 * pHIn.r3_c4);

      pHOut.r3_c1 = (pH.r3_c1 * pHIn.r1_c1) + (pH.r3_c2 * pHIn.r2_c1) + (pH.r3_c3 * pHIn.r3_c1);
      pHOut.r3_c2 = (pH.r3_c1 * pHIn.r1_c2) + (pH.r3_c2 * pHIn.r2_c2) + (pH.r3_c3 * pHIn.r3_c2);
      pHOut.r3_c3 = (pH.r3_c1 * pHIn.r1_c3) + (pH.r3_c2 * pHIn.r2_c3) + (pH.r3_c3 * pHIn.r3_c3);
      pHOut.r3_c4 = (pH.r3_c1 * pHIn.r1_c4) + (pH.r3_c2 * pHIn.r2_c4) + (pH.r3_c3 * pHIn.r3_c4);
    }


    void changeReferenceTransposeTransform(
        const AL::Math::Transform&  pH,
        const AL::Math::Transform&  pHIn,
        AL::Math::Transform&        pHOut)
    {
      pHOut.r1_c1 = (pH.r1_c1 * pHIn.r1_c1) + (pH.r2_c1 * pHIn.r2_c1) + (pH.r3_c1 * pHIn.r3_c1);
      pHOut.r1_c2 = (pH.r1_c1 * pHIn.r1_c2) + (pH.r2_c1 * pHIn.r2_c2) + (pH.r3_c1 * pHIn.r3_c2);
      pHOut.r1_c3 = (pH.r1_c1 * pHIn.r1_c3) + (pH.r2_c1 * pHIn.r2_c3) + (pH.r3_c1 * pHIn.r3_c3);
      pHOut.r1_c4 = (pH.r1_c1 * pHIn.r1_c4) + (pH.r2_c1 * pHIn.r2_c4) + (pH.r3_c1 * pHIn.r3_c4);

      pHOut.r2_c1 = (pH.r1_c2 * pHIn.r1_c1) + (pH.r2_c2 * pHIn.r2_c1) + (pH.r3_c2 * pHIn.r3_c1);
      pHOut.r2_c2 = (pH.r1_c2 * pHIn.r1_c2) + (pH.r2_c2 * pHIn.r2_c2) + (pH.r3_c2 * pHIn.r3_c2);
      pHOut.r2_c3 = (pH.r1_c2 * pHIn.r1_c3) + (pH.r2_c2 * pHIn.r2_c3) + (pH.r3_c2 * pHIn.r3_c3);
      pHOut.r2_c4 = (pH.r1_c2 * pHIn.r1_c4) + (pH.r2_c2 * pHIn.r2_c4) + (pH.r3_c2 * pHIn.r3_c4);

      pHOut.r3_c1 = (pH.r1_c3 * pHIn.r1_c1) + (pH.r2_c3 * pHIn.r2_c1) + (pH.r3_c3 * pHIn.r3_c1);
      pHOut.r3_c2 = (pH.r1_c3 * pHIn.r1_c2) + (pH.r2_c3 * pHIn.r2_c2) + (pH.r3_c3 * pHIn.r3_c2);
      pHOut.r3_c3 = (pH.r1_c3 * pHIn.r1_c3) + (pH.r2_c3 * pHIn.r2_c3) + (pH.r3_c3 * pHIn.r3_c3);
      pHOut.r3_c4 = (pH.r1_c3 * pHIn.r1_c4) + (pH.r2_c3 * pHIn.r2_c4) + (pH.r3_c3 * pHIn.r3_c4);
    }


    void changeReferenceTransposeVelocity6D(
        const AL::Math::Transform&  pH,
        const AL::Math::Velocity6D& pVIn,
        AL::Math::Velocity6D&       pVOut)
    {
      pVOut.xd  = pH.r1_c1 * pVIn.xd  + pH.r2_c1 * pVIn.yd  + pH.r3_c1 * pVIn.zd  ;
      pVOut.yd  = pH.r1_c2 * pVIn.xd  + pH.r2_c2 * pVIn.yd  + pH.r3_c2 * pVIn.zd  ;
      pVOut.zd  = pH.r1_c3 * pVIn.xd  + pH.r2_c3 * pVIn.yd  + pH.r3_c3 * pVIn.zd  ;
      pVOut.wxd = pH.r1_c1 * pVIn.wxd + pH.r2_c1 * pVIn.wyd + pH.r3_c1 * pVIn.wzd;
      pVOut.wyd = pH.r1_c2 * pVIn.wxd + pH.r2_c2 * pVIn.wyd + pH.r3_c2 * pVIn.wzd;
      pVOut.wzd = pH.r1_c3 * pVIn.wxd + pH.r2_c3 * pVIn.wyd + pH.r3_c3 * pVIn.wzd;
    }


    void changeReferenceTransposePosition6D(
        const AL::Math::Transform&  pH,
        const AL::Math::Position6D& pPIn,
        AL::Math::Position6D&       pPOut)
    {
      pPOut.x  = pH.r1_c1 * pPIn.x  + pH.r2_c1 * pPIn.y  + pH.r3_c1 * pPIn.z  ;
      pPOut.y  = pH.r1_c2 * pPIn.x  + pH.r2_c2 * pPIn.y  + pH.r3_c2 * pPIn.z  ;
      pPOut.z  = pH.r1_c3 * pPIn.x  + pH.r2_c3 * pPIn.y  + pH.r3_c3 * pPIn.z  ;
      pPOut.wx = pH.r1_c1 * pPIn.wx + pH.r2_c1 * pPIn.wy + pH.r3_c1 * pPIn.wz;
      pPOut.wy = pH.r1_c2 * pPIn.wx + pH.r2_c2 * pPIn.wy + pH.r3_c2 * pPIn.wz;
      pPOut.wz = pH.r1_c3 * pPIn.wx + pH.r2_c3 * pPIn.wy + pH.r3_c3 * pPIn.wz;
    }


    void transformMeanInPlace(
        const AL::Math::Transform&  pHIn1,
        const AL::Math::Transform&  pHIn2,
        const float&                pDist,
        AL::Math::Transform&        pHOut)
    {
      if ((pDist>1.0f) || (pDist<0.0f))
      {
        throw std::runtime_error(
            "ALMath: transformMeanInPlace Distance must be between 0 and 1.");
      }

      Velocity6D pV;
      Transform pHIn1i;

      transformInverse(pHIn1, pHIn1i);
      transformLogarithmInPlace(pHIn1i*pHIn2, pV);
      velocityExponentialInPlace(pDist*pV, pHOut);
      pHOut = pHIn1*pHOut;
    }


    AL::Math::Transform transformMean(
        const AL::Math::Transform&  pHIn1,
        const AL::Math::Transform&  pHIn2,
        const float&                pDist)
    {
      AL::Math::Transform pHOut;
      transformMeanInPlace(pHIn1, pHIn2, pDist, pHOut);
      return pHOut;
    }


    void transformFromPosition3DInPlace(
        const Position3D& pPosition,
        Transform&        pTransform)
    {
      pTransform.r1_c4 = pPosition.x;
      pTransform.r2_c4 = pPosition.y;
      pTransform.r3_c4 = pPosition.z;
    }


    Transform transformFromPosition3D(const Position3D& pPosition)
    {
      Transform transform = Transform();
      transformFromPosition3DInPlace(pPosition, transform);
      return transform;
    }


    void transformFromRotationInPlace(
        const Rotation& pRot,
        Transform&      pT)
    {
      pT.r1_c1 = pRot.r1_c1;
      pT.r1_c2 = pRot.r1_c2;
      pT.r1_c3 = pRot.r1_c3;

      pT.r2_c1 = pRot.r2_c1;
      pT.r2_c2 = pRot.r2_c2;
      pT.r2_c3 = pRot.r2_c3;

      pT.r3_c1 = pRot.r3_c1;
      pT.r3_c2 = pRot.r3_c2;
      pT.r3_c3 = pRot.r3_c3;
    }


    Transform transformFromRotation(const Rotation& pIn)
    {
      Transform pOut = Transform();
      transformFromRotationInPlace(pIn, pOut);
      return pOut;
    }


    void rotationFromTransformInPlace(
        const Transform& pIn,
        Rotation&        pOut)
    {
      pOut.r1_c1 = pIn.r1_c1;
      pOut.r1_c2 = pIn.r1_c2;
      pOut.r1_c3 = pIn.r1_c3;

      pOut.r2_c1 = pIn.r2_c1;
      pOut.r2_c2 = pIn.r2_c2;
      pOut.r2_c3 = pIn.r2_c3;

      pOut.r3_c1 = pIn.r3_c1;
      pOut.r3_c2 = pIn.r3_c2;
      pOut.r3_c3 = pIn.r3_c3;
    }


    Rotation rotationFromTransform(const Transform& pIn)
    {
      Rotation pOut = Rotation();
      rotationFromTransformInPlace(pIn, pOut);
      return pOut;
    }


    void position6DFromTransformInPlace(
        const Transform& pT,
        Position6D&      pPos)
    {
      pPos.x = pT.r1_c4;
      pPos.y = pT.r2_c4;
      pPos.z = pT.r3_c4;
      pPos.wz = std::atan2(pT.r2_c1, pT.r1_c1);
      const float sy = std::sin(pPos.wz);
      const float cy = std::cos(pPos.wz);
      pPos.wy = std::atan2(-pT.r3_c1, cy*pT.r1_c1+sy*pT.r2_c1);
      pPos.wx = std::atan2(sy*pT.r1_c3-cy*pT.r2_c3, cy*pT.r2_c2-sy*pT.r1_c2);
    }


    Position6D position6DFromTransform(const Transform& pT)
    {
      Position6D pPos;
      position6DFromTransformInPlace(pT, pPos);
      return pPos;
    }


    void transformFromPose2DInPlace(
        const Pose2D& pPose,
        Transform&    pT)
    {
      pT = Transform(pPose.x, pPose.y, 0.0f);
      pT *= transformFromRotZ(pPose.theta);
    }


    Transform transformFromPose2D(const Pose2D& pPos)
    {
      Transform T = Transform();
      transformFromPose2DInPlace(pPos, T);
      return T;
    }


    void position2DFromTransformInPlace(
        const Transform& pT,
        Position2D&      pPos)
    {
      pPos.x = pT.r1_c4;
      pPos.y = pT.r2_c4;
    }

    Position2D position2DFromTransform(const Transform& pT)
    {
      Position2D pPos;
      AL::Math::position2DFromTransformInPlace(pT, pPos);
      return pPos;
    }

    void pose2DFromTransformInPlace(
        const Transform& pT,
        Pose2D&          pPos)
    {
      pPos.x = pT.r1_c4;
      pPos.y = pT.r2_c4;
      pPos.theta = std::atan2(pT.r2_c1, pT.r1_c1);
    }


    Pose2D pose2DFromTransform(const Transform& pT)
    {
      Pose2D pPos;
      AL::Math::pose2DFromTransformInPlace(pT, pPos);
      return pPos;
    }


    Transform transformFromRotation3D(const Rotation3D& pRotation)
    {
      return transformFrom3DRotation(
          pRotation.wx,
          pRotation.wy,
          pRotation.wz);
    }


    Transform transformFromPosition6D(const Position6D& pPosition6D)
    {
      return transformFromPosition(
          pPosition6D.x,
          pPosition6D.y,
          pPosition6D.z,
          pPosition6D.wx,
          pPosition6D.wy,
          pPosition6D.wz);
    }


    void position6DFromTransformDiffInPlace(
        const Transform& pCurrent,
        const Transform& pTarget,
        Position6D&      result)
    {
      result.x = pTarget.r1_c4 - pCurrent.r1_c4;
      result.y = pTarget.r2_c4 - pCurrent.r2_c4;
      result.z = pTarget.r3_c4 - pCurrent.r3_c4;
      result.wx = 0.5f * ( ((pCurrent.r2_c1 * pTarget.r3_c1) - (pCurrent.r3_c1 * pTarget.r2_c1)) +
                           ((pCurrent.r2_c2 * pTarget.r3_c2) - (pCurrent.r3_c2 * pTarget.r2_c2)) +
                           ((pCurrent.r2_c3 * pTarget.r3_c3) - (pCurrent.r3_c3 * pTarget.r2_c3)) );

      result.wy = 0.5f * ( ((pCurrent.r3_c1 * pTarget.r1_c1) - (pCurrent.r1_c1 * pTarget.r3_c1)) +
                           ((pCurrent.r3_c2 * pTarget.r1_c2) - (pCurrent.r1_c2 * pTarget.r3_c2)) +
                           ((pCurrent.r3_c3 * pTarget.r1_c3) - (pCurrent.r1_c3 * pTarget.r3_c3)) );

      result.wz = 0.5f * ( ((pCurrent.r1_c1 * pTarget.r2_c1) - (pCurrent.r2_c1 * pTarget.r1_c1)) +
                           ((pCurrent.r1_c2 * pTarget.r2_c2) - (pCurrent.r2_c2 * pTarget.r1_c2)) +
                           ((pCurrent.r1_c3 * pTarget.r2_c3) - (pCurrent.r2_c3 * pTarget.r1_c3)) );
    }


    Position6D position6DFromTransformDiff(
        const Transform& pCurrent,
        const Transform& pTarget)
    {
      Position6D result;
      position6DFromTransformDiffInPlace(pCurrent, pTarget, result);
      return result;
    }


    void position3DFromTransformInPlace(
        const Transform& pT,
        Position3D&      pPos)
    {
      pPos.x = pT.r1_c4;
      pPos.y = pT.r2_c4;
      pPos.z = pT.r3_c4;
    }


    Position3D position3DFromTransform(const Transform& pT)
    {
      Position3D pOut;
      position3DFromTransformInPlace(pT, pOut);
      return pOut;
    }


    Rotation3D rotation3DFromTransform(const Transform& pT)
    {
      Rotation3D R;
      R.wz = std::atan2(pT.r2_c1,pT.r1_c1);
      const float sy = std::sin(R.wz);
      const float cy = std::cos(R.wz);
      R.wy = std::atan2(-pT.r3_c1, cy*pT.r1_c1+sy*pT.r2_c1);
      R.wx = std::atan2(sy*pT.r1_c3-cy*pT.r2_c3, cy*pT.r2_c2-sy*pT.r1_c2);
      return R;
    }


    Rotation3D rotation3DFromRotation(const Rotation& pR)
    {
      Rotation3D R;
      R.wz = std::atan2(pR.r2_c1,pR.r1_c1);
      const float sy = std::sin(R.wz);
      const float cy = std::cos(R.wz);
      R.wy = std::atan2(-pR.r3_c1, cy*pR.r1_c1+sy*pR.r2_c1);
      R.wx = std::atan2(sy*pR.r1_c3-cy*pR.r2_c3, cy*pR.r2_c2-sy*pR.r1_c2);
      return R;
    }

    Rotation rotationFromAxesXY(
        const Position3D& pX,
        const Position3D& pY)
    {
      assert(pX.isUnitVector());
      assert(pY.isUnitVector());
      assert(pX.isOrthogonal(pY, 1e-3f));
      Position3D z = pX.crossProduct(pY);
      return rotationFromAxesXYZ(pX, pY, z);
    }

    Rotation rotationFromAxesXZ(
        const Position3D& pX,
        const Position3D& pZ)
    {
      assert(pX.isUnitVector());
      assert(pZ.isUnitVector());
      assert(pX.isOrthogonal(pZ, 1e-3f));
      Position3D y = pZ.crossProduct(pX);
      return rotationFromAxesXYZ(pX, y, pZ);
    }

    Rotation rotationFromAxesYZ(
        const Position3D& pY,
        const Position3D& pZ)
    {
      assert(pY.isUnitVector());
      assert(pZ.isUnitVector());
      assert(pY.isOrthogonal(pZ, 1e-3f));
      Position3D x = pY.crossProduct(pZ);
      return rotationFromAxesXYZ(x, pY, pZ);
    }

    Rotation rotationFromAxesXYZ(
        const Position3D& pX,
        const Position3D& pY,
        const Position3D& pZ)
    {
      assert(pX.isUnitVector());
      assert(pY.isUnitVector());
      assert(pZ.isUnitVector());
      assert(pX.isOrthogonal(pY, 1e-3f));
      assert(pX.isOrthogonal(pZ, 1e-3f));
      assert(pY.isOrthogonal(pZ, 1e-3f));
      Rotation pOut = Rotation();
      pOut.r1_c1 = pX.x;
      pOut.r2_c1 = pX.y;
      pOut.r3_c1 = pX.z;
      pOut.r1_c2 = pY.x;
      pOut.r2_c2 = pY.y;
      pOut.r3_c2 = pY.z;
      pOut.r1_c3 = pZ.x;
      pOut.r2_c3 = pZ.y;
      pOut.r3_c3 = pZ.z;
      return pOut;
    }
    Position3D operator*(
        const Transform& pT,
        const Position2D&  pPos)
    {
      Position3D result;
      result.x = (pT.r1_c1 * pPos.x) + (pT.r1_c2 * pPos.y) + pT.r1_c4;
      result.y = (pT.r2_c1 * pPos.x) + (pT.r2_c2 * pPos.y) + pT.r2_c4;
      result.z = (pT.r3_c1 * pPos.x) + (pT.r3_c2 * pPos.y) + pT.r3_c4;
      return result;
    }


    Position3D operator*(
        const Transform& pT,
        const Position3D&  pPos)
    {
      Position3D result;
      result.x = (pT.r1_c1 * pPos.x) + (pT.r1_c2 * pPos.y) + (pT.r1_c3 * pPos.z) + pT.r1_c4;
      result.y = (pT.r2_c1 * pPos.x) + (pT.r2_c2 * pPos.y) + (pT.r2_c3 * pPos.z) + pT.r2_c4;
      result.z = (pT.r3_c1 * pPos.x) + (pT.r3_c2 * pPos.y) + (pT.r3_c3 * pPos.z) + pT.r3_c4;
      return result;
    }


    Transform axisRotationProjection(
        const Position3D& pAxis,
        const Transform&  pT)
    {
      AL::Math::Transform pTSol;

      // Save position part of Transform in the solution
      AL::Math::Rotation pRotationSolution;
      rotationFromTransformInPlace(pT, pRotationSolution);
      axisRotationProjectionInPlace(pAxis, pRotationSolution);

      transformFromRotationInPlace(pRotationSolution, pTSol);
      pTSol.r1_c4 = pT.r1_c4;
      pTSol.r2_c4 = pT.r2_c4;
      pTSol.r3_c4 = pT.r3_c4;

      return pTSol;
    }


    Rotation axisRotationProjection(
        const Position3D& pPos,
        const Rotation&   pRot)
    {
      Rotation pOut = pRot;
      axisRotationProjectionInPlace(pPos, pOut);
      return pOut;
    }


    void axisRotationProjectionInPlace(
        const Position3D& pAxis,
        Rotation&         pRot)
    {
      float inw = norm(pAxis);
      if (inw == 0.0f)
      {
        throw std::runtime_error(
            "ALMath: axisRotationProjectionInPlace Division by zero.");
      }

      inw = 1.0f/inw;

      float x = inw * pAxis.x;
      float y = inw * pAxis.y;
      float z = inw * pAxis.z;

      const float x_2 = std::pow(x, 2);
      const float y_2 = std::pow(y, 2);
      const float z_2 = std::pow(z, 2);

      const float a =
          x*(pRot.r2_c3 - z*(x*pRot.r2_c1 + y*pRot.r2_c2 + z*pRot.r2_c3)) +
          y*(pRot.r3_c1 - x*(x*pRot.r3_c1 + y*pRot.r3_c2 + z*pRot.r3_c2)) +
          z*(pRot.r1_c2 - y*(x*pRot.r1_c1 + y*pRot.r1_c2 + z*pRot.r1_c3));

      const float b =
          x*(-pRot.r3_c1*z + pRot.r3_c3*x) +
          y*( pRot.r1_c1*y - pRot.r1_c2*x) +
          z*( pRot.r2_c2*z - pRot.r2_c3*y);

      const float c =
          x*(z*x*pRot.r2_c1 + z*y*pRot.r2_c2 + z*z*pRot.r2_c3) +
          y*(x*x*pRot.r3_c1 + x*y*pRot.r3_c2 + x*z*pRot.r3_c3) +
          z*(y*x*pRot.r1_c1 + y*y*pRot.r1_c2 + y*z*pRot.r1_c3);

      const float d2 = a*a + b*b - c*c;

      if (d2<0)
      {
        throw std::runtime_error(
            "ALMath: axisRotationProjectionInPlace d2 < 0");
        return;
      }

      const float alpha  = std::atan2(b, a);
      const float beta   = std::acos( c / std::sqrt( a*a + b*b ) );

      const float cos_1 = 1.0f - std::cos( alpha + beta );
      const float cos_2 = 1.0f - std::cos( alpha - beta );

      const float sin_1 = std::sin( alpha + beta );
      const float sin_2 = std::sin( alpha - beta );

      const float trace_1 = pRot.r1_c1*( cos_1*( - z_2 - y_2 ) + 1.0f ) +
                      pRot.r2_c2*( cos_1*( - z_2 - x_2 ) + 1.0f ) +
                      pRot.r3_c2*( cos_1*y*z + sin_1*x ) +
                      pRot.r2_c3*( cos_1*y*z - sin_1*x ) +
                      pRot.r1_c3*( cos_1*x*z + sin_1*y ) +
                      pRot.r3_c1*( cos_1*x*z - sin_1*y ) +
                      pRot.r2_c1*( sin_1*z + cos_1*x*y ) +
                      pRot.r1_c2*( cos_1*x*y - sin_1*z ) +
                      pRot.r3_c3*( cos_1*( - y_2 - x_2 ) + 1.0f );

      const float trace_2 = pRot.r1_c1*( cos_2*( - z_2 - y_2 ) + 1.0f ) +
                      pRot.r2_c2*( cos_2*( - z_2 - x_2 ) + 1.0f ) +
                      pRot.r3_c2*( cos_2*y*z + sin_2*x ) +
                      pRot.r2_c3*( cos_2*y*z - sin_2*x ) +
                      pRot.r1_c3*( cos_2*x*z + sin_2*y ) +
                      pRot.r3_c1*( cos_2*x*z - sin_2*y ) +
                      pRot.r2_c1*( sin_2*z + cos_2*x*y ) +
                      pRot.r1_c2*( cos_2*x*y - sin_2*z ) +
                      pRot.r3_c3*( cos_2*( - y_2 - x_2 ) + 1.0f );

      if (trace_1 < trace_2)
      {
        pRot.r1_c1 = cos_2*( - z_2 - y_2 ) + 1.0f;
        pRot.r1_c2 = cos_2*x*y - sin_2*z;
        pRot.r1_c3 = cos_2*x*z + sin_2*y;
        pRot.r2_c1 = sin_2*z + cos_2*x*y;
        pRot.r2_c2 = cos_2*( - z_2 - x_2 ) + 1.0f;
        pRot.r2_c3 = cos_2*y*z - sin_2*x;
        pRot.r3_c1 = cos_2*x*z - sin_2*y;
        pRot.r3_c2 = cos_2*y*z + sin_2*x;
        pRot.r3_c3 = cos_2*( - y_2 - x_2 ) + 1.0f;

      }
      else
      {
        pRot.r1_c1 = cos_1*( - z_2 - y_2 ) + 1.0f;
        pRot.r1_c2 = cos_1*x*y - sin_1*z;
        pRot.r1_c3 = cos_1*x*z + sin_1*y;
        pRot.r2_c1 = sin_1*z + cos_1*x*y;
        pRot.r2_c2 = cos_1*( - z_2 - x_2 ) + 1.0f;
        pRot.r2_c3 = cos_1*y*z - sin_1*x;
        pRot.r3_c1 = cos_1*x*z - sin_1*y;
        pRot.r3_c2 = cos_1*y*z + sin_1*x;
        pRot.r3_c3 = cos_1*( - y_2 - x_2 ) + 1.0f;

      }
    } // end axisRotationProjectionInPlace


    void axisRotationProjectionInPlace(
        const Position3D& pAxis,
        Transform&        pH)
    {
      float inw = norm(pAxis);
      if (inw == 0.0f)
      {
        throw std::runtime_error(
            "ALMath: axisRotationProjectionInPlace Division by zero.");
      }

      inw = 1.0f/inw;

      const float x = inw * pAxis.x;
      const float y = inw * pAxis.y;
      const float z = inw * pAxis.z;

      const float x_2 = std::pow(x, 2);
      const float y_2 = std::pow(y, 2);
      const float z_2 = std::pow(z, 2);

      const float a =
          x*(pH.r2_c3 - z*(x*pH.r2_c1 + y*pH.r2_c2 + z*pH.r2_c3)) +
          y*(pH.r3_c1 - x*(x*pH.r3_c1 + y*pH.r3_c2 + z*pH.r3_c2)) +
          z*(pH.r1_c2 - y*(x*pH.r1_c1 + y*pH.r1_c2 + z*pH.r1_c3));

      const float b =
          x*(-pH.r3_c1*z + pH.r3_c3*x) +
          y*( pH.r1_c1*y - pH.r1_c2*x) +
          z*( pH.r2_c2*z - pH.r2_c3*y);

      const float c =
          x*(z*x*pH.r2_c1 + z*y*pH.r2_c2 + z*z*pH.r2_c3) +
          y*(x*x*pH.r3_c1 + x*y*pH.r3_c2 + x*z*pH.r3_c3) +
          z*(y*x*pH.r1_c1 + y*y*pH.r1_c2 + y*z*pH.r1_c3);

      const float d2 = a*a + b*b - c*c;

      if (d2<0.0f)
      {
        throw std::runtime_error(
            "ALMath: axisRotationProjectionInPlace d2 < 0");
        return;
      }

      const float alpha  = std::atan2(b, a);
      const float beta   = std::acos(c / std::sqrt(a*a + b*b));

      const float cos_1 = 1.0f - std::cos(alpha + beta);
      const float cos_2 = 1.0f - std::cos(alpha - beta);

      const float sin_1 = std::sin(alpha + beta);
      const float sin_2 = std::sin(alpha - beta);

      const float trace_1 =
          pH.r1_c1*(cos_1*( - z_2 - y_2) + 1.0f) +
          pH.r2_c2*(cos_1*( - z_2 - x_2) + 1.0f) +
          pH.r3_c2*(cos_1*y*z + sin_1*x) +
          pH.r2_c3*(cos_1*y*z - sin_1*x) +
          pH.r1_c3*(cos_1*x*z + sin_1*y) +
          pH.r3_c1*(cos_1*x*z - sin_1*y) +
          pH.r2_c1*(sin_1*z + cos_1*x*y) +
          pH.r1_c2*(cos_1*x*y - sin_1*z) +
          pH.r3_c3*(cos_1*( - y_2 - x_2) + 1.0f);

      const float trace_2 =
          pH.r1_c1*(cos_2*( - z_2 - y_2) + 1.0f) +
          pH.r2_c2*(cos_2*( - z_2 - x_2) + 1.0f) +
          pH.r3_c2*(cos_2*y*z + sin_2*x) +
          pH.r2_c3*(cos_2*y*z - sin_2*x) +
          pH.r1_c3*(cos_2*x*z + sin_2*y) +
          pH.r3_c1*(cos_2*x*z - sin_2*y) +
          pH.r2_c1*(sin_2*z + cos_2*x*y) +
          pH.r1_c2*(cos_2*x*y - sin_2*z) +
          pH.r3_c3*(cos_2*( - y_2 - x_2) + 1.0f);

      if (trace_1 < trace_2)
      {
        pH.r1_c1 = cos_2*( - z_2 - y_2 ) + 1.0f;
        pH.r1_c2 = cos_2*x*y - sin_2*z;
        pH.r1_c3 = cos_2*x*z + sin_2*y;
        pH.r2_c1 = sin_2*z + cos_2*x*y;
        pH.r2_c2 = cos_2*( - z_2 - x_2 ) + 1.0f;
        pH.r2_c3 = cos_2*y*z - sin_2*x;
        pH.r3_c1 = cos_2*x*z - sin_2*y;
        pH.r3_c2 = cos_2*y*z + sin_2*x;
        pH.r3_c3 = cos_2*( - y_2 - x_2 ) + 1.0f;

      }
      else
      {
        pH.r1_c1 = cos_1*( - z_2 - y_2 ) + 1.0f;
        pH.r1_c2 = cos_1*x*y - sin_1*z;
        pH.r1_c3 = cos_1*x*z + sin_1*y;
        pH.r2_c1 = sin_1*z + cos_1*x*y;
        pH.r2_c2 = cos_1*( - z_2 - x_2 ) + 1.0f;
        pH.r2_c3 = cos_1*y*z - sin_1*x;
        pH.r3_c1 = cos_1*x*z - sin_1*y;
        pH.r3_c2 = cos_1*y*z + sin_1*x;
        pH.r3_c3 = cos_1*( - y_2 - x_2 ) + 1.0f;
      }
    } // end axisRotationProjectionInPlace


    // Main
    void transformFromRotVecInPlace(
        const int                   pAxis,
        const float                 pTheta,
        const AL::Math::Position3D& pM,
        Transform&                  pT)
    {
      // Usefull initialization
      pT = AL::Math::Transform();

      const float c = std::cos(pTheta);
      const float s = std::sin(pTheta);

      pT.r1_c4 = pM.x;
      pT.r2_c4 = pM.y;
      pT.r3_c4 = pM.z;

      switch (pAxis)
      {
      case AL::Math::AXIS_MASK_X:
        pT.r2_c2 = c;
        pT.r2_c3 = -s;
        pT.r3_c2 = s;
        pT.r3_c3 = c;
        break;

      case AL::Math::AXIS_MASK_Y:
        pT.r1_c1 = c;
        pT.r1_c3 = s;
        pT.r3_c1 = -s;
        pT.r3_c3 = c;
        break;

      case AL::Math::AXIS_MASK_Z:
        pT.r1_c1 = c;
        pT.r1_c2 = -s;
        pT.r2_c1 = s;
        pT.r2_c2 = c;
        break;

      default: // identity
        break;
      }
    }


    Transform transformFromRotVec(
        const int                   pAxis,
        const float                 pTheta,
        const AL::Math::Position3D& pM)
    {
      Transform pOut;
      transformFromRotVecInPlace(pAxis, pTheta, pM, pOut);
      return pOut;
    }


    void transformFromRotVecInPlace(
        const AL::Math::Position3D& pPosition,
        AL::Math::Transform&        pTransform)
    {
      int pAxis  = AL::Math::AXIS_MASK_WX; // 8
      float pRot = 0.0f;

      transformFromRotVecInPlace(pAxis, pRot, pPosition, pTransform);
    }


    AL::Math::Transform transformFromRotVec(
        const AL::Math::Position3D& pPosition)
    {
      AL::Math::Transform transform;
      transformFromRotVecInPlace(pPosition, transform);
      return transform;
    }


    AL::Math::Transform transformFromRotVec(
        const int&   pAxis,
        const float& pRot)
    {
      AL::Math::Transform transform;

      AL::Math::Position3D pM = AL::Math::Position3D();

      transformFromRotVecInPlace(pAxis, pRot, pM, transform);

      return transform;
    }


    void orthogonalSpace(
        const Position3D& pAxis,
        Transform&        pTf)
    {
      AL::Math::Position3D axis;
      AL::Math::Position3D axis1;
      AL::Math::Position3D axis2;
      AL::Math::Position3D axis3 = pAxis/norm(pAxis);

      const float coef = 0.0001f;
      if (std::abs(axis3.x)> coef)
      {
        axis = AL::Math::Position3D(-axis3.y, axis3.x, 0.0f);
      }
      else if (std::abs(axis3.y)> coef)
      {
        axis = AL::Math::Position3D(0.0f, -axis3.z, axis3.y);
      }
      else
      {
        axis = AL::Math::Position3D(axis3.z, 0.0f, 0.0f);
      }

      axis1 = AL::Math::normalize(axis);

      axis = AL::Math::crossProduct(axis3, axis1);
      axis2 = AL::Math::normalize(axis);

      pTf.r1_c1 = axis1.x;
      pTf.r2_c1 = axis1.y;
      pTf.r3_c1 = axis1.z;

      pTf.r1_c2 = axis2.x;
      pTf.r2_c2 = axis2.y;
      pTf.r3_c2 = axis2.z;

      pTf.r1_c3 = axis3.x;
      pTf.r2_c3 = axis3.y;
      pTf.r3_c3 = axis3.z;
    }

    Transform orthogonalSpace(const Position3D& pAxis)
    {
      Transform tfOut;
      orthogonalSpace(pAxis, tfOut);
      return tfOut;
    }

    Transform transformFromQuaternion(
        const Quaternion& pQua)
    {
      Transform tfOut;

      tfOut.r1_c1 = 1.0f - 2.0f*(std::pow(pQua.y, 2) + std::pow(pQua.z, 2));
      tfOut.r1_c2 = 2.0f*(pQua.x*pQua.y - pQua.z*pQua.w);
      tfOut.r1_c3 = 2.0f*(pQua.x*pQua.z + pQua.y*pQua.w);

      tfOut.r2_c1 = 2.0f*(pQua.x*pQua.y + pQua.z*pQua.w);
      tfOut.r2_c2 = 1.0f - 2.0f*(std::pow(pQua.x, 2) + std::pow(pQua.z, 2));
      tfOut.r2_c3 = 2.0f*(pQua.y*pQua.z - pQua.x*pQua.w);

      tfOut.r3_c1 = 2.0f*(pQua.x*pQua.z - pQua.y*pQua.w);
      tfOut.r3_c2 = 2.0f*(pQua.y*pQua.z + pQua.x*pQua.w);
      tfOut.r3_c3 = 1.0f - 2.0f*(std::pow(pQua.x, 2) + std::pow(pQua.y, 2));

      return tfOut;
    }


    Quaternion quaternionFromTransform(
        const Transform& pT)
    {
      // TR2Q   Convert homogeneous transform to a unit-quaternion
      //
      //   Q = tr2q(T)
      //
      //   Return a unit quaternion corresponding to the rotational part of the
      //   homogeneous transform T.
      //
      //   See also: Q2TR

      float kx = pT.r3_c2 - pT.r2_c3; // Oz - Ay
      float ky = pT.r1_c3 - pT.r3_c1; // Ax - Nz
      float kz = pT.r2_c1 - pT.r1_c2; // Ny - Ox

      float kx1 = 0.0f;
      float ky1 = 0.0f;
      float kz1 = 0.0f;
      bool add  = false;

      if ((pT.r1_c1 >= pT.r2_c2) && (pT.r1_c1 >= pT.r3_c3))
      {
        kx1 = pT.r1_c1 - pT.r2_c2 - pT.r3_c3 + 1.0f; // Nx - Oy - Az + 1
        ky1 = pT.r2_c1 + pT.r1_c2;                   // Ny + Ox
        kz1 = pT.r3_c1 + pT.r1_c3;                   // Nz + Ax
        add = (kx >= 0.0f);
      }
      else if (pT.r2_c2 >= pT.r3_c3)
      {
        kx1 = pT.r2_c1 + pT.r1_c2;                   // Ny + Ox
        ky1 = pT.r2_c2 - pT.r1_c1 - pT.r3_c3 + 1.0f; // Oy - Nx - Az + 1
        kz1 = pT.r3_c2 + pT.r2_c3;                   // Oz + Ay
        add = (ky >= 0.0f);
      }
      else
      {
        kx1 = pT.r3_c1 + pT.r1_c3;                   // Nz + Ax
        ky1 = pT.r3_c2 + pT.r2_c3;                   // Oz + Ay
        kz1 = pT.r3_c3 - pT.r1_c1 - pT.r2_c2 + 1.0f; // Az - Nx - Oy + 1
        add = (kz >= 0.0f);
      }

      if (add)
      {
        kx = kx + kx1;
        ky = ky + ky1;
        kz = kz + kz1;
      }
      else
      {
        kx = kx - kx1;
        ky = ky - ky1;
        kz = kz - kz1;
      }

      const float nm = std::sqrt(std::pow(kx, 2) + std::pow(ky, 2) + std::pow(kz, 2));

      if (nm == 0.0f)
      {
        return Quaternion();
      }
      else
      {
        float trace = pT.r1_c1 + pT.r2_c2 + pT.r3_c3 + 1.0f;
        if (trace < 0.0f)
        {
          trace = 0.0f;
        }

        const float qs = 0.5f*std::sqrt(trace);
        const float s = std::sqrt(1.0f - std::pow(qs, 2)) / nm;
        return Quaternion(qs, s*kx, s*ky, s*kz);
      }
    } // end quaternionFromTransform


    Transform transformFromDisplacement(const Displacement& pDisp)
    {
      Transform result = transformFromQuaternion(pDisp.Q);
      result.r1_c4 = pDisp.P.x;
      result.r2_c4 = pDisp.P.y;
      result.r3_c4 = pDisp.P.z;
      return result;
    }

    Displacement displacementFromTransform(const Transform& pTrans)
    {
      Quaternion resQ = quaternionFromTransform(pTrans);
      Position3D resP = position3DFromTransform(pTrans);
      return Displacement(resP, resQ);
    }

  } // namespace Math
} // namespace AL
