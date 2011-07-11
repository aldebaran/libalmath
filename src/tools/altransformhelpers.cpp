/**
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved
*/

#include <cmath>

#include <almath/tools/altransformhelpers.h>
#include <stdexcept>
#include <almath/tools/altrigonometry.h>
#include <almath/tools/almathio.h> // tmp TODO

namespace AL {
  namespace Math {

  Transform transformFromPosition3DAndRotation(
    const float&    pX,
    const float&    pY,
    const float&    pZ,
    const Rotation& pRot)
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

  Transform transformFromPosition3DAndRotation(
    const Position3D& pPos,
    const Rotation&   pRot)
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


    Velocity6D transformLogarithme(const Transform& pH)
    {
      Velocity6D pV;
      transformLogarithme(pH, pV);
      return pV;
    }

    void transformLogarithme(
      const Transform& pH,
      Velocity6D&      pVOut)
    {
      float epsilon = 0.001f; // new

      // square root of sum of squares of the elements
      float si = 0.5f*sqrtf( (pH.r3_c2 - pH.r2_c3)*(pH.r3_c2 - pH.r2_c3) +
        (pH.r1_c3 - pH.r3_c1)*(pH.r1_c3 - pH.r3_c1) +
        (pH.r2_c1 - pH.r1_c2)*(pH.r2_c1 - pH.r1_c2) );
      float co = 0.5f*( pH.r1_c1 + pH.r2_c2 + pH.r3_c3 - 1.0f);

      float angle = atan2f(si, co);// arctan(sin/cos)

      float coeff  = 0.0f;
      float lambda = 0.0f;
      if (si < epsilon)
      {
        if (co > 1.0f - epsilon)
        {
          coeff = angle/( 2.0f*si + epsilon ); // was pow(2.0f,-52)
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
            //std::cout << "TransformLogarithme : co cas non traite" << std::endl;
          }
        }
        else
        {
          //std::cout << "TransformLogarithme : si cas non traite" << std::endl;
        }
      }
      else
      {
        coeff = angle/(2.0f*si); // was pow(2.0f,-52)
        pVOut.wxd = coeff * (pH.r3_c2 - pH.r2_c3);
        pVOut.wyd = coeff * (pH.r1_c3 - pH.r3_c1);
        pVOut.wzd = coeff * (pH.r2_c1 - pH.r1_c2);
      }

      float coeff_2 = powf(coeff, 2);

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

    } // end transformLogarithme


    void velocityExponential(
      const AL::Math::Velocity6D& pM,
      AL::Math::Transform&        tM)
    {
      float t;
      // square root of sum of squares of the elements (w.norm_Frobenius())
      t = sqrtf(pM.wxd*pM.wxd + pM.wyd*pM.wyd + pM.wzd*pM.wzd);

      float CC, SC, dSC;

      if (t >= 0.001f) // seuil
      {
        CC  = (1-cosf(t)) / (t*t); // motionCos cardinal cosinus
        SC  = sinf(t) / t;       // motionSin cardinal sinus
        // CK WTF Cyrille ... store and reuse
        dSC = (t-sinf(t)) / powf(t, 3); // motionSin cardinal sinus derivative powf
      }
      else
      {
        CC  = 0.5f;
        SC  = 1.0f - t*t / 6.0f;
        dSC = 0.166666667f;
      }

      // Maxima
      tM.r1_c1 = 1 - CC*(pM.wzd*pM.wzd + pM.wyd*pM.wyd);
      tM.r1_c2 =   - SC*pM.wzd  + CC*pM.wxd*pM.wyd;
      tM.r1_c3 =     SC*pM.wyd  + CC*pM.wxd*pM.wzd;
      tM.r2_c1 =     SC*pM.wzd  + CC*pM.wxd*pM.wyd;
      tM.r2_c2 = 1 - CC*(pM.wxd*pM.wxd + pM.wzd*pM.wzd);
      tM.r2_c3 =   - SC*pM.wxd  + CC*pM.wyd*pM.wzd;
      tM.r3_c1 =   - SC*pM.wyd  + CC*pM.wxd*pM.wzd;
      tM.r3_c2 =     SC*pM.wxd  + CC*pM.wyd*pM.wzd;
      tM.r3_c3 = 1 - CC*(pM.wxd*pM.wxd + pM.wyd*pM.wyd);

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
      velocityExponential(pM, tM);
      return tM;
    }


    void changeRepereVelocity6D(
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


    void changeReperePosition6D(
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


    void changeReperePosition3D(
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


    void changeRepereTransposePosition3D(
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


    void changeReperePosition3D(
      const Transform&  pH,
      const Position3D& pPosIn,
      Position3D&       pPosOut)
    {
      pPosOut.x = pH.r1_c1 * pPosIn.x + pH.r1_c2 * pPosIn.y + pH.r1_c3 * pPosIn.z;
      pPosOut.y = pH.r2_c1 * pPosIn.x + pH.r2_c2 * pPosIn.y + pH.r2_c3 * pPosIn.z;
      pPosOut.z = pH.r3_c1 * pPosIn.x + pH.r3_c2 * pPosIn.y + pH.r3_c3 * pPosIn.z;
    }


    void changeRepereTransposePosition3D(
      const Transform&  pH,
      const Position3D& pPosIn,
      Position3D&       pPosOut)
    {
      pPosOut.x = pH.r1_c1 * pPosIn.x + pH.r2_c1 * pPosIn.y + pH.r3_c1 * pPosIn.z;
      pPosOut.y = pH.r1_c2 * pPosIn.x + pH.r2_c2 * pPosIn.y + pH.r3_c2 * pPosIn.z;
      pPosOut.z = pH.r1_c3 * pPosIn.x + pH.r2_c3 * pPosIn.y + pH.r3_c3 * pPosIn.z;
    }


    void changeRepereTransform(
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


    void changeRepereTransposeTransform(
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


    void changeRepereTransposeVelocity6D(
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


    void changeRepereTransposePosition6D(
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


    void transformMean(
      const AL::Math::Transform&  pHIn1,
      const AL::Math::Transform&  pHIn2,
      const float&                pDist,
      AL::Math::Transform&        pHOut)
    {
      if ((pDist>1.0f) || (pDist<0.0f))
      {
        throw std::runtime_error(
          "ALMath: transformMean Distance must be between 0 and 1.");
      }

      Velocity6D pV;
      Transform pHIn1i;

      transformInverse(pHIn1,pHIn1i);
      transformLogarithme(pHIn1i*pHIn2, pV);
      velocityExponential(pDist*pV,pHOut);
      pHOut = pHIn1*pHOut;
    }


    AL::Math::Transform transformMean(
      const AL::Math::Transform&  pHIn1,
      const AL::Math::Transform&  pHIn2,
      const float&                pDist)
    {
      AL::Math::Transform pHOut;
      transformMean(pHIn1, pHIn2, pDist, pHOut);
      return pHOut;
    }


    void transformFromPosition3D(
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
      transformFromPosition3D(pPosition, transform);
      return transform;
    }


    void rotationToTransform(
      const Rotation& pIn,
      Transform&      pOut)
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


    Transform rotationToTransform(const Rotation& pIn)
    {
      Transform pOut = Transform();
      rotationToTransform(pIn, pOut);
      return pOut;
    }


    void rotationFromTransform(
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
      rotationFromTransform(pIn, pOut);
      return pOut;
    }

    void transformToPosition3D(
      const Transform& pTransform,
      Position3D&      pPosition)
    {
      pPosition.x = pTransform.r1_c4;
      pPosition.y = pTransform.r2_c4;
      pPosition.z = pTransform.r3_c4;
    }


    Position3D transformToPosition3D(const Transform& pTransform)
    {
      return Position3D(
            pTransform.r1_c4,
            pTransform.r2_c4,
            pTransform.r3_c4);
    }


    void position6DFromTransform(
        const Transform& pT,
        Position6D&      pPos)
    {
      pPos.x = pT.r1_c4;
      pPos.y = pT.r2_c4;
      pPos.z = pT.r3_c4;
      pPos.wz = atan2(pT.r2_c1, pT.r1_c1);
      float sy = sinf(pPos.wz);
      float cy = cosf(pPos.wz);
      pPos.wy = atan2(-pT.r3_c1, cy*pT.r1_c1+sy*pT.r2_c1);
      pPos.wx = atan2(sy*pT.r1_c3-cy*pT.r2_c3, cy*pT.r2_c2-sy*pT.r1_c2);
    }


    Position6D position6DFromTransform(const Transform& pT)
    {
      Position6D pPos;
      position6DFromTransform(pT, pPos);
      return pPos;
    }


    void transformFromPose2D(
        const Pose2D& pPose,
        Transform&    pT)
    {
      pT = Transform(pPose.x, pPose.y, 0.0f);
      pT *= transformFromRotZ(pPose.theta);
    }


    Transform transformFromPose2D(const Pose2D& pPos)
    {
      Transform T = Transform();
      transformFromPose2D(pPos, T);
      return T;
    }


    void pose2DFromTransform(
        const Transform& pT,
        Pose2D&          pPos)
    {
      pPos.x = pT.r1_c4;
      pPos.y = pT.r2_c4;
      pPos.theta = atan2(pT.r2_c1, pT.r1_c1);
    }


    Pose2D pose2DFromTransform(const Transform& pT)
    {
      Pose2D pPos;
      pose2DFromTransform(pT, pPos);
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


    void transformDiffToPosition(
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


    Position6D transformDiffToPosition(
      const Transform& pCurrent,
      const Transform& pTarget)
    {
      Position6D result;
      transformDiffToPosition(pCurrent, pTarget, result);
      return result;
    }


    void position3DFromTransformInPlace(
      const Transform& pH,
      Position3D&      pOut)
    {
      pOut.x = pH.r1_c4;
      pOut.y = pH.r2_c4;
      pOut.z = pH.r3_c4;
    }


    Position3D position3DFromTransform(const Transform& pH)
    {
      Position3D pOut;
      position3DFromTransformInPlace(pH, pOut);
      return pOut;
    }


    Transform rotationPosition3DToTransform(
      const Rotation&   pRot,
      const Position3D& pPos)
    {
      Transform pOut;
      pOut.r1_c1 = pRot.r1_c1;
      pOut.r1_c2 = pRot.r1_c2;
      pOut.r1_c3 = pRot.r1_c3;

      pOut.r2_c1 = pRot.r2_c1;
      pOut.r2_c2 = pRot.r2_c2;
      pOut.r2_c3 = pRot.r2_c3;

      pOut.r3_c1 = pRot.r3_c1;
      pOut.r3_c2 = pRot.r3_c2;
      pOut.r3_c3 = pRot.r3_c3;

      pOut.r1_c4 = pPos.x;
      pOut.r2_c4 = pPos.y;
      pOut.r3_c4 = pPos.z;

      return pOut;
    }


    Rotation3D rotation3DFromTransform(const Transform& pT)
    {
      Rotation3D R;
      R.wz = atan2(pT.r2_c1,pT.r1_c1);
      float sy = sinf(R.wz);
      float cy = cosf(R.wz);
      R.wy = atan2(-pT.r3_c1, cy*pT.r1_c1+sy*pT.r2_c1);
      R.wx = atan2(sy*pT.r1_c3-cy*pT.r2_c3, cy*pT.r2_c2-sy*pT.r1_c2);
      return R;
    }


    Rotation3D rotation3DFromRotation(const Rotation& pR)
    {
      Rotation3D R;
      R.wz = atan2(pR.r2_c1,pR.r1_c1);
      float sy = sinf(R.wz);
      float cy = cosf(R.wz);
      R.wy = atan2(-pR.r3_c1, cy*pR.r1_c1+sy*pR.r2_c1);
      R.wx = atan2(sy*pR.r1_c3-cy*pR.r2_c3, cy*pR.r2_c2-sy*pR.r1_c2);
      return R;
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


    Transform& operator+=(Transform& pT, const Position3D& pPos)
    {
      pT.r1_c4 += pPos.x;
      pT.r2_c4 += pPos.y;
      pT.r3_c4 += pPos.z;
      return pT;
    }


//    Position6D POSITION6D(
//      const Position3D& pPos,
//      const Transform&  pHRot)
//    {
//      Position6D result;
//      Rotation3D pRot = Rotation3DFromTransform(pHRot);
//      result.x = pPos.x;
//      result.y = pPos.y;
//      result.z = pPos.z;
//      result.wx = pRot.wx;
//      result.wy = pRot.wy;
//      result.wz = pRot.wz;
//      return result;
//    }


    void axisRotationProjection(
      const Position3D& pAxis,
      Transform&        pH)
    {
      // Usefull tempo save ?
      float x = pH.r1_c4;
      float y = pH.r2_c4;
      float z = pH.r3_c4;

      AL::Math::Rotation pRotationSolution;
      rotationFromTransform(pH, pRotationSolution);

      // Compute Projection
      axisRotationProjectionInPlace(pRotationSolution, pAxis);

      // Rotation to Transform
      rotationToTransform(pRotationSolution, pH);

      // Save position part of Transform in the solution
      pH.r1_c4 = x;
      pH.r2_c4 = y;
      pH.r3_c4 = z;
    } // end AxisRotationProjection


    Transform axisRotationProjection(
      const Transform&  pH,
      const Position3D& pAxis)
    {
      AL::Math::Transform pHSol;

      // Save position part of Transform in the solution
      AL::Math::Rotation pRotationSolution;
      rotationFromTransform(pH, pRotationSolution);
      axisRotationProjectionInPlace(pRotationSolution, pAxis);

      rotationToTransform(pRotationSolution, pHSol);
      pHSol.r1_c4 = pH.r1_c4;
      pHSol.r2_c4 = pH.r2_c4;
      pHSol.r3_c4 = pH.r3_c4;

      return pHSol;
    } // end axisRotationProjection


    Rotation axisRotationProjection(
      const Rotation&   pRot,
      const Position3D& pAxis)
    {
      Rotation pOut = pRot;
      axisRotationProjectionInPlace(pOut, pAxis);
      return pOut;
    }


    // NOTE: although this is not about transforms, it is here because
    // other methods here use it.
    void axisRotationProjectionInPlace(
      Rotation&         pRot,
      const Position3D& pAxis)
    {
      float inw = norm(pAxis);
      if (inw == 0.0f)
      {
        throw std::runtime_error(
          "ALMath: axisRotationProjectionInPlace Division by zeros.");
      }

      inw = 1.0f/inw;

      float x = inw * pAxis.x;
      float y = inw * pAxis.y;
      float z = inw * pAxis.z;

      float x_2 = powf(x, 2);
      float y_2 = powf(y, 2);
      float z_2 = powf(z, 2);

      float a =
        x*(pRot.r2_c3 - z*(x*pRot.r2_c1 + y*pRot.r2_c2 + z*pRot.r2_c3)) +
        y*(pRot.r3_c1 - x*(x*pRot.r3_c1 + y*pRot.r3_c2 + z*pRot.r3_c2)) +
        z*(pRot.r1_c2 - y*(x*pRot.r1_c1 + y*pRot.r1_c2 + z*pRot.r1_c3));

      float b =
        x*(-pRot.r3_c1*z + pRot.r3_c3*x) +
        y*( pRot.r1_c1*y - pRot.r1_c2*x) +
        z*( pRot.r2_c2*z - pRot.r2_c3*y);

      float c =
        x*(z*x*pRot.r2_c1 + z*y*pRot.r2_c2 + z*z*pRot.r2_c3) +
        y*(x*x*pRot.r3_c1 + x*y*pRot.r3_c2 + x*z*pRot.r3_c3) +
        z*(y*x*pRot.r1_c1 + y*y*pRot.r1_c2 + y*z*pRot.r1_c3);

      float d2 = a*a + b*b - c*c;

      if (d2<0)
      {
        throw std::runtime_error(
          "ALMath: axisRotationProjectionInPlace d2 < 0");
        return;
      }

      float alpha  = atan2f( b , a );
      float beta   = acosf( c / sqrtf( a*a + b*b ) );

      float cos_1 = 1.0f - cosf( alpha + beta );
      float cos_2 = 1.0f - cosf( alpha - beta );

      float sin_1 = sinf( alpha + beta );
      float sin_2 = sinf( alpha - beta );

      float trace_1 = pRot.r1_c1*( cos_1*( - z_2 - y_2 ) + 1.0f ) +
        pRot.r2_c2*( cos_1*( - z_2 - x_2 ) + 1.0f ) +
        pRot.r3_c2*( cos_1*y*z + sin_1*x ) +
        pRot.r2_c3*( cos_1*y*z - sin_1*x ) +
        pRot.r1_c3*( cos_1*x*z + sin_1*y ) +
        pRot.r3_c1*( cos_1*x*z - sin_1*y ) +
        pRot.r2_c1*( sin_1*z + cos_1*x*y ) +
        pRot.r1_c2*( cos_1*x*y - sin_1*z ) +
        pRot.r3_c3*( cos_1*( - y_2 - x_2 ) + 1.0f );

      float trace_2 = pRot.r1_c1*( cos_2*( - z_2 - y_2 ) + 1.0f ) +
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
          "ALMath: axisRotationProjectionInPlace Division by zeros.");
      }

      inw = 1.0f/inw;

      float x = inw * pAxis.x;
      float y = inw * pAxis.y;
      float z = inw * pAxis.z;

      float x_2 = powf(x, 2);
      float y_2 = powf(y, 2);
      float z_2 = powf(z, 2);

      float a =
        x*(pH.r2_c3 - z*(x*pH.r2_c1 + y*pH.r2_c2 + z*pH.r2_c3)) +
        y*(pH.r3_c1 - x*(x*pH.r3_c1 + y*pH.r3_c2 + z*pH.r3_c2)) +
        z*(pH.r1_c2 - y*(x*pH.r1_c1 + y*pH.r1_c2 + z*pH.r1_c3));

      float b =
        x*(-pH.r3_c1*z + pH.r3_c3*x) +
        y*( pH.r1_c1*y - pH.r1_c2*x) +
        z*( pH.r2_c2*z - pH.r2_c3*y);

      float c =
        x*(z*x*pH.r2_c1 + z*y*pH.r2_c2 + z*z*pH.r2_c3) +
        y*(x*x*pH.r3_c1 + x*y*pH.r3_c2 + x*z*pH.r3_c3) +
        z*(y*x*pH.r1_c1 + y*y*pH.r1_c2 + y*z*pH.r1_c3);

      float d2 = a*a + b*b - c*c;

      if (d2<0)
      {
        throw std::runtime_error(
          "ALMath: axisRotationProjectionInPlace d2 < 0");
        return;
      }

      float alpha  = atan2f(b, a);
      float beta   = acosf(c / sqrtf( a*a + b*b ));

      float cos_1 = 1.0f - cosf( alpha + beta );
      float cos_2 = 1.0f - cosf( alpha - beta );

      float sin_1 = sinf( alpha + beta );
      float sin_2 = sinf( alpha - beta );

      float trace_1 = pH.r1_c1*( cos_1*( - z_2 - y_2 ) + 1.0f ) +
        pH.r2_c2*( cos_1*( - z_2 - x_2 ) + 1.0f ) +
        pH.r3_c2*( cos_1*y*z + sin_1*x ) +
        pH.r2_c3*( cos_1*y*z - sin_1*x ) +
        pH.r1_c3*( cos_1*x*z + sin_1*y ) +
        pH.r3_c1*( cos_1*x*z - sin_1*y ) +
        pH.r2_c1*( sin_1*z + cos_1*x*y ) +
        pH.r1_c2*( cos_1*x*y - sin_1*z ) +
        pH.r3_c3*( cos_1*( - y_2 - x_2 ) + 1.0f );

      float trace_2 = pH.r1_c1*( cos_2*( - z_2 - y_2 ) + 1.0f ) +
        pH.r2_c2*( cos_2*( - z_2 - x_2 ) + 1.0f ) +
        pH.r3_c2*( cos_2*y*z + sin_2*x ) +
        pH.r2_c3*( cos_2*y*z - sin_2*x ) +
        pH.r1_c3*( cos_2*x*z + sin_2*y ) +
        pH.r3_c1*( cos_2*x*z - sin_2*y ) +
        pH.r2_c1*( sin_2*z + cos_2*x*y ) +
        pH.r1_c2*( cos_2*x*y - sin_2*z ) +
        pH.r3_c3*( cos_2*( - y_2 - x_2 ) + 1.0f );

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
    void rotVecToTransform(
      const int                   pAxis,
      const float                 pTheta,
      const AL::Math::Position3D& pM,
      Transform&                  pH)
    {
      // Usefull initialization
      pH = AL::Math::Transform();

      float c = cosf(pTheta); // degree to radian
      float s = sinf(pTheta); // degree to radian

      pH.r1_c4 = pM.x;
      pH.r2_c4 = pM.y;
      pH.r3_c4 = pM.z;

      switch (pAxis)
      {
      case AL::Math::AXIS_MASK_X:
        pH.r2_c2 = c;
        pH.r2_c3 = -s;
        pH.r3_c2 = s;
        pH.r3_c3 = c;
        break;

      case AL::Math::AXIS_MASK_Y:
        pH.r1_c1 = c;
        pH.r1_c3 = s;
        pH.r3_c1 = -s;
        pH.r3_c3 = c;
        break;

      case AL::Math::AXIS_MASK_Z:
        pH.r1_c1 = c;
        pH.r1_c2 = -s;
        pH.r2_c1 = s;
        pH.r2_c2 = c;
        break;

      default: // identity
        break;
      }
    }


    Transform rotVecToTransform(
      const int                   pAxis,
      const float                 pTheta,
      const AL::Math::Position3D& pM)
    {
      Transform pOut;
      rotVecToTransform(pAxis, pTheta, pM, pOut);
      return pOut;
    }


    void rotVecToTransform(
      const AL::Math::Position3D& pPosition,
      AL::Math::Transform&        pTransform)
    {
      int pAxis  = AL::Math::AXIS_MASK_WX; // 8
      float pRot = 0.0f;

      rotVecToTransform(pAxis, pRot, pPosition, pTransform);
    }


    AL::Math::Transform rotVecToTransform(
      const AL::Math::Position3D& pPosition)
    {
      AL::Math::Transform transform;
      rotVecToTransform(pPosition, transform);
      return transform;
    }


    AL::Math::Transform rotVecToTransform(
      const int&   pAxis,
      const float& pRot)
    {
      AL::Math::Transform transform;

      AL::Math::Position3D pM = AL::Math::Position3D();

      rotVecToTransform(pAxis, pRot, pM, transform);

      return transform;
    }

  } // namespace Math
} // namespace AL
