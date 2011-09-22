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
      pPos.wz = atan2(pT.r2_c1, pT.r1_c1);
      float sy = sinf(pPos.wz);
      float cy = cosf(pPos.wz);
      pPos.wy = atan2(-pT.r3_c1, cy*pT.r1_c1+sy*pT.r2_c1);
      pPos.wx = atan2(sy*pT.r1_c3-cy*pT.r2_c3, cy*pT.r2_c2-sy*pT.r1_c2);
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


    void pose2DFromTransformInPlace(
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


    Transform axisRotationProjection(
      const Position3D& pPos,
      const Transform&  pT)
    {
      AL::Math::Transform pTSol;

      // Save position part of Transform in the solution
      AL::Math::Rotation pRotationSolution;
      rotationFromTransformInPlace(pT, pRotationSolution);
      axisRotationProjectionInPlace(pPos, pRotationSolution);

      transformFromRotationInPlace(pRotationSolution, pTSol);
      pTSol.r1_c4 = pT.r1_c4;
      pTSol.r2_c4 = pT.r2_c4;
      pTSol.r3_c4 = pT.r3_c4;

      return pTSol;
    } // end axisRotationProjection


    Rotation axisRotationProjection(
      const Position3D& pPos,
      const Rotation&   pRot)
    {
      Rotation pOut = pRot;
      axisRotationProjectionInPlace(pPos, pOut);
      return pOut;
    }


    // NOTE: although this is not about transforms, it is here because
    // other methods here use it.
    void axisRotationProjectionInPlace(
      const Position3D& pAxis,
      Rotation&         pRot)
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
    void transformFromRotVecInPlace(
      const int                   pAxis,
      const float                 pTheta,
      const AL::Math::Position3D& pM,
      Transform&                  pT)
    {
      // Usefull initialization
      pT = AL::Math::Transform();

      float c = cosf(pTheta); // degree to radian
      float s = sinf(pTheta); // degree to radian

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
      Transform&        HOut)
    {
      //Transform HOut;
      Position3D pAxisTmp = Position3D();
      Position3D pAxis3 = pAxis/norm(pAxis);
      Position3D pAxis1 = Position3D();
      Position3D pAxis2 = Position3D();

      // Trouver l indice du premier element non nul
      float coef = 0.0001f;
      if (fabsf(pAxis3.x)> coef)
      {
        pAxisTmp = Position3D(-pAxis3.y, pAxis3.x, 0.0f);
      }
      else if (fabsf(pAxis3.y)> coef)
      {
        pAxisTmp = Position3D(0.0f, -pAxis3.z, pAxis3.y);
      }
      else
      {
        pAxisTmp = Position3D(pAxis3.z, 0.0f, 0.0f);
      }

      pAxis1 = AL::Math::normalize(pAxisTmp);

      pAxisTmp = crossProduct(pAxis3, pAxis1); // was pAxis3, pAxis1
      pAxis2 = AL::Math::normalize(pAxisTmp);

      HOut.r1_c1 = pAxis1.x;
      HOut.r2_c1 = pAxis1.y;
      HOut.r3_c1 = pAxis1.z;

      HOut.r1_c2 = pAxis2.x;
      HOut.r2_c2 = pAxis2.y;
      HOut.r3_c2 = pAxis2.z;

      HOut.r1_c3 = pAxis3.x;
      HOut.r2_c3 = pAxis3.y;
      HOut.r3_c3 = pAxis3.z;
    }

    // ORTHSPACE: Plan orthogonal d'une droite vectorielle
    //
    // Retourne une base orthonormee de l espace orthogonal au vecteur e3, choisie de
    // telle sorte que base_espace = [e1 e2 e3] soit une base orthogonale directe de
    // l espace R3 tout entier (et meme, une base orthonormee directe, si e3 est
    // norme).
    Transform orthogonalSpace(const Position3D& pAxis)
    {
      Transform HOut;
      orthogonalSpace(pAxis, HOut);
      return HOut;
    }

  } // namespace Math
} // namespace AL
