/**
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved \n
*
*/

#include <almath/tools/almath.h>

#include <cmath>

namespace AL
{
  namespace Math
  {

    bool clipData(
      const float& pMin,
      const float& pMax,
      float& pData)
    {
      if (pData < pMin)
      {
        pData = pMin;
        return true;
      }
      else if (pData > pMax)
      {
        pData = pMax;
        return true;
      }
      else
      {
        return false;
      }
    }


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


    Position6D position6DFromVelocity6D(const Velocity6D& pVel)
    {
      Position6D pos;
      pos.x  = pVel.xd;
      pos.y  = pVel.yd;
      pos.z  = pVel.zd;
      pos.wx = pVel.wxd;
      pos.wy = pVel.wyd;
      pos.wz = pVel.wzd;
      return pos;
    }

    Position3D operator*(
      const Rotation&   pRot,
      const Position3D& pPos)
    {
      Position3D result;
      result.x = (pRot.r1_c1 * pPos.x) + (pRot.r1_c2 * pPos.y) + (pRot.r1_c3 * pPos.z);
      result.y = (pRot.r2_c1 * pPos.x) + (pRot.r2_c2 * pPos.y) + (pRot.r2_c3 * pPos.z);
      result.z = (pRot.r3_c1 * pPos.x) + (pRot.r3_c2 * pPos.y) + (pRot.r3_c3 * pPos.z);
      return result;
    }

    Velocity6D operator*(
      const float       pVal,
      const Position6D& pPos)
    {
      /** cyrille 27/04/2009 static ? **/

      Velocity6D pVel;
      pVel.xd  = pVal * pPos.x;
      pVel.yd  = pVal * pPos.y;
      pVel.zd  = pVal * pPos.z;
      pVel.wxd = pVal * pPos.wx;
      pVel.wyd = pVal * pPos.wy;
      pVel.wzd = pVal * pPos.wz;
      return pVel;
    }


    AL::Math::Rotation rotationFromAngleDirection(
      const float&                pTheta,
      const AL::Math::Position3D& pPos)
    {
      return AL::Math::rotationFromAngleDirection(
            pTheta,
            pPos.x,
            pPos.y,
            pPos.z);
    }

    void applyRotation(
      const AL::Math::Rotation& pRotation,
      AL::Math::Position3D&     pVector)
    {
      float x = pVector.x;
      float y = pVector.y;
      float z = pVector.z;
      pVector.x = x*pRotation.r1_c1 + y*pRotation.r1_c2 + z*pRotation.r1_c3;
      pVector.y = x*pRotation.r2_c1 + y*pRotation.r2_c2 + z*pRotation.r2_c3;
      pVector.z = x*pRotation.r3_c1 + y*pRotation.r3_c2 + z*pRotation.r3_c3;
    }


  } // namespace Math
} // namespace AL

