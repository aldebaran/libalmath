/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
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

    Position3D operator*(
        const Quaternion& pQuat,
        const Position3D& pPos) {
      Position3D result;
      result.x = pQuat.w * pQuat.w * pPos.x + 2.0f *pQuat.y * pQuat.w * pPos.z;
      result.x += - 2.0f * pQuat.z * pQuat.w * pPos.y + pQuat.x * pQuat.x * pPos.x;
      result.x += 2.0f * pQuat.y * pQuat.x * pPos.y + 2.0f * pQuat.z * pQuat.x * pPos.z;
      result.x += - pQuat.z * pQuat.z * pPos.x - pQuat.y * pQuat.y * pPos.x;

      result.y = 2.0f * pQuat.x * pQuat.y * pPos.x + pQuat.y * pQuat.y * pPos.y;
      result.y += 2.0f * pQuat.z * pQuat.y * pPos.z + 2.0f * pQuat.w * pQuat.z * pPos.x;
      result.y += - pQuat.z * pQuat.z * pPos.y + pQuat.w * pQuat.w * pPos.y;
      result.y += - 2.0f * pQuat.x * pQuat.w * pPos.z - pQuat.x * pQuat.x * pPos.y;

      result.z = 2.0f * pQuat.x * pQuat.z * pPos.x + 2.0f * pQuat.y * pQuat.z * pPos.y;
      result.z += pQuat.z * pQuat.z * pPos.z - 2.0f * pQuat.w * pQuat.y * pPos.x;
      result.z += - pQuat.y * pQuat.y * pPos.z + 2.0f * pQuat.w * pQuat.x * pPos.y;
      result.z += - pQuat.x * pQuat.x * pPos.z + pQuat.w * pQuat.w * pPos.z;
      return result;
    }

    Velocity6D operator*(
      const float       pVal,
      const Position6D& pPos)
    {
      Velocity6D pVel;
      pVel.xd  = pVal * pPos.x;
      pVel.yd  = pVal * pPos.y;
      pVel.zd  = pVal * pPos.z;
      pVel.wxd = pVal * pPos.wx;
      pVel.wyd = pVal * pPos.wy;
      pVel.wzd = pVal * pPos.wz;
      return pVel;
    }

    Velocity3D operator*(
      const float       pVal,
      const Position3D& pPos)
    {
      Velocity3D pVel;
      pVel.xd  = pVal * pPos.x;
      pVel.yd  = pVal * pPos.y;
      pVel.zd  = pVal * pPos.z;
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
  } // namespace Math
} // namespace AL

