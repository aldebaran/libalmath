/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/tools/almath.h>
#include <almath/tools/altrigonometry.h>

#include <cmath>

namespace AL
{
  namespace Math
  {

    void modulo2PIInPlace(float& pAngle)
    {
      pAngle = fmod(pAngle, AL::Math::_2_PI_);

      if (pAngle>AL::Math::PI)
      {
        pAngle = pAngle - AL::Math::_2_PI_;
      }
      else if (pAngle<-AL::Math::PI)
      {
        pAngle = pAngle + AL::Math::_2_PI_;
      }
    }

    float modulo2PI(float pAngle)
    {
      float result = pAngle;
      modulo2PIInPlace(result);
      return result;
    }

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

      return false;
    }


    void changeReferencePose2D(
        const float&  pTheta,
        const Pose2D& pPosIn,
        Pose2D&       pPosOut)
    {
      const float cs = std::cos(pTheta);
      const float sn = std::sin(pTheta);
      pPosOut.x = cs*pPosIn.x - sn*pPosIn.y;
      pPosOut.y = sn*pPosIn.x + cs*pPosIn.y;
      pPosOut.theta = pPosIn.theta;
    }


    void changeReferencePose2DInPlace(
        const float& pTheta,
        Pose2D&      pPosOut)
    {
      const float cs = std::cos(pTheta);
      const float sn = std::sin(pTheta);
      const float xOld = pPosOut.x;
      const float yOld = pPosOut.y;

      pPosOut.x = cs*xOld - sn*yOld;
      pPosOut.y = sn*xOld + cs*yOld;
    }


    Position3D position3DFromPosition6D(const Position6D& pPose6d)
    {
      return Position3D(pPose6d.x, pPose6d.y, pPose6d.z);
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
        const Position3D& pPos)
    {
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


    void position2DFromPose2DInPlace(
        const Pose2D& pPose2d,
        Position2D&   pPosition2d)
    {
      pPosition2d.x = pPose2d.x;
      pPosition2d.y = pPose2d.y;
    }


    Position2D position2DFromPose2D(const Pose2D& pPose2d)
    {
      Position2D position2d;
      position2DFromPose2DInPlace(pPose2d, position2d);
      return position2d;
    }


    void position6DFromPose2DInPlace(
        const Pose2D& pPose2d,
        Position6D&   pPose6d)
    {
      pPose6d = Position6D(pPose2d.x, pPose2d.y, 0.0f, 0.0f, 0.0f, pPose2d.theta);
    }


    Position6D position6DFromPose2D(const Pose2D& pPose2d)
    {
      Position6D pose6d = Position6D();
      position6DFromPose2DInPlace(pPose2d, pose6d);
      return pose6d;
    }


    void pose2DFromPosition6DInPlace(
        const Position6D& pPose6d,
        Pose2D&           pPose2d)
    {
      pPose2d.x     = pPose6d.x;
      pPose2d.y     = pPose6d.y;
      pPose2d.theta = pPose6d.wz;
    }


    Pose2D pose2DFromPosition6D(const Position6D& pPose6d)
    {
      Pose2D pPose2d;
      AL::Math::pose2DFromPosition6DInPlace(pPose6d, pPose2d);
      return pPose2d;
    }


    void pose2DFromPosition2DInPlace(
        const Position2D& pPosition2d,
        const float       pAngle,
        Pose2D&           pPose2d)
    {
      pPose2d.x     = pPosition2d.x;
      pPose2d.y     = pPosition2d.y;
      pPose2d.theta = pAngle;
    }


    Pose2D pose2DFromPosition2D(const Position2D& pPosition2d,
                                const float       pAngle)
    {
      Pose2D pPose2d;
      AL::Math::pose2DFromPosition2DInPlace(pPosition2d, pAngle, pPose2d);
      return pPose2d;
    }


    Position2D operator*(
      const Pose2D&     pVal,
      const Position2D& pPos)
    {
      return Position2D(
            pVal.x + std::cos(pVal.theta)*pPos.x - std::sin(pVal.theta)*pPos.y,
            pVal.y + std::sin(pVal.theta)*pPos.x + std::cos(pVal.theta)*pPos.y);
    }

  } // namespace Math
} // namespace AL

