/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/tools/almath.h>
#include <almath/tools/altrigonometry.h>

#include <cmath>
#include <boost/math/special_functions/pow.hpp>
#include <boost/algorithm/clamp.hpp>

namespace AL
{
  namespace Math
  {

    void modulo2PIInPlace(float& pAngle)
    {
      pAngle = std::fmod(pAngle, AL::Math::_2_PI_);

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

    float meanAngle(const std::vector<float> &pAngles) {
      const std::vector<float> weights(pAngles.size(), 1.f);
      return weightedMeanAngle(pAngles, weights);
    }

    float weightedMeanAngle(const std::vector<float> &pAngles,
                            const std::vector<float> &pWeights) {
      if (pAngles.size() != pWeights.size()) {
        throw std::runtime_error("Angles and weights must have the same size.");
      }
      Math::Position2D sumPosition;
      std::vector<float>::const_iterator a = pAngles.begin();
      std::vector<float>::const_iterator w = pWeights.begin();
      float weightSum = 0.f;
      for (; a != pAngles.end(); ++a, ++w) {
        if (*w <= 0.f) {
          throw std::runtime_error("All weights must be strictly positive.");
        }
        weightSum += *w;
        sumPosition += *w * Math::Position2D(std::cos(*a), std::sin(*a));
      }
      if (sumPosition.norm() < 1e-3f) {
        throw std::runtime_error("No defined mean.");
      }
      return Math::modulo2PI(std::atan2(sumPosition.y, sumPosition.x));
    }

    bool clipData(
      const float& pMin,
      const float& pMax,
      float& pData)
    {
      const float saved = pData;
      pData = boost::algorithm::clamp(saved, pMin, pMax);
      return saved != pData;
    }

    bool clipData(
      const float& pMin,
      const float& pMax,
      std::vector<float>& pData)
    {
      bool isClipped = false;
      for (unsigned int i=0; i<pData.size(); ++i)
      {
        if (clipData(pMin, pMax, pData[i]))
        {
          isClipped = true;
        }
      }
      return isClipped;
    }

    bool clipData(
      const float& pMin,
      const float& pMax,
      std::vector<std::vector<float> >& pData)
    {
      bool isClipped = false;
      for (unsigned int i=0; i<pData.size(); ++i)
      {
        if (clipData(pMin, pMax, pData[i]))
        {
          isClipped = true;
        }
      }
      return isClipped;
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


    Position3D position3DFromPosition6D(const Position6D& pPosition6D)
    {
      return Position3D(pPosition6D.x, pPosition6D.y, pPosition6D.z);
    }


    Position6D position6DFromVelocity6D(const Velocity6D& pVel)
    {
      return Position6D(
            pVel.xd,
            pVel.yd,
            pVel.zd,
            pVel.wxd,
            pVel.wyd,
            pVel.wzd);
    }

    Position3D operator*(
      const Rotation&   pRot,
      const Position3D& pPos)
    {
      return Position3D(
            pRot.r1_c1*pPos.x + pRot.r1_c2*pPos.y + pRot.r1_c3*pPos.z,
            pRot.r2_c1*pPos.x + pRot.r2_c2*pPos.y + pRot.r2_c3*pPos.z,
            pRot.r3_c1*pPos.x + pRot.r3_c2*pPos.y + pRot.r3_c3*pPos.z);
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
      return Velocity6D(
            pVal * pPos.x,
            pVal * pPos.y,
            pVal * pPos.z,
            pVal * pPos.wx,
            pVal * pPos.wy,
            pVal * pPos.wz);
    }

    Velocity3D operator*(
      const float       pVal,
      const Position3D& pPos)
    {
      return Velocity3D(
            pVal * pPos.x,
            pVal * pPos.y,
            pVal * pPos.z);
    }

    Velocity3D operator*(
      const Rotation&   pRot,
      const Velocity3D& pVel)
    {
      return Velocity3D(
            pRot.r1_c1*pVel.xd + pRot.r1_c2*pVel.yd + pRot.r1_c3*pVel.zd,
            pRot.r2_c1*pVel.xd + pRot.r2_c2*pVel.yd + pRot.r2_c3*pVel.zd,
            pRot.r3_c1*pVel.xd + pRot.r3_c2*pVel.yd + pRot.r3_c3*pVel.zd);
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
        const Pose2D& pPose2D,
        Position2D&   pPosition2D)
    {
      pPosition2D.x = pPose2D.x;
      pPosition2D.y = pPose2D.y;
    }


    Position2D position2DFromPose2D(const Pose2D& pPose2D)
    {
      return Position2D(pPose2D.x, pPose2D.y);
    }


    void position6DFromPose2DInPlace(
        const Pose2D& pPose2D,
        Position6D&   pPosition6D)
    {
      pPosition6D.x  = pPose2D.x;
      pPosition6D.y  = pPose2D.y;
      pPosition6D.z  = 0.0f;
      pPosition6D.wx = 0.0f;
      pPosition6D.wy = 0.0f;
      pPosition6D.wz = pPose2D.theta;
    }


    Position6D position6DFromPose2D(const Pose2D& pPose2D)
    {
      return Position6D(
            pPose2D.x,
            pPose2D.y,
            0.0f,
            0.0f,
            0.0f,
            pPose2D.theta);
    }


    void position6DFromPosition3DInPlace(
        const Position3D& pPosition3D,
        Position6D&       pPosition6D)
    {
      pPosition6D.x  = pPosition3D.x;
      pPosition6D.y  = pPosition3D.y;
      pPosition6D.z  = pPosition3D.z;
      pPosition6D.wx = 0.0f;
      pPosition6D.wy = 0.0f;
      pPosition6D.wz = 0.0f;
    }

    Position6D position6DFromPosition3D(const Position3D& pPosition3D)
    {
      return Position6D(pPosition3D.x,
                        pPosition3D.y,
                        pPosition3D.z,
                        0.0f,
                        0.0f,
                        0.0f);
    }

    void pose2DFromPosition6DInPlace(
        const Position6D& pPosition6D,
        Pose2D&           pPose2D)
    {
      pPose2D.x     = pPosition6D.x;
      pPose2D.y     = pPosition6D.y;
      pPose2D.theta = pPosition6D.wz;
    }


    Pose2D pose2DFromPosition6D(const Position6D& pPosition6D)
    {
      return Pose2D(pPosition6D.x, pPosition6D.y, pPosition6D.wz);
    }


    void pose2DFromPosition2DInPlace(
        const Position2D& pPosition2D,
        const float       pAngle,
        Pose2D&           pPose2D)
    {
      pPose2D.x     = pPosition2D.x;
      pPose2D.y     = pPosition2D.y;
      pPose2D.theta = pAngle;
    }


    Pose2D pose2DFromPosition2D(const Position2D& pPosition2D,
                                const float       pAngle)
    {
      return Pose2D(pPosition2D.x, pPosition2D.y, pAngle);
    }


    Position2D operator*(
      const Pose2D&     pVal,
      const Position2D& pPos)
    {
      return Position2D(
            pVal.x + std::cos(pVal.theta)*pPos.x - std::sin(pVal.theta)*pPos.y,
            pVal.y + std::sin(pVal.theta)*pPos.x + std::cos(pVal.theta)*pPos.y);
    }

    void quaternionFromRotation3D(
        const Rotation3D& pRot3D,
        Quaternion& pQuaternion)
    {
      const float cx = std::cos(0.5f*pRot3D.wx);
      const float cy = std::cos(0.5f*pRot3D.wy);
      const float cz = std::cos(0.5f*pRot3D.wz);
      const float sx = std::sin(0.5f*pRot3D.wx);
      const float sy = std::sin(0.5f*pRot3D.wy);
      const float sz = std::sin(0.5f*pRot3D.wz);

      pQuaternion.w = cz * cy * cx + sz * sy * sx;
      pQuaternion.x = cz * cy * sx - sz * sy * cx;
      pQuaternion.y = cz * sy * cx + cy * sz * sx;
      pQuaternion.z = cy * sz * cx - cz * sy * sx;
    }

    Quaternion quaternionFromRotation3D(
        const Rotation3D& pRot3D)
    {
      Quaternion lQuat;
      quaternionFromRotation3D(pRot3D, lQuat);
      return lQuat;
    }

    void rotationFromQuaternion(
        const Quaternion& pQua,
        Rotation& pRot)
    {
      pRot.r1_c1 = 1.0f - 2.0f*(std::pow(pQua.y, 2) + std::pow(pQua.z, 2));
      pRot.r1_c2 = 2.0f*(pQua.x*pQua.y - pQua.z*pQua.w);
      pRot.r1_c3 = 2.0f*(pQua.x*pQua.z + pQua.y*pQua.w);

      pRot.r2_c1 = 2.0f*(pQua.x*pQua.y + pQua.z*pQua.w);
      pRot.r2_c2 = 1.0f - 2.0f*(std::pow(pQua.x, 2) + std::pow(pQua.z, 2));
      pRot.r2_c3 = 2.0f*(pQua.y*pQua.z - pQua.x*pQua.w);

      pRot.r3_c1 = 2.0f*(pQua.x*pQua.z - pQua.y*pQua.w);
      pRot.r3_c2 = 2.0f*(pQua.y*pQua.z + pQua.x*pQua.w);
      pRot.r3_c3 = 1.0f - 2.0f*(std::pow(pQua.x, 2) + std::pow(pQua.y, 2));
    }

    Rotation rotationFromQuaternion(
        const Quaternion& pQua)
    {
      Rotation lRot;
      rotationFromQuaternion(pQua, lRot);
      return lRot;
    }


    void rotation3DFromQuaternion(
        const AL::Math::Quaternion& pQuaternion,
        AL::Math::Rotation3D& pRot3D)
    {
      //rotationFromQuaternion
      AL::Math::Rotation lRot;
      rotationFromQuaternion(pQuaternion, lRot);

      pRot3D.wz = std::atan2(lRot.r2_c1, lRot.r1_c1);
      const float sy = std::sin(pRot3D.wz);
      const float cy = std::cos(pRot3D.wz);
      pRot3D.wy = std::atan2(-lRot.r3_c1, cy*lRot.r1_c1+sy*lRot.r2_c1);
      pRot3D.wx = std::atan2(sy*lRot.r1_c3-cy*lRot.r2_c3, cy*lRot.r2_c2-sy*lRot.r1_c2);
    }

    Rotation3D rotation3DFromQuaternion(
        const Quaternion& pQuaternion)
    {
      Rotation3D lRotation3D;
      rotation3DFromQuaternion(pQuaternion, lRotation3D);
      return lRotation3D;
    }

    void quaternionPosition3DFromPosition6D(
        const Position6D& pPos6D,
        Quaternion& pQua,
        Position3D& pPos3D)
    {
      quaternionFromRotation3D(
            AL::Math::Rotation3D(pPos6D.wx, pPos6D.wy, pPos6D.wz),
            pQua);

      pPos3D.x = pPos6D.x;
      pPos3D.y = pPos6D.y;
      pPos3D.z = pPos6D.z;
    }


    void pointMassRotationalInertia(
        float pMass,
        const Position3D& pPos,
        std::vector<float>& pInertia)
    {
      const float x2 = boost::math::pow<2>(pPos.x);
      const float y2 = boost::math::pow<2>(pPos.y);
      const float z2 = boost::math::pow<2>(pPos.z);
      pInertia.resize(9);
      pInertia[0] = pMass * (y2 + z2);
      pInertia[4] = pMass * (x2 + z2);
      pInertia[8] = pMass * (x2 + y2);
      pInertia[1] = pInertia[3] = - pMass * pPos.x * pPos.y;
      pInertia[2] = pInertia[6] = - pMass * pPos.x * pPos.z;
      pInertia[5] = pInertia[7] = - pMass * pPos.y * pPos.z;
    }
  } // namespace Math
} // namespace AL

