/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/types/alquaternion.h>
#include <cmath>
#include <stdexcept>
#include <almath/tools/altrigonometry.h>
#include <iostream>

namespace AL {
  namespace Math {

    Quaternion::Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}

    Quaternion::Quaternion(
      float pW,
      float pX,
      float pY,
      float pZ)
      :w(pW)
      ,x(pX)
      ,y(pY)
      ,z(pZ){}

    Quaternion::Quaternion(const std::vector<float>& pFloats)
    {
      if (pFloats.size() == 4u)
      {
        w = pFloats[0];
        x = pFloats[1];
        y = pFloats[2];
        z = pFloats[3];
      }
      else
      {
        std::cerr << "ALMath: WARNING: "
                  << "Quaternion constructor call with a wrong size of vector. "
                  << "Size expected: 4. Size given: " << pFloats.size() << ". "
                  << "Quaternion is set to default value." << std::endl;

        w = 0.0f;
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
      }
    }

    Quaternion& Quaternion::operator*= (const Quaternion& pQua2)
    {
      const float w1 = w;
      const float x1 = x;
      const float y1 = y;
      const float z1 = z;

      if (this == &pQua2)
      {
        // copy to manage case: a *= a
        return *this *= Quaternion(pQua2);
      }

      w = w1*pQua2.w - x1*pQua2.x - y1*pQua2.y - z1*pQua2.z;
      x = w1*pQua2.x + pQua2.w*x1 + y1*pQua2.z - z1*pQua2.y;
      y = w1*pQua2.y + pQua2.w*y1 + z1*pQua2.x - x1*pQua2.z;
      z = w1*pQua2.z + pQua2.w*z1 + x1*pQua2.y - y1*pQua2.x;
      return *this;
    }

    bool Quaternion::isNear(
        const Quaternion& pQua2,
        const float&      pEpsilon) const
    {
      return (// |pQua1 - pQua2| < epsilon
              (std::abs(w - pQua2.w) < pEpsilon &&
               std::abs(x - pQua2.x) < pEpsilon &&
               std::abs(y - pQua2.y) < pEpsilon &&
               std::abs(z - pQua2.z) < pEpsilon) ||
              (// |pQua1 + pQua2| < epsilon
               std::abs(w + pQua2.w) < pEpsilon &&
               std::abs(x + pQua2.x) < pEpsilon &&
               std::abs(y + pQua2.y) < pEpsilon &&
               std::abs(z + pQua2.z) < pEpsilon));
    }


    Quaternion& Quaternion::operator*= (float pVal)
    {
      w *= pVal;
      x *= pVal;
      y *= pVal;
      z *= pVal;
      return *this;
    }


    Quaternion& Quaternion::operator/= (float pVal)
    {
      if (pVal == 0.0f)
      {
        throw std::runtime_error(
          "ALQuaternion: operator/= Division by zero.");
      }
      *this *= (1.0f/pVal);
      return *this;
    }

    Quaternion Quaternion::operator/ (float pVal) const
    {
      if (pVal == 0.0f)
      {
        throw std::runtime_error(
          "ALQuaternion: operator/ Division by zero.");
      }
      return *this * (1.0f/pVal);
    }

    bool Quaternion::operator== (const Quaternion& pQua2) const
    {
      return (w == pQua2.w &&
              x == pQua2.x &&
              y == pQua2.y &&
              z == pQua2.z);
    }

    bool Quaternion::operator!= (const Quaternion& pQua2) const
    {
      return !(*this==pQua2);
    }


    Quaternion quaternionFromAngleAndAxisRotation(
      const float pAngle,
      const float pAxisX,
      const float pAxisY,
      const float pAxisZ)
    {
      Quaternion qua = Quaternion();

      const float sin_a = std::sin(0.5f*pAngle);
      const float cos_a = std::cos(0.5f*pAngle);

      qua.w = cos_a;
      qua.x = pAxisX*sin_a;
      qua.y = pAxisY*sin_a;
      qua.z = pAxisZ*sin_a;

      qua.normalize();
      return qua;
    }


    Quaternion Quaternion::fromAngleAndAxisRotation(
      const float pAngle,
      const float pAxisX,
      const float pAxisY,
      const float pAxisZ)
    {
      return Math::quaternionFromAngleAndAxisRotation(
            pAngle,
            pAxisX,
            pAxisY,
            pAxisZ);
    }


    void Quaternion::toVector(std::vector<float>& pReturnVector) const
    {
      pReturnVector.resize(4);
      pReturnVector[0] = w;
      pReturnVector[1] = x;
      pReturnVector[2] = y;
      pReturnVector[3] = z;
    }

    std::vector<float> Quaternion::toVector(void) const
    {
      std::vector<float> returnVector(4, 0.0f);
      this->toVector(returnVector);
      return returnVector;
    }

    float Quaternion::norm() const
    {
      return Math::norm(*this);
    }

    Quaternion Quaternion::normalize() const
    {
      return Math::normalize(*this);
    }

    float norm(const Quaternion& p)
    {
      return std::sqrt(std::pow(p.w, 2) +
                       std::pow(p.x, 2) +
                       std::pow(p.y, 2) +
                       std::pow(p.z, 2));
    }

    Quaternion normalize(const Quaternion& pQua)
    {
      const float tmpNorm = norm(pQua);
      if (tmpNorm == 0.0f)
      {
        throw std::runtime_error(
          "ALQuaternion: normalize Division by zero.");
      }
      return pQua/tmpNorm;
    }

    Quaternion Quaternion::inverse() const
    {
      return Math::quaternionInverse(*this);
    }


    void quaternionInverse(
      const Quaternion& pQua,
      Quaternion&       pQuaOut)
    {
      pQuaOut.w = pQua.w;
      pQuaOut.x = -pQua.x;
      pQuaOut.y = -pQua.y;
      pQuaOut.z = -pQua.z;
    }


    Quaternion quaternionInverse(const Quaternion& pQua)
    {
      Quaternion pQuaOut;
      quaternionInverse(pQua, pQuaOut);
      return pQuaOut;
    }


    void angleAndAxisRotationFromQuaternion(
      const Quaternion& pQuaternion,
      float& pAngle,
      float& pAxisX,
      float& pAxisY,
      float& pAxisZ)
    {
      Quaternion copy = pQuaternion;
      copy.normalize();
      pAngle          = 2.0f*std::acos(copy.w); // * AL::Math::_2_PI_;
      float sin_angle = std::sqrt(1.0f - std::pow(copy.w, 2));

      if (std::abs(sin_angle) < 0.0005f)
      {
        sin_angle = 1.0f;
      }

      pAxisX = copy.x / sin_angle;
      pAxisY = copy.y / sin_angle;
      pAxisZ = copy.z / sin_angle;

      const float normAxe =
          std::sqrt(std::pow(pAxisX, 2) +
                    std::pow(pAxisY, 2) +
                    std::pow(pAxisZ, 2));

      if (normAxe < 0.0001f)
      {
        pAxisX = 1.0f;
        pAxisY = 0.0f;
        pAxisZ = 0.0f;
      }
    }

    std::vector<float> angleAndAxisRotationFromQuaternion(
      const Quaternion& pQuaternion)
    {
      std::vector<float> result(4, 0.0f);

      angleAndAxisRotationFromQuaternion(
            pQuaternion,
            result[0],
            result[1],
            result[2],
            result[3]);

      return result;
    }

  } // end namespace math
} // end namespace al

