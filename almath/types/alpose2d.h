/**
* @author Chris Kilner
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALPOSE2D_H_
#define _LIB_ALMATH_ALMATH_ALPOSE2D_H_

#include <vector>

namespace AL {
  namespace Math {

    struct Pose2D {
      float x;
      float y;
      float theta;

      Pose2D():x(0.0f), y(0.0f), theta(0.0f) {}
      explicit Pose2D(float pInit):x(pInit), y(pInit), theta(pInit) {}

      Pose2D(
          float pX,
          float pY,
          float pTheta):x(pX), y(pY), theta(pTheta) {}

      /**
      * CONSTRUCTOR: create a Pose2D from an std vector.
      */
      Pose2D (const std::vector<float>& pFloats)
      {
        if (pFloats.size() == 3)
        {
          x = pFloats[0];
          y = pFloats[1];
          theta = pFloats[2];
        }
        else
        {
          x = 0.0f;
          y = 0.0f;
          theta = 0.0f;
        }
      }

      Pose2D operator+ (const Pose2D& pPos2) const;
      Pose2D operator- (const Pose2D& pPos2) const;

      Pose2D& operator+= (const Pose2D& pPos2);
      Pose2D& operator-= (const Pose2D& pPos2);

      Pose2D& operator*= (const Pose2D& pPos2);
      Pose2D operator* (const Pose2D& pPos2) const;

      bool operator==(const Pose2D& pPos2) const;
      bool operator!=(const Pose2D& pPos2) const;

      float distanceSquared(const Pose2D& pPos) const;
      float distance(const Pose2D& pPos) const;

      bool isNear(
          const Pose2D& pPos,
          const float&  pEpsilon=0.0001f) const;

      std::vector<float> toVector() const;

    }; // end struct

    float distanceSquared(
        const Pose2D& pPos1,
        const Pose2D& pPos2);

    float distance(
        const Pose2D& pPos1,
        const Pose2D& pPos2);

  } // end namespace math
} // end namespace AL
#endif  // _LIB_ALMATH_ALMATH_ALPOSE2D_H_
