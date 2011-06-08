/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALINTERPOLATIONTYPES_H_
#define _LIB_ALMATH_ALMATH_ALINTERPOLATIONTYPES_H_

#include <almath/types/alposition2d.h>

#include <vector>
#include <iostream>

namespace AL
{
  namespace Math
  {
    namespace Interpolation
    {

      enum InterpolationType
      {
        CONSTANT,
        LINEAR,
        BEZIER,
        BEZIER_AUTO
      };

      struct Tangent
      {
        InterpolationType fType;
        AL::Math::Position2D fOffset;

        // Position2D default constructor is fine,
        // and default to bezier auto like in choregraphe if no tangent is provided
        Tangent() :
        fType(BEZIER_AUTO),
          fOffset(AL::Math::Position2D(0.0f, 0.0f)){ }

        Tangent(
          InterpolationType pType,
          const AL::Math::Position2D& pOffset) :
        fType(pType),
          fOffset(pOffset) { }

      };

      inline std::ostream& operator<< (std::ostream& pStream, const Tangent& p)
      {
        pStream << "[" << p.fType << ", " << p.fOffset.x << ", " << p.fOffset.y << "]";
        return pStream;
      }

      struct Key
      {
        float fValue;
        Tangent fLeftTangent;
        Tangent fRightTangent;

        // default constructor
        // Tangent() default constructor if fine
        Key():
        fValue(0.0),
          fLeftTangent(Tangent()),
          fRightTangent(Tangent()) {}

        // constructor with one value and two tangents
        Key(
          const float& pValue,
          const Tangent& pLeftTangent,
          const Tangent& pRightTangent):
        fValue(pValue),
          fLeftTangent(pLeftTangent),
          fRightTangent(pRightTangent) {}

      };

      inline std::ostream& operator<< (std::ostream& pStream, const Key& p)
      {
        pStream << "[" << p.fValue << ", [" << p.fLeftTangent << ", " << p.fRightTangent << "] ]";
        return pStream;
      }

    }
  }
}

#endif  // _LIB_ALMATH_ALMATH_ALINTERPOLATIONTYPES_H_
