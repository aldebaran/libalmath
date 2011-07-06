/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALROTATION2D_H_
#define _LIB_ALMATH_ALMATH_ALROTATION2D_H_

#include <vector>

namespace AL {
namespace Math {

/// <summary>
/// A 2*2 rotation matrix
///
/// </summary>
/// <A HREF="http://en.wikipedia.org/wiki/Rotation_matrix">more information</A>
/// \ingroup Types
struct Rotation2D
{
  float r1_c1, r1_c2,
  r2_c1, r2_c2;

  /// <summary>
  /// Create a Rotation initialize to identity.
  /// </summary>
  Rotation2D(): r1_c1(1.0f), r1_c2(0.0f),
    r2_c1(0.0f), r2_c2(1.0f){}

  /// <summary>
  /// Create a Rotation with an std::vector.
  /// </summary>
  /// <param name="pFloats">
  /// An std::vector<float> of size 4:
  ///
  /// \f$ \left[\begin{array}{cccc}r1c1 & r1c2 \\ r2c1 & r2c2 \end{array}\right] = \left[\begin{array}{cccc}pFloats[0] & pFloats[1] & pFloats[2] \\ pFloats[3] & pFloats[4] & pFloats[5] \\ pFloats[6] & pFloats[7] & pFloats[8] \end{array}\right]\f$
  /// </param>
  Rotation2D (const std::vector<float>& pFloats)
  {
    if (pFloats.size() == 4)
    {
      r1_c1 = pFloats[0];
      r1_c2 = pFloats[1];
      r2_c1 = pFloats[2];
      r2_c2 = pFloats[3];
    }
    else
    {
      r1_c1 = 1.0f;
      r1_c2 = 0.0f;
      r2_c1 = 0.0f;
      r2_c2 = 1.0f;
    }
  }

  /// <summary>
  /// overloading of operator *= for Rotation2D.
  /// </summary>
  /// <param name="pRot2"> the second Rotation2D </param>
  Rotation2D& operator*= (const Rotation2D& pRot2);

  /// <summary>
  /// overloading of operator * for Rotation2D.
  /// </summary>
  /// <param name="pRot2"> the second Rotation2D </param>
  Rotation2D operator* (const Rotation2D& pRot2) const;

  /// <summary>
  /// overloading of operator == for Rotation2D.
  /// </summary>
  /// <param name="pRot2"> the second Rotation2D </param>
  bool operator==(const Rotation2D& pRot2) const;

  /// <summary>
  /// overloading of operator != for Rotation2D.
  /// </summary>
  /// <param name="pRot2"> the second Rotation2D </param>
  bool operator!=(const Rotation2D& pRot2) const;

  /// <summary>
  /// Check if the actual Rotation2D is near the one
  /// give in argument.
  ///
  /// </summary>
  /// <param name="pRot2"> the second Rotation2D </param>
  /// <param name="pEpsilon"> an optionnal epsilon distance: default: 0.0001 </param>
  /// <returns>
  /// true if the distance between the two Rotation2D is less than pEpsilon
  /// </returns>
  bool isNear(
    const Rotation2D& pRot2,
    const float&      pEpsilon=0.0001f) const;

  /// <summary>
  /// Compute the rotation transpose (inverse) of the actual Rotation2D.
  /// </summary>
  /// <returns>
  /// the Rotation2D transpose
  /// </returns>
  Rotation2D transpose() const;

  /// <summary>
  /// Compute the determinant of the Rotation2D
  ///
  /// \f$det = pRot.r1_c1*pRot.r2_c2 - pRot.r1_c2*pRot.r2_c1\f$
  /// </summary>
  /// <returns>
  /// the float determinant of the Rotation2D
  /// </returns>
  float determinant() const;

  /// <summary>
  /// Create a Rotation2D initialized with explicit angle.
  ///
  /// \f$ pRot = \left[\begin{array}{cccc}cos(pTheta) & -sin(pTheta) \\ sin(pTheta) & cos(pTheta) \end{array}\right]\f$
  ///
  /// </summary>
  /// <param name="pTheta"> the float value for angle rotation </param>
  static Rotation2D fromAngle(const float pTheta);

  /// <summary>
  /// return the Rotation2D as a vector of float
  ///
  /// \f$ \begin{array}{cccc} [r1c1, & r1c2, \\ r2c1, & r2c2] \end{array}\f$
  /// </summary>
  std::vector<float> toVector() const;
};

/// <summary>
/// Compute the Rotation2D transpose (inverse) of the actual Rotation2D.
/// </summary>
/// <param name="pRot"> the given Rotation2D </param>
/// <returns>
/// the Rotation2D transpose
/// </returns>
/// \ingroup Types
Rotation2D transpose(const Rotation2D& pRot);

/// <summary>
/// Compute the determinant of the Rotation2D
///
/// \f$det = pRot.r1_c1*pRot.r2_c2 - pRot.r1_c2*pRot.r2_c1\f$
/// </summary>
/// <param name="pRot"> the given Rotation2D </param>
/// <returns>
/// the float determinant of the Rotation2D
/// </returns>
/// \ingroup Types
float determinant(const Rotation2D& pRot);

}
}
#endif  // _LIB_ALMATH_ALMATH_ALROTATION2D_H_
