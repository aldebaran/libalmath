/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALMATH_H_
#define _LIB_ALMATH_ALMATH_ALMATH_H_

#include <almath/types/alvelocity6d.h>
#include <almath/types/alposition3d.h>
#include <almath/types/alposition6d.h>
#include <almath/types/alrotation.h>
#include <almath/types/altransform.h>

namespace AL {
  namespace Math {

  /// <summary>
  /// Clip an input data inside min and max limit.
  ///
  /// \f$ pMin \leq pData \leq pMax \f$
  ///
  /// </summary>
  /// <param name="pMin"> the min limit </param>
  /// <param name="pMax"> the max limit </param>
  /// <param name="pData"> the clipped data </param>
  /// <returns>
  /// Return true if the input pData is clipped.
  /// </returns>
  /// \ingroup Tools
  bool clipData(
    const float& pMin,
    const float& pMax,
    float&       pData);


    /// <summary>
    /// Create a Transform from a 3D points and a Rotation.
    ///
    /// \f$ T = \left[\begin{array}{cccc} pRot.r1c1 & pRot.r1c2 & pRot.r1c3 & pX \\ pRot.r2c1 & pRot.r2c2 & pRot.r2c3 & pY \\ pRot.r3c1 & pRot.r3c2 & pRot.r3c3 & pZ \\ 0.0 & 0.0 & 0.0 & 1.0 \end{array}\right] \f$
    ///
    /// </summary>
    /// <param name="pX"> the translation along x axis </param>
    /// <param name="pY"> the translation along y axis </param>
    /// <param name="pZ"> the translation along z axis </param>
    /// <param name="pRot"> the given Rotation </param>
    /// <returns>
    /// the Transform result.
    /// </returns>
    /// \ingroup Tools
    Transform transformFromPosition3DAndRotation(
      const float&    pX,
      const float&    pY,
      const float&    pZ,
      const Rotation& pRot);


    /// <summary>
    /// Create a Transform from a Position3D and a Rotation.
    ///
    /// \f$ T = \left[\begin{array}{cccc} pRot.r1c1 & pRot.r1c2 & pRot.r1c3 & pPos.x \\ pRot.r2c1 & pRot.r2c2 & pRot.r2c3 & pPos.y \\ pRot.r3c1 & pRot.r3c2 & pRot.r3c3 & pPos.z \\ 0.0 & 0.0 & 0.0 & 1.0 \end{array}\right] \f$
    ///
    /// </summary>
    /// <param name="pPos"> the given Position3D </param>
    /// <param name="pRot"> the given Rotation </param>
    /// <returns>
    /// the Transform result.
    /// </returns>
    /// \ingroup Tools
    Transform transformFromPosition3DAndRotation(
      const Position3D& pPos,
      const Rotation&   pRot);


    /// <summary>
    /// Create a Position6D from a Velocity6D
    ///
    /// \f$\begin{array}{ccc} result.x & = & pVel.xd \\ result.y & = & pVel.yd \\ result.z & = & pVel.zd \\ result.wx & = & pVel.wxd \\ result.wy & = & pVel.wyd \\ result.wz & = & pVel.wzd \end{array} \f$
    ///
    /// </summary>
    /// <param name="pVel"> the given Velocity6D </param>
    /// <returns>
    /// the Position6D result.
    /// </returns>
    /// \ingroup Tools
    Position6D position6DFromVelocity6D(const Velocity6D& pVel);


    /// <summary>
    /// Overloading of operator * between Rotation and Position3D:
    ///
    /// \f$\left[\begin{array}{c} result.x \\ result.y \\ result.z \end{array}\right] = \left[\begin{array}{ccc} pRot.r1c1 & pRot.r1c2 & pRot.r1c3 \\ pRot.r2c1 & pRot.r2c2 & pRot.r2c3 \\ pRot.r3c1 & pRot.r3c2 & pRot.r3c3 \end{array}\right] * \left[\begin{array}{c} pPos.x \\ pPos.y \\ pPos.z \end{array}\right] \f$
    ///
    /// </summary>
    /// <param name="pRot"> the given Rotation </param>
    /// <param name="pPos"> the given Position3D </param>
    /// <returns>
    /// the Position3D result.
    /// </returns>
    /// \ingroup Tools
    Position3D operator*(
      const Rotation&   pRot,
      const Position3D& pPos);


    /// <summary>
    /// Overloading of operator * for float to Position6D, give a Velocity6D:
    ///
    /// \f$\begin{array}{ccc} pVel.xd & = & pVal*pPos.x \\ pVel.yd & = & pVal*pPos.y \\ pVel.zd & = & pVal*pPos.z \\ pVel.wxd & = & pVal*pPos.wx \\ pVel.wyd & = & pVal*pPos.wy \\ pVel.wzd & = & pVal*pPos.wz \end{array} \f$
    ///
    /// </summary>
    /// <param name="pVal"> the given float </param>
    /// <param name="pPos"> the given Position6D </param>
    /// <returns>
    /// the Velocity6D
    /// </returns>
    /// \ingroup Tools
    Velocity6D operator*(
      const float       pVal,
      const Position6D& pPos);


    /// <summary>
    /// Creates a 3*3 Rotation Matrix from a an angle and a normalized Position3D.
    /// </summary>
    /// <param name="pTheta"> the float value of angle in radian </param>
    /// <param name="pPos"> the Position3D direction of the vector of the rotation, normalized </param>
    /// <returns>
    /// the Rotation matrix
    /// </returns>
    /// \ingroup Tools
    AL::Math::Rotation rotationFromAngleDirection(
      const float&      pTheta,
      const Position3D& pPos);


    /// <summary>
    /// Apply Rotation to a 3D point.
    /// </summary>
    /// <param name="pRot"> the given rotation </param>
    /// <param name="pPos"> the 3D point rotated </param>
    /// \ingroup Tools
    void applyRotation(
      const Rotation& pRot,
      Position3D&     pPos);


  } // namespace Math
} // namespace AL
#endif  // _LIB_ALMATH_ALMATH_ALMATH_H_
