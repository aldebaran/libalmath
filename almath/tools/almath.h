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

#include <almath/types/alposition2d.h>
#include <almath/types/alvelocity3d.h>
#include <almath/tools/altrigonometry.h>
#include <almath/types/alpositionandvelocity.h>
#include <almath/types/altransformandvelocity6d.h>
#include <almath/tools/altransformhelpers.h>

namespace AL {
  namespace Math {

    bool clipData(
      const float& pMin,
      const float& pMax,
      float&       pData);

    /**
    * Function Diff Log : cette fonction calcule la differentielle du
    * logarithme associe a une matrice de type 'Deplacement - matrice homogene' 4x4 (SE3).
    * @param pM   (Velocity6D) : displacement vector 6D
    * @param pIn  (Velocity6D) : multiply 6*6 pM by pIn
    * @param pOut (Velocity6D) : Output solution of DiffLog(pM)*pIn
    **/
    void diffLog(
      const AL::Math::Velocity6D& pM,
      const AL::Math::Velocity6D& pIn,
      AL::Math::Velocity6D&       pOut);

    /**
    * Function Inv Diff Log : Inverse de Diff Log
    * @param pM   (Velocity6D) : displacement vector 6D
    * @param pIn  (Velocity6D) : multiply 6*6 inverse pM by pIn
    * @param pOut (Velocity6D) : Output solution of inverse(DiffLog(pM))*pIn
    **/
    void invDiffLog(
      const AL::Math::Velocity6D& pM,
      const AL::Math::Velocity6D& pIn,
      AL::Math::Velocity6D&       pOut);


    Transform transformFromPosition3DAndRotation(
      const float x,
      const float y,
      const float z,
      const Rotation& pRotation);


    Transform transformFromPosition3DAndRotation(
      const Position3D& pPosition,
      const Rotation& pRotation);


    Position6D position6DFromVelocity6D(const Velocity6D& pIn);


    /// <summary>
    /// Overloading of operator * between Rotation and Position3D.
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
    /// Overloading of operator * for float to Position6D, give a Velocity6D.
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
      const float&                pTheta,
      const AL::Math::Position3D& pPos);


    /// <summary>
    /// Apply Rotation to a 3D point.
    /// </summary>
    /// <param name="pRot"> the given rotation </param>
    /// <param name="pPos"> the 3D point rotated </param>
    /// \ingroup Tools
    void applyRotation(
      const AL::Math::Rotation& pRot,
      AL::Math::Position3D&     pPos);


  } // namespace Math
} // namespace AL
#endif  // _LIB_ALMATH_ALMATH_ALMATH_H_
