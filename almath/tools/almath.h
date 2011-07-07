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

    /**
    * Returns 1.0f if the value is greater
    * or equal to zero, otherwise returns -1.0f
    */
    float sign(const float& pValue);

    bool sign(
      const float& pValue1,
      const float& pValue2);


    Transform transformFromPosition3DAndRotation(
      const float x,
      const float y,
      const float z,
      const Rotation& pRotation);


    Transform transformFromPosition3DAndRotation(
      const Position3D& pPosition,
      const Rotation& pRotation);


    Position6D position6DFromVelocity6D(const Velocity6D& pIn);

    Position3D operator*(
      const Rotation&   pRot,
      const Position3D& pPos);


    /*
    * Velocity6D = Stiffness (float) * Delta Position (Position6D)
    * @param (float,Position6D)
    * @return Velocity6D
    */
    Velocity6D operator*(
      const float       pK,
      const Position6D& pDelta);



    /**
    * Function rotationFromAngleDirection :
    * Computes the matrix rotation given an angle and a direction
    * It is in almath and not in alrotation so alrotation does not need alposition3D
    * @param pAngle : The angle of the rotation
    * @param pDirection : The direction of the rotation
    * @return The matrix rotation
    **/
    AL::Math::Rotation rotationFromAngleDirection(
      float                       pAngle,
      const AL::Math::Position3D& pDirection);

    void applyRotation(
      const AL::Math::Rotation& pRotation,
      AL::Math::Position3D&     pVector);


    /// <summary> Private Function to create a trapzeoidale. </summary>
    /// <param name="pNumSamples">      number of samples to compute. </param>
    /// <param name="pHeight">          Height of the Trapezoid ans Sign. </param>
    /// <param name="pAttackFraction">  the NumSamples to use for the attack phase. </param>
    /// <param name="pDecayFraction">   the NumSamples to use for the decay phase. </param>
    /// <returns> The Trapezoid Function </returns>
    std::vector<float> smoothTrapezoid(
      const unsigned int& pNumSamples,
      const float&        pHeight,
      const unsigned int& pAttack,
      const unsigned int& pDecay);

  } // namespace Math
} // namespace AL
#endif  // _LIB_ALMATH_ALMATH_ALMATH_H_
