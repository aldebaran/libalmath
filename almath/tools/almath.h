/**
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved
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

    /**
    * if you want to avoid too small numbers (<5.10^-29 if using float):
    * a = small_one;
    * a += killDeNormalNumber;
    * make_operations(a);
    * a -= killDeNormalNumber;
    * it will save a LOT of cpu (a multiplication with a denormal operand takes about 170 cycles !!!)
    *
    * for your information : denormal number = (FLOAT_MIN*2^32) = 5.0486911077.10^(-29)
    * the killDeNormalNumber const is chosen bigger than this denormal number
    *
    * for more informations, see : www.musicdsp.org/files/denormal.pdf
    */
#define killDeNormalNumber 1e-20f

    struct Complex
    {
      float re;
      float im;

      // USEFULL CONSTRUCTOR
      Complex(
          const float pRe = 0.0f,
          const float pIm = 0.0f):
          re(pRe),
          im(pIm){};
    }; // end Complex


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
    void DiffLog(
      const AL::Math::Velocity6D& pM,
      const AL::Math::Velocity6D& pIn,
      AL::Math::Velocity6D&       pOut);

    /**
    * Function Inv Diff Log : Inverse de Diff Log
    * @param pM   (Velocity6D) : displacement vector 6D
    * @param pIn  (Velocity6D) : multiply 6*6 inverse pM by pIn
    * @param pOut (Velocity6D) : Output solution of inverse(DiffLog(pM))*pIn
    **/
    void InvDiffLog(
      const AL::Math::Velocity6D& pM,
      const AL::Math::Velocity6D& pIn,
      AL::Math::Velocity6D&       pOut);

    /**
    * Returns 1.0f if the value is greater
    * or equal to zero, otherwise returns -1.0f
    */
    float Sign(const float& pValue);

    bool Sign(
      const float& pValue1,
      const float& pValue2);

    /**
    * inlineFactoriel: return the factoriel of the given number
    * @param pNumber the factoriel you want to compute
    **/
    inline unsigned int inlineFactorial(unsigned int pNumber)
    {
      return pNumber> 1 ? (pNumber * inlineFactorial(pNumber-1)) : 1;
    }

    Transform TransformFromPosition3DAndRotation(
      const float x,
      const float y,
      const float z,
      const Rotation& pRotation);


    Transform TransformFromPosition3DAndRotation(
      const Position3D& pPosition,
      const Rotation& pRotation);


    Position6D Position6DFromVelocity6D(const Velocity6D& pIn);

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


    Position2D operator*(
      const Rotation2D& pR,
      const Position2D& pPos);


//    /**
//    * Helper method to compose a position 6D from
//    * a Position3D and a rotation 3D
//    */
//    Position6D POSITION6D(
//      const Position3D& pPos,
//      const Rotation3D& pRot);


    /**
    * Function to compute the gradiant of a line 2D defined by 2 points
    * This function avoid division by zero
    *
    * @param pPointA the first point of the line
    * @param pPointB the second point of the line
    * @return the gradiant of the line 2D
    */
    float computeGradiant(
      const Position2D& pPointA,
      const Position2D& pPointB);

    /**
    * Function to compute the offset of a line
    * from a point of the line and the gradiant
    * @param pPointA the first point of the line
    * @param pGradiant the gradiant of the line
    * @return the offset of the line 2D
    */
    float computeOffset(
      const Position2D& pPointA,
      const float&      pGradiant);

    /**
    * Function to compute the offset of a line 2D defined by 2 points
    *
    * @param pPointA the first point of the line
    * @param pPointB the second point of the line
    * @return the offset of the line 2D
    */
    float computeOffset(
      const Position2D& pPointA,
      const Position2D& pPointB);

    /**
    * Filter a vector of Position6D in order to move only the axis controlled by the axis mask
    * @return          : Vector of Position6D filter
    * @param pXi       : Vector of Position6D of the different stay point of the Interpolation
    * @param pAxisMask : Axis controlled during SE(3) interpolation
    */
    void FilterPosition6D(
      const std::vector<AL::Math::Position6D>& pXi,
      const AL::Math::AXIS_MASK                pAxisMask,
      std::vector<AL::Math::Position6D>&       pOut);

    std::vector<AL::Math::Position6D>  FilterPosition6D(
      const std::vector<AL::Math::Position6D>& pXi,
      const AL::Math::AXIS_MASK                pAxisMask);

    void AxisMaskToPosition6DOn(
      const AL::Math::Position6D& pXi,
      const AL::Math::AXIS_MASK   pAxisMask,
      AL::Math::Position6D&       pOut);

    AL::Math::Position6D AxisMaskToPosition6DOn(
      const AL::Math::Position6D& pXi,
      const AL::Math::AXIS_MASK   pAxisMask);

    void AxisMaskToPosition6DOff(
      const AL::Math::Position6D& pXi,
      const AL::Math::AXIS_MASK   pAxisMask,
      AL::Math::Position6D&       pPos);

    AL::Math::Position6D AxisMaskToPosition6DOff(
      const AL::Math::Position6D& pXi,
      const AL::Math::AXIS_MASK   pAxisMask);

    void AxisMaskToVelocity6DOn(
      const AL::Math::Velocity6D& pXi,
      const AL::Math::AXIS_MASK   pAxisMask,
      AL::Math::Velocity6D&       pOut);

    AL::Math::Velocity6D AxisMaskToVelocity6DOn(
      const AL::Math::Velocity6D& pXi,
      const AL::Math::AXIS_MASK   pAxisMask);

    /**
    * Function RotationFromAngleDirection :
    * Computes the matrix rotation given an angle and a direction
    * It is in almath and not in alrotation so alrotation does not need alposition3D
    * @param pAngle : The angle of the rotation
    * @param pDirection : The direction of the rotation
    * @return The matrix rotation
    **/
    AL::Math::Rotation RotationFromAngleDirection(
      float pAngle,
      const AL::Math::Position3D& pDirection);

    void ApplyRotation(
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
