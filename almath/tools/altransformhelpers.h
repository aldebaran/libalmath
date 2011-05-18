/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALTRANSFORMHELPERS_H_
#define _LIB_ALMATH_ALMATH_ALTRANSFORMHELPERS_H_

#include <almath/types/alposition3d.h>
#include <almath/types/alposition6d.h>
#include <almath/types/alrotation.h>
#include <almath/types/alrotation2d.h>
#include <almath/types/alrotation3d.h>
#include <almath/types/alvelocity6d.h>
#include <almath/types/altransform.h>
#include <almath/types/alaxismask.h>
#include <almath/types/alpose2d.h>

namespace AL {
  namespace Math {
    /**
    * Function Transform Logarithme : cette fonction calcule le
    * logarithme associe a une matrice de type Deplacement - matrice homogene 4x4 (SE3)
    * La sortie est un torseur cinematique de se3.
    * Le resultat n'est garanti que pour des angles dans [-pi+0.001,pi-0.001].
    * cette fonction calcule la differentielle du logarithme associe
    * a une matrice de type Deplacement - matrice homogene 4x4 (SE3).
    * @param Transform Homogenous Matrix (4*4)
    * @return Velocity6D (torseur cinematique)
    **/
    AL::Math::Velocity6D TransformLogarithme(const AL::Math::Transform& pH);

    void TransformLogarithme(
      const AL::Math::Transform& pH,
      AL::Math::Velocity6D&      pV);

    /**
    * Function Velocity Exponential : compute homogenous matrix
    * diplacement from a dt * 6D velocity vector
    * @param Velocity6D  : dt*velocity
    * @param Transform
    **/
    void VelocityExponential(
      const AL::Math::Velocity6D& pM,
      AL::Math::Transform&        tM);

    AL::Math::Transform VelocityExponential(const AL::Math::Velocity6D& pM);

    void ChangeRepereVelocity6D(
      const AL::Math::Transform&  pH,
      const AL::Math::Velocity6D& pVIn,
      AL::Math::Velocity6D&       pVOut);

    void ChangeReperePosition6D(
      const AL::Math::Transform&  pH,
      const AL::Math::Position6D& pPIn,
      AL::Math::Position6D&       pPOut);

    void ChangeReperePosition3D(
      const AL::Math::Transform& pH,
      AL::Math::Position3D&      pPosOut);

    void ChangeRepereTransposePosition3D(
      const AL::Math::Transform& pH,
      AL::Math::Position3D&      pPosOut);

    void ChangeReperePosition3D(
      const AL::Math::Transform&  pH,
      const AL::Math::Position3D& pPosIn,
      AL::Math::Position3D&       pPosOut);

    void ChangeRepereTransposePosition3D(
      const AL::Math::Transform&  pH,
      const AL::Math::Position3D& pPosIn,
      AL::Math::Position3D&       pPosOut);

    void ChangeRepereTransform(
      const AL::Math::Transform& pH,
      const AL::Math::Transform& pHIn,
      AL::Math::Transform&       pHOut);

    void ChangeRepereTransposeTransform(
      const AL::Math::Transform& pH,
      const AL::Math::Transform& pHIn,
      AL::Math::Transform&       pHOut);

    void ChangeRepereTransposeVelocity6D(
      const AL::Math::Transform&  pH,
      const AL::Math::Velocity6D& pVIn,
      AL::Math::Velocity6D&       pVOut);

    void ChangeRepereTransposePosition6D(
      const AL::Math::Transform&  pH,
      const AL::Math::Position6D& pPIn,
      AL::Math::Position6D&       pPOut);

    void TransformMean(
      const AL::Math::Transform& pHIn1,
      const AL::Math::Transform& pHIn2,
      const float&               pDist,
      AL::Math::Transform&       pHOut);

    AL::Math::Transform TransformMean(
      const AL::Math::Transform& pHIn1,
      const AL::Math::Transform& pHIn2);

    AL::Math::Transform TransformMean(
      const AL::Math::Transform& pHIn1,
      const AL::Math::Transform& pHIn2,
      const float&               pDist);


    /**
    * Creates a 4*4 transform matrix from XYZ coordinates
    * indicating the translational offsets.
    */

    void TransformFromPosition3D(
      const Position3D& pPosition,
      Transform&        pTransform);


    Transform TransformFromPosition3D(const Position3D& pPosition);

    void RotationToTransform(
      const Rotation& pRotation,
      Transform&      pTransform);

    Transform RotationToTransform(const Rotation& pRotation);

    void RotationFromTransform(
      const Transform& pTransform,
      Rotation&        pRotation);

    Rotation RotationFromTransform(const Transform& pTransform);

    Rotation3D Rotation3DFromRotation(const Rotation& pRotation1);

    /**
    * Extracts the position coordinates from a Matrix
    * into a Position3D struct
    * @param pTransform The transform form which you want to extract the coordinates.
    */
    void TransformToPosition3D(
      const Transform& pTransform,
      Position3D&      pPosition);

    Position3D TransformToPosition3D(const Transform& pTransform);

    /**
    * Return Position6D corresponding to 4*4 Homogenous Matrix pT
    * @param pT 4*4 Homogenous Matrix pT
    */
    void Position6DFromTransform(
        const Transform& pT,
        Position6D&      pPos);

    Position6D Position6DFromTransform(const Transform& pT);

    /**
    * Creates a 4*4 transform matrix from Pose2D
    */
    void TransformFromPose2D(
        const Pose2D& pPose,
        Transform&    pT);

    Transform TransformFromPose2D(const Pose2D& pPose);

    /**
    * Return Pose2D corresponding to the 4*4 Homogenous Matrix pT
    * @param pT 4*4 Homogenous Matrix pT
    */
    void Pose2DFromTransform(
        const Transform& pT,
        Pose2D&          pPos);

    Pose2D Pose2DFromTransform(const Transform& pT);

    /**
    * Creates a 4*4 Homogenous Matrix
    * from a Roll, Pitch and yaw Angle in radian
    * @param pRotation Rotation structure
    */
    Transform TransformFromRotation3D(const Rotation3D& pRotation);

    /**
    * Creates a 4*4 transform matrix from a rotation(RPY) and a position
    * indicating the translational offsets.
    */
    Transform TransformFromPosition6D(const Position6D& pPosition6D);

    /**
    * Return a 6 differential motion require to move
    * from a 4*4 Homogenous transform matrix Current to
    * a 4*4 Homogenous transform matrix target
    * @param  pCurrent 4*4 Homogenous transform matrix
    * @param  pTarget  4*4 Homogenous transform matrix
    * @param  result   6*1 Position6D
    */
    void TransformDiffToPosition(
      const Transform& pCurrent,
      const Transform& pTarget,
      Position6D&      result);

    /**
    * Return a 6 differential motion require to move
    * from a 4*4 Homogenous transform matrix Current to
    * a 4*4 Homogenous transform matrix target
    * @param  pCurrent 4*4 Homogenous transform matrix
    * @param  pTarget  4*4 Homogenous transform matrix
    * @return  pDelta  6*1 Position6D
    */
    Position6D TransformDiffToPosition(
      const Transform& pCurrent,
      const Transform& pTarget);

    void Position3DFromTransformInPlace(
      const Transform& pH,
      Position3D&      pOut);

    Position3D Position3DFromTransform(const Transform& pH);

    Transform RotationPosition3DToTransform(
      const Rotation&   pRot,
      const Position3D& pPos);

    /**
    * Return Rotation3D (Roll, Pitch, Yaw) corresponding to the rotational
    * part of the 4*4 Homogenous Matrix pT
    * @param pT 4*4 Homogenous Matrix pT
    */
    Rotation3D Rotation3DFromTransform(const Transform& pT);

    Position3D operator*(
      const Transform&  pT,
      const Position3D& pPos);

    Rotation2D Rotation2DFromTransformZ(const Transform& pH);

    Transform& operator+=(
      Transform&        pT,
      const Position3D& pPos);

//    /**
//    * Helper method to compose a position 6D from
//    * a Position3D and a Homogenous Transform
//    */
//    Position6D POSITION6D(
//      const Position3D& pPos,
//      const Transform&  pHRot);


    /**
    * Function AxisRotationProjection :
    * finding the closest rotation Rw of R around an axis (Position3D)
    * @param Transform : useful only for Rotation part
    * @param Position3D : axis of rotation
    * @return Transform
    **/
    void AxisRotationProjection(
      const Position3D& pAxis,
      Transform&        pH);

    Transform AxisRotationProjection(
      const Transform&  pH,
      const Position3D& pAxis);

    std::vector<AL::Math::Transform> FilterTransform(
      const std::vector<AL::Math::Transform>& pHi,
      const AL::Math::AXIS_MASK               pAxisMask);

    AL::Math::Transform AxisMaskToTransformOn(
      const AL::Math::Transform& pHi,
      const AL::Math::AXIS_MASK  pAxisMask);

    void computeMixTransformWithAxisMask(
      const Transform& pDesired,
      const Transform& pReference,
      const AXIS_MASK& pAxisMask,
      Transform&       pResult);

    // Main
    void RotVecToTransform(
      const int                   pAxis,
      const float                 pTheta,
      const AL::Math::Position3D& pM,
      Transform&                  pTransform);

    AL::Math::Transform RotVecToTransform(
      const int                   pAxis,
      const float                 pTheta,
      const AL::Math::Position3D& pM);

    /**
    @param rotation axis, angle in degree, translation 3D Vector
    @return Transform
    **/
    void RotVecToTransform(
      const AL::Math::Position3D& pPosition,
      AL::Math::Transform&        pTransform);

    AL::Math::Transform RotVecToTransform(const AL::Math::Position3D& pPosition);

    AL::Math::Transform RotVecToTransform(
      const int&   pAxis,
      const float& pRot);


    /**
    * Function AxisRotationProjection :
    * finding the closest rotation Rw of R around an axis (Position3D)
    * @param Transform : useful only for Rotation part
    * @param Position3D : axis of rotation
    * @return Transform
    **/
    Rotation AxisRotationProjection(
      const Rotation&   pRot,
      const Position3D& pAxis);

    /**
    * Function AxisRotationProjectionInPlace :
    * finding the closest rotation Rw of R around an axis (Position3D)
    * @param Position3D : axis of rotation
    * @param pH: a transform
    **/
    void AxisRotationProjectionInPlace(
      const Position3D& pAxis,
      Transform&        pH);

    /**
    * Function AxisRotationProjectionInPlace :
    * finding the closest rotation Rw of R around an axis (Position3D)
    * @param pRot : a rotation
    * @param Position3D : axis of rotation
    **/
    void AxisRotationProjectionInPlace(
      Rotation&         pRot,
      const Position3D& pAxis);

    // ORTHSPACE: Plan orthogonal d'une droite vectorielle
    //
    // Retourne une base orthonormee de l espace orthogonal au vecteur e3, choisie de
    // telle sorte que base_espace = [e1 e2 e3] soit une base orthogonale directe de
    // l espace R3 tout entier (et meme, une base orthonormee directe, si e3 est
    // norme).
    //Transform Orthospace(const Position3D& pAxis, unsigned int& idx);
    void Orthospace(const Position3D& pAxis, Transform& HOut);
    Transform Orthospace(const Position3D& pAxis);

    // OK
    void AxisRotationToTransform(
        const Position3D& pAxisRotation,
        const float&      Ca,
        const float&      Sa,
        Transform&        pHOut);

    void DiffAxisToAntiSynmetric(
        const Position3D& pA,
        const Position3D& pB,
        Transform&        HOut);

    bool FindRotationBest(
        const Position3D& pAxisInit,
        const Position3D& pAxisFinal,
        Transform&        pHOut);

    void FindRotation(
        const Position3D& pAxisInit,
        const Position3D& pAxisFinal,
        Transform&        pHOut);

  } // namespace Math
} // namespace AL
#endif  // _LIB_ALMATH_ALMATH_ALTRANSFORMHELPERS_H_
