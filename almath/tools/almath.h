/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */


#pragma once
#ifndef _LIBALMATH_ALMATH_TOOLS_ALMATH_H_
#define _LIBALMATH_ALMATH_TOOLS_ALMATH_H_

#include <almath/types/alvelocity3d.h>
#include <almath/types/alvelocity6d.h>
#include <almath/types/alposition2d.h>
#include <almath/types/alposition3d.h>
#include <almath/types/alposition6d.h>
#include <almath/types/alrotation.h>
#include <almath/types/alrotation3d.h>
#include <almath/types/alquaternion.h>
#include <almath/types/alpose2d.h>

namespace AL {
  namespace Math {

  /// <summary>
  /// Set an angle between ]-PI, PI].
  ///
  /// </summary>
  /// <param name="pAngle"> the input/output angle </param>
  /// \ingroup Tools
  void modulo2PIInPlace(float& pAngle);

  /// <summary>
  /// Return an angle between ]-PI, PI].
  ///
  /// </summary>
  /// <param name="pAngle"> the input angle </param>
  /// <returns>
  /// Return an angle between ]-PI, PI].
  /// </returns>
  /// \ingroup Tools
  float modulo2PI(float pAngle);

  /// <summary>
  /// Returns the mean of given angles (between ]-PI, PI]). It is defined by the
  /// direction of the mean of all unitary vectors corresponding to the given
  /// angles. If that mean vector is a null vector, then the angular mean is not
  /// defined and the method will throw. For example, meanAngle([0, PI]) throws.
  /// </summary>
  /// <param name="pAngles"> the input/output angles </param>
  /// <returns>
  /// The computed mean (in ]-PI, PI]).
  /// </returns>
  /// \ingroup Tools
  float meanAngle(const std::vector<float>& pAngles);

  /// <summary>
  /// Returns the weighted mean of given angles (between ]-PI, PI]). It is
  /// defined by the direction of the mean of all unitary vectors corresponding
  /// to the given angles, multiplied by the given weights. If that mean vector
  /// is a null vector, then the angular mean is not defined and the method will
  /// throw. For example, meanAngle([0, PI], [1.0, 1.0]) throws.
  ///
  /// All weights must be strictly positive, else the method throws.
  /// </summary>
  /// <param name="pAngles"> the input/output angles </param>
  /// <param name="pWeights"> the input/output weights </param>
  /// <returns>
  /// The computed mean (in ]-PI, PI]).
  /// </returns>
  /// \ingroup Tools
  float weightedMeanAngle(const std::vector<float>& pAngles,
                          const std::vector<float>& pWeights);

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

  bool clipData(
    const float& pMin,
    const float& pMax,
    std::vector<float>& pData);

  bool clipData(
    const float& pMin,
    const float& pMax,
    std::vector<std::vector<float> >& pData);

  /// <summary>
  /** \f$ \left[\begin{array}{c}
    * pPosOut.x \\
    * pPosOut.y \\
    * pPosOut.theta \\
    * \end{array}\right]
    * =
    * \left[\begin{array}{cccccc}
    * cos(pTheta) & -sin(pTheta) & 0 \\
    * sin(pTheta) & cos(pTheta) & 0 \\
    * 0 & 0 & 1 \\
    * \end{array}\right]
    *
    * \left[\begin{array}{c}
    * pPosIn.x \\
    * pPosIn.y \\
    * pPosIn.theta \\
    * \end{array}\right]
    * \f$
    */
  /// </summary>
  /// <param name = "pTheta"> the given angle in radian </param>
  /// <param name = "pPosIn"> a input Pose2D </param>
  /// <param name = "pPosOut"> the output Pose2D </param>
  /// \ingroup Tools
  void changeReferencePose2D(
      const float&  pTheta,
      const Pose2D& pPosIn,
      Pose2D&       pPosOut);

  /// <summary>
  /// Change orientation of a Pose2D in place.
  /// </summary>
  /// <param name = "pTheta"> the given angle in radian </param>
  /// <param name = "pPosOut"> the input output Pose2D </param>
  /// \ingroup Tools
  void changeReferencePose2DInPlace(
      const float& pTheta,
      Pose2D&      pPosOut);

    /// <summary>
    /// Create a Position6D from a Velocity6D
    ///
    /** \f$\begin{array}{ccc}
      * result.x & = & pVel.xd \\
      * result.y & = & pVel.yd \\
      * result.z & = & pVel.zd \\
      * result.wx & = & pVel.wxd \\
      * result.wy & = & pVel.wyd \\
      * result.wz & = & pVel.wzd
      * \end{array} \f$
      */
    ///
    /// </summary>
    /// <param name="pVel"> the given Velocity6D </param>
    /// <returns>
    /// the Position6D result.
    /// </returns>
    /// \ingroup Tools
    Position6D position6DFromVelocity6D(const Velocity6D& pVel);

    /// <summary>
    /// Compute a Position2D from a Pose2D.
    /// The theta member of the Pose2D is not taken into account.
    /// </summary>
    /// <param name = "pPose2D"> the Pose2D to extract </param>
    /// <param name = "pPosition2D"> the result Position2D </param>
    /// \ingroup Tools
    void position2DFromPose2DInPlace(
        const Pose2D& pPose2D,
        Position2D&   pPosition2D);

    /// <summary>
    /// Create a Position2D from a Pose2D
    ///
    /** \f$\begin{array}{ccc}
      * result.x & = & pPose2d.x \\
      * result.y & = & pPose2d.y \\
      * \end{array} \f$
      */
    ///
    /// </summary>
    /// <param name="pPose2D"> the given Pose2D </param>
    /// <returns>
    /// the Position2D result.
    /// </returns>
    /// \ingroup Tools
    Position2D position2DFromPose2D(const Pose2D& pPose2D);

    /// <summary>
    /// Create a Position3D from a Position6D
    ///
    /** \f$\begin{array}{ccc}
      * result.x & = & pPose6d.x \\
      * result.y & = & pPose6d.y \\
      * result.z & = & pPose6d.z \\
      * \end{array} \f$
      */
    ///
    /// </summary>
    /// <param name="pPosition6D"> the given Position6D </param>
    /// <returns>
    /// the Position3D result.
    /// </returns>
    /// \ingroup Tools
    Position3D position3DFromPosition6D(const Position6D& pPosition6D);

    /// <summary>
    /// Overloading of operator * between Rotation and Position3D:
    ///
    /** \f$\left[\begin{array}{c}
      * result.x \\
      * result.y \\
      * result.z
      * \end{array}\right] =
      * \left[\begin{array}{ccc}
      * pRot.r_1c_1 & pRot.r_1c_2 & pRot.r_1c_3 \\
      * pRot.r_2c_1 & pRot.r_2c_2 & pRot.r_2c_3 \\
      * pRot.r_3c_1 & pRot.r_3c_2 & pRot.r_3c_3
      * \end{array}\right] *
      * \left[\begin{array}{c}
      * pPos.x \\
      * pPos.y \\
      * pPos.z
      * \end{array}\right] \f$
      */
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
    /// Overloading of operator * between Quaternion and Position3D
    /// </summary>
    /// <param name="pQuat"> the given Quaternion </param>
    /// <param name="pPos"> the given Position3D </param>
    /// <returns>
    /// the Position3D result.
    /// </returns>
    /// \ingroup Tools
    Position3D operator*(
        const Quaternion& pQuat,
        const Position3D& pPos);


    /// <summary>
    /// Overloading of operator * for float to Position6D, give a Velocity6D:
    ///
    /** \f$\begin{array}{ccc}
      * pVel.xd & = & pVal*pPos.x \\
      * pVel.yd & = & pVal*pPos.y \\
      * pVel.zd & = & pVal*pPos.z \\
      * pVel.wxd & = & pVal*pPos.wx \\
      * pVel.wyd & = & pVal*pPos.wy \\
      * pVel.wzd & = & pVal*pPos.wz
      * \end{array} \f$
      */
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
    /// Overloading of operator * for float to Position3D, give a Velocity3D:
    ///
    /** \f$\begin{array}{ccc}
      * pVel.xd & = & pVal*pPos.x \\
      * pVel.yd & = & pVal*pPos.y \\
      * pVel.zd & = & pVal*pPos.z \\
      * \end{array} \f$
      */
    ///
    /// </summary>
    /// <param name="pVal"> the given float </param>
    /// <param name="pPos"> the given Position3D </param>
    /// <returns>
    /// the Velocity3D
    /// </returns>
    /// \ingroup Tools
    Velocity3D operator*(
      const float       pVal,
      const Position3D& pPos);

    /// <summary>
    /// Overloading of operator * between Rotation and Velocity3D:
    ///
    /** \f$\left[\begin{array}{c}
      * result.x \\
      * result.y \\
      * result.z
      * \end{array}\right] =
      * \left[\begin{array}{ccc}
      * pRot.r_1c_1 & pRot.r_1c_2 & pRot.r_1c_3 \\
      * pRot.r_2c_1 & pRot.r_2c_2 & pRot.r_2c_3 \\
      * pRot.r_3c_1 & pRot.r_3c_2 & pRot.r_3c_3
      * \end{array}\right] *
      * \left[\begin{array}{c}
      * pVel.x \\
      * pVel.y \\
      * pVel.z
      * \end{array}\right] \f$
      */
    ///
    /// </summary>
    /// <param name="pRot"> the given Rotation </param>
    /// <param name="pVel"> the given Velocity3D </param>
    /// <returns>
    /// the Velocity3D result.
    /// </returns>
    /// \ingroup Tools
    Velocity3D operator*(
      const Rotation&   pRot,
      const Velocity3D& pVel);

    /// <summary>
    /// Creates a 3*3 Rotation Matrix from a an angle and a normalized Position3D.
    /// </summary>
    /// <param name="pTheta"> the float value of angle in radian </param>
    /// <param name="pPos"> the Position3D direction of the vector of the rotation, normalized </param>
    /// <returns>
    /// the Rotation matrix
    /// </returns>
    /// \ingroup Tools
    Rotation rotationFromAngleDirection(
      const float&      pTheta,
      const Position3D& pPos);


    /// <summary>
    /// Compute a Position6D from a Pose2D.
    /// </summary>
    /// <param name = "pPose2D"> the Pose2D to extract </param>
    /// <param name = "pPosition6D"> the result Position6D </param>
    /// \ingroup Tools
    void position6DFromPose2DInPlace(
        const Pose2D& pPose2D,
        Position6D&   pPosition6D);

    /// <summary>
    /// Create a Position6D from a Pose2D.
    /// </summary>
    /// <param name = "pPose2D"> the pose2D you want to extract </param>
    /// <returns> the result Position6D </returns>
    /// \ingroup Tools
    Position6D position6DFromPose2D(const Pose2D& pPose2D);

    /// <summary>
    /// Compute a Position6D from a Position3D.
    /// </summary>
    /// <param name = "pPosition3D"> the Position3D to extract </param>
    /// <param name = "pPosition6D"> the result Position6D </param>
    /// \ingroup Tools
    void position6DFromPosition3DInPlace(
        const Position3D& pPosition3D,
        Position6D&       pPosition6D);

    /// <summary>
    /// Create a Position6D from a Position3D.
    /// </summary>
    /// <param name = "pPosition3D"> the position3D you want to extract </param>
    /// <returns> the result Position6D </returns>
    /// \ingroup Tools
    Position6D position6DFromPosition3D(const Position3D& pPosition3D);

    /// <summary>
    /// Compute a Pose2D from a Position6D.
    /// </summary>
    /// <param name = "pPosition6D"> the Position6D you want to extract </param>
    /// <param name = "pPose2D"> the result Pose2D </param>
    /// \ingroup Tools
    void pose2DFromPosition6DInPlace(
        const Position6D& pPosition6D,
        Pose2D&           pPose2D);

    /// <summary>
    /// Create a Pose2D from a Position6D.
    /// </summary>
    /// <param name = "pPosition6D"> the position6d you want to extract </param>
    /// <returns> the Pose2D extracted from the Position6D </returns>
    /// \ingroup Tools
    Pose2D pose2DFromPosition6D(const Position6D& pPosition6D);

    /// <summary>
    /// Compute a Pose2D from a Position2D.
    /// pPose2D.x = pPosition2D.x
    /// pPose2D.y = pPosition2D.y
    /// pPose2D.theta = pAngle
    /// </summary>
    /// <param name = "pPosition2D"> the Position2D you want to extract </param>
    /// <param name = "pAngle"> the angle in radians to set pPose2d to </param>
    /// <param name = "pPose2D"> the result Pose2D </param>
    /// \ingroup Tools
    void pose2DFromPosition2DInPlace(
        const Position2D& pPosition2D,
        const float pAngle,
        Pose2D& pPose2D);

    /// <summary>
    /// Create a Pose2D from a Position2D.
    /// </summary>
    /// <param name = "pPosition2D"> the Position2D you want to extract </param>
    /// <param name = "pAngle"> the angle in radians to set the new Pose2D to </param>
    /// <returns> the Pose2D extracted from the Position2D </returns>
    /// \ingroup Tools
    Pose2D pose2DFromPosition2D(const Position2D& pPosition2D,
                                const float       pAngle=0.0f);

    /// <summary>
    /// Overloading of operator * for Pose2D to Position2D, give a Position2D:
    ///
    /** \f$\begin{array}{ccc}
      * pRes.x & = & pVal.x + cos(pVal.theta)*pPos.x - sin(pVal.theta)*pPos.y \\
      * pRes.y & = & pVal.y + sin(pVal.theta)*pPos.x + cos(pVal.theta)*pPos.y \\
      * \end{array} \f$
      */
    ///
    /// </summary>
    /// <param name="pVal"> the given Pose2D </param>
    /// <param name="pPos"> the given Position2D </param>
    /// <returns>
    /// the Position2D
    /// </returns>
    /// \ingroup Tools
    Position2D operator*(
      const Pose2D& pVal,
      const Position2D& pPos);

    /// <summary>
    /// Create a Quaternion from a Rotation3D when composed
    /// in the following order: Rz(wz) * Ry(wy) * Rx(wx)
    /// </summary>
    /// <param name = "pRot3D"> the rotation3d you want to extract </param>
    /// <returns> the Quaternion extracted from the Rotation3D </returns>
    /// \ingroup Tools
    void quaternionFromRotation3D(
        const Rotation3D& pRot3D,
        Quaternion& pQuaternion);

    Quaternion quaternionFromRotation3D(
        const Rotation3D& pRot3D);

    /// <summary>
    /// Create a Rotation Matrix from a Quaternion
    /// </summary>
    /// <param name = "pQua"> the quaternion you want to extract </param>
    /// <returns> the Rotation matrix extracted from the Quaternion </returns>
    /// \ingroup Tools
    void rotationFromQuaternion(
        const Quaternion& pQua,
        Rotation& pRot);

    Rotation rotationFromQuaternion(
        const Quaternion& pQua);

    /// <summary>
    /// Create a Rotation3D from a Quaternion when composed
    /// in the following order: Rz(wz) * Ry(wy) * Rx(wx)
    /// </summary>
    /// <param name = "pQuaternion"> the quaternion you want to extract </param>
    /// <returns> the Rotation3D extracted from the Quaternion </returns>
    /// \ingroup Tools
    void rotation3DFromQuaternion(
        const Quaternion& pQuaterion,
        Rotation3D& pRot3D);

    Rotation3D rotation3DFromQuaternion(
        const Quaternion& pQuaternion);

    /// <summary>
    /// Convert a Position6D to Quaternion and Position3D
    /// </summary>
    /// <param name = "pPos6D"> the input Position6D you want to extract </param>
    /// <param name = "pQua"> the Quaternion extracted from Position6D </param>
    /// <param name = "pPos3D"> the Position3D extracted from Position6D </param>
    /// \ingroup Tools
    void quaternionPosition3DFromPosition6D(
        const Position6D& pPos6D,
        Quaternion& pQua,
        Position3D& pPos3D);


    /// <summary>
    /// Return the rotational inertia, expressed at the origin, of a point mass
    /// located at pPos. The inertia value is
    /** \f$
      * \left[\begin{array}{ccc}
      *  m (y^2 + z^2) &         -m x y &         -m x z \\
      *         -m x y &  m (x^2 + z^2) &         -m y z \\
      *         -m x z &         -m y z &  m (x^2 + y^2) \\
      * \end{array}\right]
      * \f$
      */
    /// </summary>
    /// <param name = "pMass"> mass of the point</param>
    /// <param name = "pPos"> position of the point</param>
    /// <param name = "pInertia"> a vector of size 9, rotational inertia matrix
    /// of the point mass, expressed at the origin</param>
    /// </param>
    /// Thanks to the Huygens–Steiner theorem, this function can be used to
    /// change the reference of a rigid-body (non-necessarily punctual)
    /// rotational inertia matrix:
    ///
    /// typedef std::vector<float> Inertia;
    /// float mass = ...;
    /// Inertia inertiaAtCom = ...;
    /// AL::Math::Position3d comPosition = ...;
    /// Inertia inertiaAtOrigin;
    /// // first compute the contribution of the translation
    /// pointMassRotationalInertia(mass, comPosition, inertiaAtOrigin);
    /// // then add the "proper" body inertia
    /// std::transform(inertiaAtCom.begin(), inertiaAtCom.end(),
    ///                inertiaAtOrigin.begin(),
    ///                inertiaAtOrigin.begin(),
    ///                std::plus<float>());
    /// \ingroup Tools
    void pointMassRotationalInertia(
      float pMass,
      const Position3D& pPos,
      std::vector<float>& pInertia);
  } // namespace Math
} // namespace AL
#endif  // _LIBALMATH_ALMATH_TOOLS_ALMATH_H_
