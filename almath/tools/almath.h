/*
 * Copyright (c) 2012, Aldebaran Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Aldebaran Robotics nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Aldebaran Robotics BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#pragma once
#ifndef _LIBALMATH_ALMATH_TOOLS_ALMATH_H_
#define _LIBALMATH_ALMATH_TOOLS_ALMATH_H_

#include <almath/types/alvelocity6d.h>
#include <almath/types/alposition3d.h>
#include <almath/types/alposition6d.h>
#include <almath/types/alrotation.h>

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


  } // namespace Math
} // namespace AL
#endif  // _LIBALMATH_ALMATH_TOOLS_ALMATH_H_
