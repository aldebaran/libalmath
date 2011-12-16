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
#ifndef _LIBALMATH_ALMATH_TYPES_ALROTATION3D_H_
#define _LIBALMATH_ALMATH_TYPES_ALROTATION3D_H_

#include <vector>

namespace AL {
  namespace Math {

  /// <summary>
  /// A Rotation3D give 3 composed angles in radians.
  /// </summary>
  /// \ingroup Types
    struct Rotation3D {
      /// <summary> </summary>
      float wx;
      /// <summary> </summary>
      float wy;
      /// <summary> </summary>
      float wz;

      /// <summary>
      /// Create a Rotation3D initialized with 0.0f.
      /**
       *
       * \f$ \left[\begin{array}{c}
       *         wx \\
       *         wy \\
       *         wz
       *      \end{array}\right] =
       *      \left[\begin{array}{c}
       *         0.0 \\
       *         0.0 \\
       *         0.0
       *      \end{array}\right]\f$
       */
      /// </summary>
      Rotation3D();

      /// <summary>
      /// Create a Rotation3D initialized with the same float.
      /// </summary>
      /// <param name="pInit"> the float value for each member </param>
      /**
       *
       * \f$ \left[\begin{array}{c}
       *         wx \\
       *         wy \\
       *         wz
       *      \end{array}\right] =
       *      \left[\begin{array}{c}
       *         pInit \\
       *         pInit \\
       *         pInit
       *      \end{array}\right]\f$
       */
      /// </summary>
      explicit Rotation3D(float pInit);

      /// <summary>
      /// Create a Rotation3D initialized with explicit value.
      /**
       *
       * \f$ \left[\begin{array}{c}
       *         wx \\
       *         wy \\
       *         wz
       *      \end{array}\right] =
       *      \left[\begin{array}{c}
       *         pWx \\
       *         pWy \\
       *         pWz
       *      \end{array}\right]\f$
       */
      /// </summary>
      /// <param name="pWx"> the float value for wx </param>
      /// <param name="pWy"> the float value for wy </param>
      /// <param name="pWz"> the float value for wz </param>
      Rotation3D(
        float pWx,
        float pWy,
        float pWz);

      /// <summary>
      /// Create a Rotation3D with an std::vector.
      /**
       *
       * \f$ \left[\begin{array}{c}
       *         wx \\
       *         wy \\
       *         wz
       *      \end{array}\right] =
       *      \left[\begin{array}{c}
       *         pFloats[0] \\
       *         pFloats[1] \\
       *         pFloats[2]
       *      \end{array}\right]\f$
       */
      /// </summary>
      /// <param name="pFloats">
      /// An std::vector<float> of size 3 for respectively:
      /// wx, wy and wz
      /// </param>
      Rotation3D (const std::vector<float>& pFloats);

      /// <summary>
      /// Overloading of operator + for Rotation3D.
      /// </summary>
      /// <param name="pRot2"> the second Rotation3D </param>
      Rotation3D operator+ (const Rotation3D& pRot2) const;

      /// <summary>
      /// Overloading of operator - for Rotation3D.
      /// </summary>
      /// <param name="pRot2"> the second Rotation3D </param>
      Rotation3D operator- (const Rotation3D& pRot2) const;

      /// <summary>
      /// Overloading of operator += for Rotation3D.
      /// </summary>
      /// <param name="pRot2"> the second Rotation3D </param>
      Rotation3D& operator+= (const Rotation3D& pRot2);

      /// <summary>
      /// Overloading of operator -= for Rotation3D.
      /// </summary>
      /// <param name="pRot2"> the second Rotation3D </param>
      Rotation3D& operator-= (const Rotation3D& pRot2);

      /// <summary>
      /// Overloading of operator == for Rotation3D.
      /// </summary>
      /// <param name="pRot2"> the second Rotation3D </param>
      bool operator== (const Rotation3D& pRot2) const;

      /// <summary>
      /// Overloading of operator != for Rotation3D.
      /// </summary>
      /// <param name="pRot2"> the second Rotation3D </param>
      bool operator!= (const Rotation3D& pRot2) const;

      /// <summary>
      /// Overloading of operator * for Rotation3D.
      /// </summary>
      /// <param name="pVal"> the float factor </param>
      Rotation3D operator* (const float pVal) const;

      /// <summary>
      /// Overloading of operator / for Rotation3D.
      /// </summary>
      /// <param name="pVal"> the float factor </param>
      Rotation3D operator/ (const float pVal) const;

      /// <summary>
      /// Overloading of operator *= for Rotation3D.
      /// </summary>
      /// <param name="pVal"> the float factor </param>
      Rotation3D& operator*= (const float pVal);

      /// <summary>
      /// Overloading of operator /= for Rotation3D.
      /// </summary>
      /// <param name="pVal"> the float factor </param>
      Rotation3D& operator/= (const float pVal);

      /// <summary>
      /// Check if the actual Rotation3D is near the one
      /// given in argument.
      ///
      /// </summary>
      /// <param name="pRot2"> the second Rotation3D </param>
      /// <param name="pEpsilon"> an optional epsilon distance </param>
      /// <returns>
      /// true if the difference of each float of the two Rotation3D is less than pEpsilon
      /// </returns>
      bool isNear(
        const Rotation3D& pRot2,
        const float&      pEpsilon=0.0001f) const;

      /// <summary>
      /// Compute the norm of the actual Position6D:
      ///
      /// \f$\sqrt{pRot.wx^2 + pRot.wy^2 + pRot.wz^2}\f$
      /// </summary>
      /// <returns>
      /// the float norm of the Position6D
      /// </returns>
      float norm() const;

      /// <summary>
      /// Return the Rotation3D as a vector of float [wx, wy, wz].
      /// </summary>
      std::vector<float> toVector() const;
    };

    /// <summary>
    /// Compute the norm of a Rotation3D:
    ///
    /// \f$\sqrt{pRot.wx^2 + pRot.wy^2 + pRot.wz^2}\f$
    /// </summary>
    /// <param name="pRot"> the given Rotation3D </param>
    /// <returns>
    /// the float norm of the given Rotation3D
    /// </returns>
    /// \ingroup Types
    float norm(const Rotation3D& pRot);

  } // end namespace Math
} // end namespace AL
#endif  // _LIBALMATH_ALMATH_TYPES_ALROTATION3D_H_
