/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/


#pragma once
#ifndef _LIBALMATH_ALMATH_TYPES_ALQUATERNION_H_
#define _LIBALMATH_ALMATH_TYPES_ALQUATERNION_H_

#include <vector>

namespace AL {
  namespace Math {

    /// <summary>
    /// Create and play with a Quaternion.
    ///
    /// A Quaternion is just defined by w, x, y and z.
    /// </summary>
    /// \ingroup Types
    struct Quaternion {
      /// <summary> </summary>
      float w;
      /// <summary> </summary>
      float x;
      /// <summary> </summary>
      float y;
      /// <summary> </summary>
      float z;

      /// <summary>
      /// Create a Quaternion initialize with 0.0f.
      /**
       *
       * \f$ \left[\begin{array}{c}
       *         w \\
       *         x \\
       *         y \\
       *         z
       *      \end{array}\right] =
       *      \left[\begin{array}{c}
       *         0.0 \\
       *         0.0 \\
       *         0.0 \\
       *         0.0
       *      \end{array}\right]\f$
       */
      /// </summary>
      Quaternion();

      /// <summary>
      /// Create a Quaternion initialize with explicit value.
      /**
       *
       * \f$ \left[\begin{array}{c}
       *         w \\
       *         x \\
       *         y \\
       *         z
       *      \end{array}\right] =
       *      \left[\begin{array}{c}
       *         pW \\
       *         pX \\
       *         pY \\
       *         pZ
       *      \end{array}\right]\f$
       */
      /// </summary>
      /// <param name="pW"> the float value for w </param>
      /// <param name="pX"> the float value for x </param>
      /// <param name="pY"> the float value for y </param>
      /// <param name="pZ"> the float value for z </param>
      Quaternion(
        float pW,
        float pX,
        float pY,
        float pZ);

      /// <summary>
      /// Create a Quaternion with an std::vector.
      /**
       *
       * \f$ \left[\begin{array}{c}
       *         w \\
       *         x \\
       *         y \\
       *         z
       *      \end{array}\right] =
       *      \left[\begin{array}{c}
       *         pFloats[0] \\
       *         pFloats[1] \\
       *         pFloats[2] \\
       *         pFloats[3]
       *      \end{array}\right]\f$
       */
      /// </summary>
      /// <param name="pFloats">
      /// An std::vector<float> of size 4 for respectively:
      /// w, x, y and z
      /// </param>
      Quaternion(const std::vector<float>& pFloats);

      /// <summary>
      /// Overloading of operator == for Quaternion.
      /// </summary>
      /// <param name="pQua2"> the second Quaternion </param>
      bool operator== (const Quaternion& pQua2) const;

      /// <summary>
      /// Overloading of operator != for Quaternion.
      /// </summary>
      /// <param name="pQua2"> the second Quaternion </param>
      bool operator!= (const Quaternion& pQua2) const;

      /// <summary>
      /// Check if the actual Quaternion is near the one
      /// give in argument.
      ///
      /// </summary>
      /// <param name="pQua2"> the second Quaternion </param>
      /// <param name="pEpsilon"> an optionnal epsilon distance </param>
      /// <returns>
      /// true if the distance between the two Quaternion is less than pEpsilon
      /// </returns>
      bool isNear(
        const Quaternion& pQua2,
        const float&      pEpsilon=0.0001f) const;

      /// <summary>
      /// Return the Quaternion as a vector of float [w, x, y, z].
      /// </summary>
      std::vector<float> toVector() const;
    };

  } // end namespace math
} // end namespace al
#endif  // _LIBALMATH_ALMATH_TYPES_ALQUATERNION_H_
