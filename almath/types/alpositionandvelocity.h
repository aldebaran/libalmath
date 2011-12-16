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
#ifndef _LIBALMATH_ALMATH_TYPES_ALPOSITIONANDVELOCITY_H_
#define _LIBALMATH_ALMATH_TYPES_ALPOSITIONANDVELOCITY_H_

namespace AL {
  namespace Math {

    /// <summary>
    /// Create and play with a PositionAndVelocity.
    ///
    /// A PositionAndVelocity is just defined by q and dq.
    /// </summary>
    /// \ingroup Types
    struct PositionAndVelocity
    {
      /// <summary> </summary>
      float q;
      /// <summary> </summary>
      float dq;

      /// <summary>
      /// Create a PositionAndVelocity initialize with explicit value.
      /**
       *
       * \f$ \left[\begin{array}{c}
       *         q \\
       *         dq
       *      \end{array}\right] =
       *      \left[\begin{array}{c}
       *         pq \\
       *         pdq
       *      \end{array}\right]\f$
       */
      /// </summary>
      /// <param name="pq"> the float value for q </param>
      /// <param name="pdq"> the float value for dq </param>
      PositionAndVelocity(
          const float pq = 0.0f,
          const float pdq = 0.0f);

      /// <summary>
      /// Check if the actual PositionAndVelocity is near the one
      /// give in argument.
      ///
      /// </summary>
      /// <param name="pDat2"> the second PositionAndVelocity </param>
      /// <param name="pEpsilon"> an optional epsilon distance (default = 0.0001f) </param>
      /// <returns>
      /// true if the difference of each float of the two PositionAndVelocity is less than pEpsilon
      /// </returns>
      bool isNear(
        const PositionAndVelocity& pDat2,
        const float&               pEpsilon=0.0001f) const;
    };

  }
}
#endif  // _LIBALMATH_ALMATH_TYPES_ALPOSITIONANDVELOCITY_H_
