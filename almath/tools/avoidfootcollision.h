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
#ifndef _LIBALMATH_ALMATH_TOOLS_AVOIDFOOTCOLLISION_H_
#define _LIBALMATH_ALMATH_TOOLS_AVOIDFOOTCOLLISION_H_

#include <almath/types/alpose2d.h>
#include <vector>

namespace AL
{
  namespace Math
  {
    /// <summary>
    /// Compute the best position(orientation) of the foot to avoid collision.
    /// </summary>
    /// <param name="pLFootBoundingBox">  vector<Pose2D> of the left footBoundingBox.</param>
    /// <param name="pRFootBoundingBox">  vector<Pose2D> of the right footBoundingBox.</param>
    /// <param name="pIsLeftSupport">     Bool true if left is the support leg. </param>
    /// <param name="pMove">              the desired and return Pose2D. </param>
    /// <returns>
    /// true if pMove is clamped.
    /// </returns>
    /// \ingroup Tools
    const bool avoidFootCollision(
        const std::vector<Pose2D>&  pLFootBoundingBox,
        const std::vector<Pose2D>&  pRFootBoundingBox,
        const bool&                 pIsLeftSupport,
        Pose2D&                     pMove);


    /// <summary>
    /// Clip foot move with ellipsoid function
    /// </summary>
    /// <param name="pMaxFootX">  float of the max step along x axis. </param>
    /// <param name="pMaxFootY">  float of the max step along y axis. </param>
    /// <param name="pMove">      the desired and return Pose2D. </param>
    /// <returns>
    /// true if pMove is clamped.
    /// </returns>
    /// \ingroup Tools
    const bool clipFootWithEllipse(
        const float&    pMaxFootX,
        const float&    pMaxFootY,
        Pose2D&         pMove);

  } // namespace Math
} // namespace AL

#endif  // _LIBALMATH_ALMATH_TOOLS_AVOIDFOOTCOLLISION_H_

