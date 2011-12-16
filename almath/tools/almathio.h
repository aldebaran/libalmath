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
#ifndef _LIBALMATH_ALMATH_TOOLS_ALMATHIO_H_
#define _LIBALMATH_ALMATH_TOOLS_ALMATHIO_H_

#include <iostream>
#include <sstream>

#include <almath/types/alpose2d.h>
#include <almath/types/alposition2d.h>
#include <almath/types/alposition3d.h>
#include <almath/types/alposition6d.h>
#include <almath/types/alpositionandvelocity.h>
#include <almath/types/alrotation.h>
#include <almath/types/alrotation3d.h>
#include <almath/types/altransform.h>
#include <almath/types/altransformandvelocity6d.h>
#include <almath/types/alvelocity3d.h>
#include <almath/types/alvelocity6d.h>
#include <almath/types/alquaternion.h>

/// The purpose of grouping ostream operations in one place, is to speed
/// compilation times when not requiring output.
namespace AL {
namespace Math {

/// <summary>
/// Overloading of operator << for Pose2D.
///
/// </summary>
/// <param name="pStream"> the given ostream </param>
/// <param name="pPos"> the given Pose2D </param>
/// <returns>
/// the Pose2D print
/// </returns>
/// \ingroup Types
std::ostream& operator<< (std::ostream& pStream, const Pose2D& pPos);

/// <summary>
/// Overloading of operator << for Position2D.
///
/// </summary>
/// <param name="pStream"> the given ostream </param>
/// <param name="pPos"> the given Position2D </param>
/// <returns>
/// the Position2D print
/// </returns>
/// \ingroup Types
std::ostream& operator<< (std::ostream& pStream, const Position2D& pPos);

/// <summary>
/// Overloading of operator << for Position3D.
///
/// </summary>
/// <param name="pStream"> the given ostream </param>
/// <param name="pPos"> the given Position3D </param>
/// <returns>
/// the Position3D print
/// </returns>
/// \ingroup Types
std::ostream& operator<< (std::ostream& pStream, const Position3D& pPos);

/// <summary>
/// Overloading of operator << for Position6D.
///
/// </summary>
/// <param name="pStream"> the given ostream </param>
/// <param name="pPos"> the given Position6D </param>
/// <returns>
/// the Position6D print
/// </returns>
/// \ingroup Types
std::ostream& operator<< (std::ostream& pStream, const Position6D& pPos);

/// <summary>
/// Overloading of operator << for PositionAndVelocity.
///
/// </summary>
/// <param name="pStream"> the given ostream </param>
/// <param name="pPosVel"> the given PositionAndVelocity </param>
/// <returns>
/// the PositionAndVelocity print
/// </returns>
/// \ingroup Types
std::ostream& operator<< (std::ostream& pStream, const PositionAndVelocity& pPosVel);

/// <summary>
/// Overloading of operator << for Rotation.
///
/// </summary>
/// <param name="pStream"> the given ostream </param>
/// <param name="pRot"> the given Rotation </param>
/// <returns>
/// the Rotation print
/// </returns>
/// \ingroup Types
std::ostream& operator<< (std::ostream& pStream, const Rotation& pRot);

/// <summary>
/// Overloading of operator << for Rotation3D.
///
/// </summary>
/// <param name="pStream"> the given ostream </param>
/// <param name="pRot"> the given Rotation3D </param>
/// <returns>
/// the Rotation3D print
/// </returns>
/// \ingroup Types
std::ostream& operator<< (std::ostream& pStream, const Rotation3D& pRot);

/// <summary>
/// Overloading of operator << for Transform.
///
/// </summary>
/// <param name="pStream"> the given ostream </param>
/// <param name="pT"> the given Transform </param>
/// <returns>
/// the Transform print
/// </returns>
/// \ingroup Types
std::ostream& operator<< (std::ostream& pStream, const Transform& pT);

/// <summary>
/// Overloading of operator << for TransformAndVelocity6D.
///
/// </summary>
/// <param name="pStream"> the given ostream </param>
/// <param name="pTV"> the given TransformAndVelocity6D </param>
/// <returns>
/// the TransformAndVelocity6D print
/// </returns>
/// \ingroup Types
std::ostream& operator<< (std::ostream& pStream, const TransformAndVelocity6D& pTV);

/// <summary>
/// Overloading of operator << for Velocity3D.
///
/// </summary>
/// <param name="pStream"> the given ostream </param>
/// <param name="pVel"> the given Velocity3D </param>
/// <returns>
/// the Velocity3D print
/// </returns>
/// \ingroup Types
std::ostream& operator<< (std::ostream& pStream, const Velocity3D& pVel);

/// <summary>
/// Overloading of operator << for Velocity6D.
///
/// </summary>
/// <param name="pStream"> the given ostream </param>
/// <param name="pVel"> the given Velocity6D </param>
/// <returns>
/// the Velocity6D print
/// </returns>
/// \ingroup Types
std::ostream& operator<< (std::ostream& pStream, const Velocity6D& pVel);

/// <summary>
/// Create a string of Velocity6D.
///
/// </summary>
/// <param name="pVel"> the given Velocity6D </param>
/// <returns>
/// the Velocity6D string
/// </returns>
/// \ingroup Types
std::string toSpaceSeparated(const Velocity6D& pVel);

/// <summary>
/// Create a string of Transform.
///
/// </summary>
/// <param name="pT"> the given Transform </param>
/// <returns>
/// the Transform string
/// </returns>
/// \ingroup Types
std::string toSpaceSeparated(const Transform& pT);

/// <summary>
/// Create a string of Position6D.
///
/// </summary>
/// <param name="pPos"> the given Position6D </param>
/// <returns>
/// the Position6D string
/// </returns>
/// \ingroup Types
std::string toSpaceSeparated(const Position6D& pPos);

/// <summary>
/// Overloading of operator << for Quaternion.
///
/// </summary>
/// <param name="pStream"> the given ostream </param>
/// <param name="pPos"> the given Quaternion </param>
/// <returns>
/// the Quaternion print
/// </returns>
/// \ingroup Types
std::ostream& operator<< (std::ostream& pStream, const Quaternion& pQua);

}
}
#endif  // _LIBALMATH_ALMATH_TOOLS_ALMATHIO_H_
