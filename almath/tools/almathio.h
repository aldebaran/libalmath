/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALMATHIO_H_
#define _LIB_ALMATH_ALMATH_ALMATHIO_H_

#include <iostream>
#include <sstream>

#include <almath/types/alpose2d.h>
#include <almath/types/alposition2d.h>
#include <almath/types/alposition3d.h>
#include <almath/types/alposition6d.h>
#include <almath/types/alpositionandvelocity.h>
#include <almath/types/alrotation.h>
#include <almath/types/alrotation2d.h>
#include <almath/types/alrotation3d.h>
#include <almath/types/altransform.h>
#include <almath/types/altransformandvelocity6d.h>
#include <almath/types/alvelocity3d.h>
#include <almath/types/alvelocity6d.h>


/// The purpose of grouping ostream operations in one place, is to speed
/// compilation times when not requiring output.
namespace AL {
namespace Math {

/// <summary>
/// overloading of operator << for Pose2D
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
/// overloading of operator << for Position2D
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
/// overloading of operator << for Position3D
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
/// overloading of operator << for Position6D
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
/// overloading of operator << for PositionAndVelocity
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
/// overloading of operator << for Rotation
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
/// overloading of operator << for Rotation2D
///
/// </summary>
/// <param name="pStream"> the given ostream </param>
/// <param name="pRot"> the given Rotation2D </param>
/// <returns>
/// the Rotation2D print
/// </returns>
/// \ingroup Types
std::ostream& operator<< (std::ostream& pStream, const Rotation2D& pRot);

/// <summary>
/// overloading of operator << for Rotation3D
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
/// overloading of operator << for Transform
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
/// overloading of operator << for TransformAndVelocity6D
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
/// overloading of operator << for Velocity3D
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
/// overloading of operator << for Velocity6D
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
/// create a string of Velocity6D
///
/// </summary>
/// <param name="pVel"> the given Velocity6D </param>
/// <returns>
/// the Velocity6D string
/// </returns>
/// \ingroup Types
std::string toSpaceSeparated(const Velocity6D& pVel);

/// <summary>
/// create a string of Transform
///
/// </summary>
/// <param name="pT"> the given Transform </param>
/// <returns>
/// the Transform string
/// </returns>
/// \ingroup Types
std::string toSpaceSeparated(const Transform& pT);

/// <summary>
/// create a string of Position6D
///
/// </summary>
/// <param name="pPos"> the given Position6D </param>
/// <returns>
/// the Position6D string
/// </returns>
/// \ingroup Types
std::string toSpaceSeparated(const Position6D& pPos);

}
}
#endif  // _LIB_ALMATH_ALMATH_ALMATHIO_H_
