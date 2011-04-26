/**
* @author Chris Kilner
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved
*
* Concentrate the use of iostream into this file
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALMATHIO_H_
#define _LIB_ALMATH_ALMATH_ALMATHIO_H_

#include <iostream>
#include <sstream>

#include <almath/types/alrotation.h>
#include <almath/types/alrotation2d.h>
#include <almath/types/alrotation3d.h>
#include <almath/types/alvelocity6d.h>
#include <almath/types/altransform.h>
#include <almath/types/alposition2d.h>
#include <almath/types/alposition3d.h>
#include <almath/types/alposition6d.h>
#include <almath/types/alpose2d.h>
#include <almath/types/alvelocity3d.h>

namespace AL {
  namespace Math {

    std::ostream& operator<< (std::ostream& pStream, const Velocity6D& p);

    std::string toSpaceSeparated(const Velocity6D& p);

    std::ostream& operator<< (std::ostream& pStream, const Rotation3D& p);

    std::ostream& operator<< (std::ostream& pStream, const Transform& pT);

    std::string toSpaceSeparated(const Transform& pT);

    std::ostream& operator<< (std::ostream& pStream, const Rotation& p);

    std::ostream& operator<< (std::ostream& pStream, const Rotation2D& p);

    std::ostream& operator<< (std::ostream& pStream, const Position3D& p);

    std::ostream& operator<< (std::ostream& pStream, const Position2D& p);

    std::ostream& operator<< (std::ostream& pStream, const Pose2D& p);

    std::ostream& operator<< (std::ostream& pStream, const Velocity3D& p);

    std::ostream& operator<< (std::ostream& pStream, const Position6D& p);

    std::string toSpaceSeparated(const Position6D& p);

  }
}
#endif  // _LIB_ALMATH_ALMATH_ALMATHIO_H_
