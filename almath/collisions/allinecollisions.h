/**
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved \n
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALLINECOLLISIONS_H_
#define _LIB_ALMATH_ALMATH_ALLINECOLLISIONS_H_

#include <almath/types/alposition3d.h>
#include <vector>


namespace AL
{
  namespace Math
  {
    /**
    * Function to compute the shortest distance between a point and a line segment
    * We use dot product to compute this distance
    *
    * @param pA the start point of the line segment
    * @param pB the end point of the line segment
    * @param pC an other point
    */
    float DistancePointToLineSegment3D(
        Position3D& pA,
        Position3D& pB,
        Position3D& pC);

    /**
    * Function to compute the shortest distance between a point and a line segment
    * We use dot product to compute this distance
    *
    * @param pA the start point of the line segment
    * @param pB the end point of the line segment
    * @param pC an other point
    * @return the closest point of the line 3D to pC
    */
    Position3D ClosestPointOfLineSegment3D(Position3D& pA, Position3D& pB, Position3D& pC);

    /**
    * Function to compute the shortest distance between two line segments
    *
    * @param pA the start point of the first line segment
    * @param pB the end point of the first line segment
    * @param pC the start point of the second line segment
    * @param pD the end point of the second line segment
    *
    * http://softsurfer.com/Archive/algorithm_0106/algorithm_0106.htm
    *  Copyright 2001 - 2006, softSurfer (www.softsurfer.com)
    *  This code may be freely used and modified for any purpose
    *  providing that this copyright notice is included with it.
    *  SoftSurfer makes no warranty for this code, and cannot be held
    *  liable for any real or imagined damage resulting from its use.
    *  Users of this code must verify correctness for their application.
    */
    float DistanceBetweenTwoLineSegment3D(Position3D& pA, Position3D& pB, Position3D& pC, Position3D& pD);


  } // namespace Math

} // namespace AL

#endif  // _LIB_ALMATH_ALMATH_ALLINECOLLISIONS_H_
