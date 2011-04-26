#pragma once

#ifndef _LIB_ALMATH_ALMATH_CONVEXHULL_H_
#define _LIB_ALMATH_ALMATH_CONVEXHULL_H_

#include <almath/types/alposition2d.h>
#include <vector>

namespace AL
{
  namespace Math
  {

    /**
     * Useful for polygons
     */
    struct Segment
    {
      Position2D start;
      Position2D end;

      Segment(): start(Position2D()), end(Position2D()) {}

      Segment(
          const Position2D& pStart,
          const Position2D& pEnd):
          start(pStart),
          end(pEnd) {}
    };

    bool isIndexInList(
      unsigned short pk,
      std::vector<unsigned short> pList);

    /**
     * To sort a vector of Position2D
     */
    bool sortPredicate(
      const AL::Math::Position2D& p1,
      const AL::Math::Position2D& p2);

    void removeAlignedPoint(
      const std::vector<AL::Math::Position2D>& pPoints,
      std::vector<AL::Math::Position2D>&       pOut);

    /**
     * To delete doubles points from a none sorted vector of Position2D
     */
    void deleteDoublesInNoneSortVector(
      std::vector<Position2D>& pPoints,
      const float              pEps = 0.0001f);

    /**
     * To delete doubles points from a sorted vector of Position2D
     */
    void deleteDoublesInSortVector(
      std::vector<Position2D>& pPoints,
      const float pEps = 0.0001f);


    /**
    * Function to know the position of a point C relative to a line (2D) (AB)
    * return :
    *   1 if point C is left of (AB)
    *   0 if point C is on (AB)
    *  -1 if point C is right of (AB)
    */
    int isLeft(
      const Position2D& pA,
      const Position2D& pB,
      const Position2D& pC,
      const float&      pEps = 0.00001f);

    /**
    * Function to know the position of a point C relative to a line (2D) (AB)
    * return :
    * -4: at least two points are equal
    * -3: points are nearly aligned but B is not in the middle of A, C
    * -2: points are aligned and B is in the middle of A, C
    * -1: if point C is right of (AB)
    * +1: if point C is left of (AB)
    */
    int isLeftBest(
      const Position2D& pA,
      const Position2D& pB,
      const Position2D& pC);


    int isLeftOld(
      const Position2D& pA,
      const Position2D& pB,
      const Position2D& pC,
      const float&      pEps = 0.000001f);

    /**
    * Simplify the ConvexHull.
    * param "pConvexHull" The brut convex hull.
    */
    void simplifyConvexHull(std::vector<AL::Math::Position2D>& pConvexHull);

    /**
    * Function to get the convex hull of a set of points sorted by Y axis
    * (if Y equal, sorted by X axis)
    * Uses Jarvis March algorithm O(nh) where n = number of point in input
    * and h number of points in the convex hull.
    * In our case (n = 8 and max(h) = 8) this algorithm is very fast.
    * If we got a big number of point in input, use an other algorithm
    * Algorithm :
    * def jarvis(P) //P is the set of points
    *   i = 0
    *   p[0] = leftmost point of P
    *   do
    *     p[i+1] = point such that all other points in P are to the
    *               right of the line p[i]p[i+1]
    *     i = i + 1
    *   while p[i] != p[0]
    *   return p
    */
    std::vector<AL::Math::Position2D> getConvexHull(
      std::vector<AL::Math::Position2D>& pPoints);


//    std::vector<AL::Math::Position2D> jarvis(
//      std::vector<AL::Math::Position2D>& pPoints);

    //    /**
    //     * Get the closest line from the polygon to the point.
    //     *
    //     * The polygon is given by a sequence of points, (the
    //     * corners)
    //     *
    //     * We build a sequence of segments and return the closest
    //     * one
    //     *
    //     */
    //    Segment getClosestSegment(
    //      Position2D              pPoint,
    //      std::vector<Position2D> pPolygon);

    //    /**
    //     * Distance from a point to a segment.
    //     *
    //     */
    //    float distance(
    //      const Position2D& pPos1,
    //      const Segment&    pSegment);

    //    /**
    //     * Used for building convex hull
    //     */
    //    float getDirection(
    //      const AL::Math::Position2D& p0,
    //      const AL::Math::Position2D& p1,
    //      const AL::Math::Position2D& p2);

  } // namespace Math
} // namespace AL

#endif  // _LIB_ALMATH_ALMATH_CONVEXHULL_H_

