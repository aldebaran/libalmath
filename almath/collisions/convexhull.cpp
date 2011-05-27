#include <almath/collisions/convexhull.h>

#include "float.h"
#include <algorithm>
#include <vector>
#include <almath/tools/almath.h>
#include <stdexcept>
#include <iostream>
#include <sstream>

#include <almath/tools/almathio.h>

//#define DEBUG_CONVEX

namespace AL
{
  namespace Math
  {

  bool isIndexInList(
    unsigned short              pk,
    std::vector<unsigned short> pList)
  {
    for (unsigned int i=0; i<pList.size(); i++)
    {
      if (pk == pList.at(i))
      {
        return true;
      }
    }
    return false;
  }


  bool sortPredicate(
    const AL::Math::Position2D& p1,
    const AL::Math::Position2D& p2)
  {
    if (p1.x == p2.x)
    {
      return (p1.y < p2.y);
    }

    return  (p1.x < p2.x);
  }

  void deleteDoublesInNoneSortVector(
      std::vector<Position2D>& pPoints,
      const float              pEps)
    {
      std::vector<bool> isDouble;
      isDouble.resize(pPoints.size());
      for (unsigned int i=0; i<isDouble.size(); i++)
      {
        isDouble.at(i) = false;
      }

      // Find doubles
      for (unsigned int i=0; i<pPoints.size()-1; i++)
      {
        for (unsigned int j=i+1; j<pPoints.size(); j++)
        {
          if (pPoints.at(i).isNear(pPoints.at(j), pEps))
          {
            isDouble.at(i) = true;
          }
        }
      }

      // Delete doubles
      unsigned int nbDeleted = 0;
      for (unsigned int i=0; i<isDouble.size(); i++)
      {
        if (isDouble.at(i))
        {
          pPoints.erase(pPoints.begin()+(i-nbDeleted));
          nbDeleted++;
        }
      }

    }

    // Algo works if double are contigus
    void deleteDoublesInSortVector(
      std::vector<Position2D>& pPoints,
      const float              pEps)
    {

      for (unsigned int i=0; i<pPoints.size()-1;)
      {
        if (pPoints.at(i).isNear(pPoints.at(i+1), pEps))
          //if (pPoints[i] == pPoints[i+1])
        {
          pPoints.erase(pPoints.begin() + i);
        }
        else
        {
          i++;
        }
      }
    } // end deleteDoublesInSortVector


    int isLeftOld(
      const Position2D& pA,
      const Position2D& pB,
      const Position2D& pC,
      const float&      pEps)
    {
      float sinAngle = ((pA.y - pC.y)*(pB.x - pA.x)
                        -(pA.x - pC.x)*(pB.y - pA.y));

      if (sinAngle < -pEps) // was 0.0f
      {
        return 1;
      }
      else if (sinAngle > +pEps) // was 0.0f
      {
        return -1;
      }
      else
      {
        return 0;
      }
    } // end isLeftOld


    int isLeftBest(
      const Position2D& pA,
      const Position2D& pB,
      const Position2D& pC)
    {
      // -4: at least two points are equal
      // -3: points are nearly aligned but pB is not in the middle of pA, pC
      // -2: points are aligned and pB is in the middle of pA, pC
      // -1: pC is left
      // +1: pC is right

      // point are near with epsilonFloat = 0.00001f;
      float epsilonFloat = 0.0001f;

      if (pA.isNear(pB, epsilonFloat) ||
          pA.isNear(pC, epsilonFloat) ||
          pB.isNear(pC, epsilonFloat))
      {
        return -4;
      }

      AL::Math::Position2D d1 = pB - pA;
      AL::Math::Position2D d2 = pA - pC;

      try
      {
        d1 = AL::Math::normalize(d1);
      }
      catch(const std::exception e)
      {
        return -4;
      }

      try
      {
        d2 = AL::Math::normalize(d2);
      }
      catch(const std::exception e)
      {
        return -4;
      }

      float sinAngle = 0.0f;
      AL::Math::crossProduct(d1, d2, sinAngle);

      // point are nearly aligned angle = 0.3 degree
      float epsilonAngle = sinf(0.01f*TO_RAD); // was 0.3

      if (sinAngle < -epsilonAngle) // was 0.0f
      {
        return 1;
      }
      else if (sinAngle > +epsilonAngle) // was 0.0f
      {
        return -1;
      }
      else
      {
        d1 = pA - pB;
        try
        {
          d1 = AL::Math::normalize(d1);
        }
        catch(const std::exception e)
        {
          return -4;
        }
        d2 = pC - pB;
        try
        {
          d2 = AL::Math::normalize(d2);
        }
        catch(const std::exception e)
        {
          return -4;
        }
        // Point are different and nearly aligned
        // -3: points are nearly aligned but pB is not in the middle of pA, pC
        // -2: points are aligned and pB is in the middle of pA, pC

        // epsilon must be big because of error tolerance (point can be "nearly" aligned
        // and if d1!=d2, there is a big difference between the two vector
        // epsilonFloat = 0.01f;
        // if (d1.isNear(d2, epsilonFloat))
        // {
        //   return -3;
        // }
        // else
        // {
        //   return -2;
        // }

        float distanceAB = AL::Math::distance(pA, pB);
        float distanceAC = AL::Math::distance(pA, pC);
        float distanceBC = AL::Math::distance(pB, pC);
        if (
            (distanceAB > distanceAC) ||
            (distanceBC > distanceAC))
        {
          return -3;
        }
        else
        {
          return -2;
        }
      }
    } // end isLeftBest


    int isLeft(
      const Position2D& pA,
      const Position2D& pB,
      const Position2D& pC,
      const float&      pEps)
    {
      // if return 0, point are equal or in a same line

      // AL::Math::Position2D pAB = pB - pA;
      // AL::Math::crossProduct()
      // crossProduct(pB - pA, pC - pA)

      AL::Math::Position2D d1 = pB - pA;
      AL::Math::Position2D d2 = pA - pC;

      try
      {
        d1 = AL::Math::normalize(d1);
      }
      catch(const std::exception e)
      {
        return 0;
      }

      try
      {
        d2 = AL::Math::normalize(d2);
      }
      catch(const std::exception)
      {
        return 0;
      }

      float sinAngle = 0.0f;
      AL::Math::crossProduct(d1, d2, sinAngle);

      // float sinAngle = ((pA.y - pC.y)*(pB.x - pA.x)
      //                  -(pA.x - pC.x)*(pB.y - pA.y));

      //std::cout << "compare sinAngle/leftParameter: " << sinAngle << " " << leftParameter << std::endl;

      if (sinAngle < -pEps)
      {
        return 1;
      }
      else if (sinAngle > pEps)
      {
        return -1;
      }
      else
      {
        return 0;
      }
    } // end isLeft


    void removeAlignedPoint(
      const std::vector<AL::Math::Position2D>& pPoints,
      std::vector<AL::Math::Position2D>&       pOut)
    {
      unsigned int nbPts = pPoints.size();
      std::vector<bool> isInList;
      //isInList.resize(pPoints.size());
      isInList.reserve(nbPts);
      isInList.assign(nbPts, false);

      // possible optimization:
      pOut.clear();
      //std::vector<unsigned short> isInTheMiddle;
      for (unsigned int j=0; j<nbPts; j++)
      {
        for (unsigned int i=0; i<nbPts; i++)
        {
          for (unsigned int k=i+1; k<nbPts; k++) // k = i+1
          {
            if ((i != j) && (j != k)) //(i != k)
            {
              if (AL::Math::isLeftBest(pPoints.at(i), pPoints.at(j), pPoints.at(k)) == -2)
              {
                isInList.at(j) = true;
//                if (
//                    ((j == 396) || (j == 397) || (j == 398) || (j == 399))
//                    )
//                {
//                  std::cout << "i: " << i << " j: " << j << " k: " << k << " isLeft: " << AL::Math::isLeftBest(pPoints.at(i), pPoints.at(j), pPoints.at(k)) << std::endl;
//                  std::cout << "i: " << i << " " << pPoints.at(i) << std::endl;
//                  std::cout << "j: " << j << " " << pPoints.at(j) << std::endl;
//                  std::cout << "k: " << k << " " << pPoints.at(k) << std::endl;
//                }

                // if (!AL::Math::isIndexInList(j, isInTheMiddle))
                // {
                //   isInTheMiddle.push_back(j);
                // }
              }
            }
          }
        }
      }

      for (unsigned int i=0; i<nbPts; i++)
      {
        if (!isInList.at(i))
        {
          pOut.push_back(pPoints.at(i));
        }
//        if (!AL::Math::isIndexInList(i, isInTheMiddle))
//        {
//          pOut.push_back(pPoints.at(i));
//        }
      }

    } // end removeAlignedPoint


    void simplifyConvexHull(std::vector<AL::Math::Position2D>& pConvexHull)
      {
        //sort the list of points
        pConvexHull.pop_back();
        std::vector<unsigned int> idToDelete;
        const float offsetAngle = sinf(3.0f * TO_RAD); // clement // CK wtf! FIXME

        unsigned int i = 0, j = 0, k = 0;
        float sinAngle = 0.0f;

        AL::Math::Position2D d1 = AL::Math::Position2D();
        AL::Math::Position2D d2 = AL::Math::Position2D();

        for (j = 0 ; j < pConvexHull.size() ; j++)
        {
          (j == 0) ? i = pConvexHull.size()-1 : i = j - 1;
          (j == pConvexHull.size()-1)? k = 0 : k = j + 1;

          d1 = pConvexHull.at(j) - pConvexHull.at(i);
          d1 = AL::Math::normalize(d1);

          d2 = pConvexHull.at(k) - pConvexHull.at(j);
          d2 = AL::Math::normalize(d2);

          AL::Math::crossProduct(d1, d2, sinAngle);
          //float sinAngle = AL::Math::crossProduct(d1, d2);

          if (fabsf(sinAngle) < offsetAngle)
          {
            idToDelete.push_back(j);
          }
        }

        // CK this is madness ...
        // TODO just keep a track of the number that have been deleted.
        for (i = 0 ; i < idToDelete.size() ; i++)
        {
          j = idToDelete.at(i);
          pConvexHull.erase(pConvexHull.begin() + j);
          for ( k = i+1 ; k < idToDelete.size() ; k++ )
          {
            idToDelete.at(k)--;
          }
        }
        pConvexHull.push_back(pConvexHull.front());
      } // end simplifyConvexHull


    // def jarvis(P) //P is the set of points
    //   i = 0
    //   p[0] = leftmost point of P
    //   do
    //     p[i+1] = point such that all other points in P are to the
    //               right of the line p[i]p[i+1]
    //     i = i + 1
    //   while p[i] != p[0]
    //   return p
//    std::vector<AL::Math::Position2D> jarvis(
//      std::vector<AL::Math::Position2D>& pPoints)
//    {
//      std::vector<AL::Math::Position2D> solution;

//      float epsilon = 0.00001f;
//      AL::Math::deleteDoublesInNoneSortVector(pPoints, epsilon);
//      if (pPoints.size() < 3)
//      {
//        //std::cout<<"\nNumber of points insufficient\n";
//        return pPoints;
//      }
//      //sort the list of points
//      std::sort(pPoints.begin(), pPoints.end(), sortPredicate);

//      //leftmost point of P
//      solution.push_back(pPoints.at(0));

//      return solution;
//    } // end jarvis


    std::vector<AL::Math::Position2D> getConvexHull(
      std::vector<AL::Math::Position2D>& pPointsInput)
    {
//      std::cout << "-------" << std::endl;
//      for (unsigned int i=0; i<pPointsInput.size(); i++)
//      {
//        std::cout << "(" << pPointsInput.at(i).x << "f, " << pPointsInput.at(i).y << "f);" << std::endl;
//      }
//      std::cout << std::endl;
      std::vector<AL::Math::Position2D> lConvexHull;
      std::vector<AL::Math::Position2D>::iterator itPoints;
      AL::Math::deleteDoublesInNoneSortVector(pPointsInput);
      std::vector<AL::Math::Position2D> pPoints;
      AL::Math::removeAlignedPoint(pPointsInput, pPoints);

//      for (unsigned int i=0; i<pPoints.size(); i++)
//      {
//        std::cout << "(" << pPoints.at(i).x << "f, " << pPoints.at(i).y << "f);" << std::endl;
//      }

      if (pPoints.size() < 3)
      {
        //std::cout<<"\nNumber of points insufficient\n";
        return pPoints;
      }
      unsigned short i = 0, k = 1;
      bool firstPass = false;
      bool sortie    = false;
      //sort the list of points
      std::sort(pPoints.begin(), pPoints.end(), sortPredicate);
      //push the leftmost point of pPoint in the convex hull
      lConvexHull.push_back(pPoints.at(0));
      do
      {
        bool isOK = true;
        //The next point of the convex hull is the one such that all points of pPoints are left of
        // the line defined by the last point in the convex hull and the point in pPoints we test
        for (itPoints = pPoints.begin() ; itPoints != pPoints.end() ; itPoints++)
        {
          if (AL::Math::isLeftOld(lConvexHull.at(i), pPoints.at(k), (*itPoints)) < 0)
          {
            isOK = false;
            break;
          }
        }
        //if all the other points are left of the line, add this point
        if (isOK && (pPoints.at(k) != lConvexHull.back()) )
        {
          lConvexHull.push_back( pPoints.at(k) );
          firstPass = true;
          i++;
        }
        //update the new point of pPoints to test
        if (++k >= pPoints.size())
        {
          k=0;
        }
        //update the end condition, if we add at least 2 points
        if (lConvexHull.back() == pPoints.front())
        {
          sortie = firstPass;
        }
      }while ( !sortie );

//      for (unsigned int i=0; i<lConvexHull.size(); i++)
//      {
//        std::cout << "(" << lConvexHull.at(i).x << "f, " << lConvexHull.at(i).y << "f);" << std::endl;
//      }
      return lConvexHull;
    } // end getConvexHull


//    std::vector<AL::Math::Position2D> getConvexHull(
//      std::vector<AL::Math::Position2D>& pPointsInput)
//    {
//#ifdef DEBUG_CONVEX
//      std::cout.precision(25);
//      std::cout.setf(std::ios::fixed, std::ios::floatfield);
//      std::cout << "# data brut" << std::endl;
//      std::cout << "    pt0 = [" << std::endl;
//      for (unsigned i=0; i<pPointsInput.size(); i++)
//      {
//        std::cout << "[ " << pPointsInput.at(i).x << ", " << pPointsInput.at(i).y << " ]," << std::endl;
//      }
//      std::cout << "]" << std::endl;
//      std::cout << std::endl;
//#endif

//      std::vector<AL::Math::Position2D> lConvexHull;

//      float epsilon = 0.0001f;
//      AL::Math::deleteDoublesInNoneSortVector(pPointsInput, epsilon);
//      epsilon = 0.00001f;
//      // was AL::Math::deleteDoublesInSortVector(pPoints);
//      if (pPointsInput.size() < 3)
//      {
//        //std::cout<<"\nNumber of points insufficient\n";
//        return pPointsInput;
//      }
//      unsigned short i = 0;
//      unsigned short k = 1;
//      bool firstPass = false;
//      bool sortie    = false;

//      std::vector<AL::Math::Position2D> pPoints;
//      AL::Math::removeAlignedPoint(pPointsInput, pPoints);

//      // list of index of k where pPoints.at(k) is in a middle of a segment.
//      //std::vector<unsigned short> badIndex;

//      //sort the list of points
//      std::sort(pPoints.begin(), pPoints.end(), sortPredicate);

//#ifdef DEBUG_CONVEX
//      std::cout << "# data sort" << std::endl;
//      std::cout << "    pt1 = [" << std::endl;
//      for (unsigned i=0; i<pPoints.size(); i++)
//      {
//        std::cout << "[ " << pPoints.at(i).x << ", " << pPoints.at(i).y << " ]," << std::endl;
//      }
//      std::cout << "]" << std::endl;
//      std::cout << std::endl;
//      std::cout.precision(25);
//      std::cout.setf(std::ios::fixed, std::ios::floatfield);
//#endif
//      //push the leftmost point of pPoint in the convex hull
//      lConvexHull.push_back(pPoints.at(0));
//      //float epsilon = sinf(1.0f * TO_RAD);
//      //unsigned int nbIteration = 0;
//      do
//      {
//        bool isOK = true;
//        // if A, B and C are aligned and B is in the midlle of AC
//        //bool isInMiddleOfSegment = false;
//        //nbIteration++;
//        //The next point of the convex hull is the one such that all points of pPoints are left of
//        // the line defined by the last point in the convex hull and the point in pPoints we test
//        //for (itPoints = pPoints.begin(); itPoints != pPoints.end(); itPoints++)
//        //{
//        for (unsigned int id=0; id<pPoints.size(); id++)
//        {
////#ifdef DEBUG_CONVEX
////          std::cout << "==================" << std::endl;
////          std::cout << "point 0: " << lConvexHull.at(i).x << " " << lConvexHull.at(i).y << std::endl;
////          std::cout << "point 1: " << pPoints.at(k).x << " " << pPoints.at(k).y << std::endl;
////          std::cout << "point 2: " << (*itPoints).x << " " << (*itPoints).y << std::endl;
////          std::cout << "solution: " << AL::Math::isLeftOld(lConvexHull.at(i), pPoints.at(k), (*itPoints)) << std::endl;
////#endif
////          if (AL::Math::isLeft(
////                lConvexHull.at(i), // output lConvexHull
////                pPoints.at(k),     // input pPoint
////                (*itPoints),       // input pPoint
////                epsilon) < 0)      // epsilon
//          if (id != k)
//          {
//            // if one point is one the right, k is the wrong point
//            int result = AL::Math::isLeftBest(
//                  lConvexHull.at(i),   // output lConvexHull
//                  pPoints.at(k),       // input pPoint
//                  pPoints.at(id));

//            // -4: at least two points are equal
//            // -3: points are nearly aligned but B is not in the middle of A, C
//            // -2: points are aligned and B is in the middle of A, C
//            // -1: if point C is right of (AB)
//            // +1: if point C is left of (AB)

//            // pPoints.at(k) is in a middle of a segment
////            if (result == -2)
////            {
////              isInMiddleOfSegment = true;
////            }
//            // it is not the right point
//            if (result == -1)
//            {
//              isOK = false;
//              break;
//            }
//          }
//        } // end for

//        // Si pPoints.at(k) est potentiellement un point du convex
//        // Mais qu'il existe un autre point tel que pPoints.at(k) est au milieu
//        // Il faut le virer de la liste.
////        if (isOK && isInMiddleOfSegment)
////        {
////          badIndex.push_back(k);
////          isOK = false;
////        }

//        //if all the other points are left of the line, add this point
//        if (isOK && (!pPoints.at(k).isNear(lConvexHull.back(), epsilon)))
//        // was if (isOK && (pPoints.at(k) != lConvexHull.back()))
//        {
//          lConvexHull.push_back(pPoints.at(k));
//          firstPass = true;
//          i++;
//        }

//        //update the new point of pPoints to test
////        for (unsigned int id=0; id<pPoints.size(); id++)
////        {
//          k = k+1;
//          if (k >= pPoints.size())
//          {
//            k=0;
//            break;
//          }
////          if (!isIndexInList(k, badIndex))
////          {
////            break;
////          }
////        }

//        //update the end condition, if we add at least 2 points
//        if (lConvexHull.back().isNear(pPoints.front(), epsilon))
//        //was if (lConvexHull.back() == pPoints.front())
//        {
//          sortie = firstPass;
//        }
//      } while (!sortie); //|| (nbIteration>100));

////      if (nbIteration >100)
////      {
////        std::cout << "ALMATH: getConvexHull. number of iteration is to important." << std::endl;
////      }

//#ifdef DEBUG_CONVEX
//      std::cout << "# data convex hull" << std::endl;
//      std::cout << "    pt2 = [" << std::endl;
//      for (unsigned i=0; i<lConvexHull.size(); i++)
//      {
//        std::cout << "[ " << lConvexHull.at(i).x << ", " << lConvexHull.at(i).y << " ]," << std::endl;
//      }
//      std::cout << "]" << std::endl;
//      std::cout << std::endl;
//#endif
//      return lConvexHull;
//    } // end getConvexHull

    //    float distance(
    //      const Position2D& pPoint,
    //      const Segment&    pSegment)
    //    {
    //      /**
    //       * distance from C to [AB]
    //       *
    //       * if we compute:
    //       *
    //       * L ^ 2 = (Bx - Ax) ^ 2  + (By - Ay) ^ 2
    //       *
    //       * and
    //       *
    //       *     (Ay-Cy)(Ay-By)-#include <allog/allog.h>(Ax-Cx)(Bx-Ax)
    //       * r = -----------------------------
    //       *                 L^2
    //       *
    //       * Then the perpendicular projection P of C on AB is given by:
    //       *
    //       *
    //       * if r=0 -> P=A
    //       *    r=1 -> P=B
    //       *    r<0 -> P is on the backward extension of AB
    //       *    r>1 -> P is on the forward  extension of AB
    //       *
    //       *    0<r<1 ->
    //       *    P is given by:
    //       *
    //       *   Px = Ax + r(Bx-Ax)
    //       *   Py = Ay + r(By-Ay)
    //       *
    //       *   so if we force r to be between 0 and 1,
    //       *   the distance is Distance(P, C)
    //       */

    //      Position2D pPos1 = pSegment.start;
    //      Position2D pPos2 = pSegment.end;

    //      Position2D vect = pPos2 - pPos1;
    //      float oneOverLengthSquared = 1.0f  /
    //        (vect.x * vect.x + vect.y * vect.y);

    //      //Test if we're on the line segment
    //      float r = (
    //          (pPos1.y - pPoint.y) * (pPos1.y - pPos2.y)
    //        - (pPos1.x - pPoint.x) * (pPos2.x - pPos1.x)
    //        ) * oneOverLengthSquared;

    //      if (r > 0.0f)
    //      {
    //        r = 0.0f;
    //      }
    //      else if (r < -1.0f)
    //      {
    //        r = -1.0f;
    //      }

    //      Position2D proj;
    //      proj.x =  pPos1.x + r * (pPos2.x - pPos1.x);
    //      proj.y =  pPos1.y + r * (pPos2.y - pPos1.y);

    //      return distance(proj, pPoint);
    //    }


    //    Segment getClosestSegment(
    //      Position2D              pPoint,
    //      std::vector<Position2D> pPolygon)
    //    {
    //      // Build a sequence of segments from the sequence of points

    //      std::vector<Segment> lPoygon;

    //      for (unsigned int i = 0; i < pPolygon.size() - 1; i++)
    //      {
    //        Segment newSegment;
    //        newSegment.start = pPolygon[i];
    //        newSegment.end   = pPolygon[i+1];
    //        lPoygon.push_back(newSegment);
    //      }

    //      Segment lastSegment;
    //      lastSegment.start =  pPolygon[pPolygon.size()];
    //      lastSegment.end   =  pPolygon[0];

    //      lPoygon.push_back(lastSegment);

    //      float bestDistance = FLT_MAX;
    //      float distance;

    //      // find the closest point
    //      std::vector<Segment>::const_iterator segment_it;
    //      std::vector<Segment>::const_iterator bestSegment_it;
    //      std::vector<Segment>::const_iterator nextBestSegment_it;

    //      for (segment_it  = lPoygon.begin();
    //          segment_it != lPoygon.end();
    //          segment_it++)
    //      {
    //        distance = distanceSquared(pPoint,(*segment_it).end);
    //        if (distance < bestDistance)
    //        {
    //          bestDistance = distance;
    //          bestSegment_it = segment_it;
    //          if (segment_it != lPoygon.end() - 1)
    //          {
    //            nextBestSegment_it = segment_it + 1;
    //          }
    //          else
    //          {
    //            nextBestSegment_it = lPoygon.begin();
    //          }

    //        }
    //      }
    //      Position2D vect1 = (*bestSegment_it).start - (*bestSegment_it).end;
    //      Position2D vect2 = (*nextBestSegment_it).end - (*nextBestSegment_it).start;


    //      Position2D p1 = (*bestSegment_it).end + normalize(vect1) * 0.00001f;
    //      Position2D p2 = (*bestSegment_it).end + normalize(vect2) * 0.00001f;

    //      //find which is closer
    //      if ( distanceSquared(pPoint, p1) <= distanceSquared(pPoint, p2) )
    //      {
    //        return (*bestSegment_it);
    //      }
    //      else
    //      {
    //        return (*nextBestSegment_it);
    //      }
    //    } // end getClosestSegment

    //    float getDirection(
    //      const AL::Math::Position2D& p0,
    //      const AL::Math::Position2D& p1,
    //      const AL::Math::Position2D& p2)
    //    {
    //        return   ((p0.x - p1.x) * (p2.y - p1.y))
    //               - ((p2.x - p1.x) * (p0.y - p1.y));
    //    }

  } // namespace Math
} // namespace AL

