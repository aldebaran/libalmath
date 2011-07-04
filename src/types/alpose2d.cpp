/**
* @author Chris Kilner and Cyrille Collette
* Copyright (c) Aldebaran Robotics 2011 All Rights Reserved
*/

#include <almath/types/alpose2d.h>
#include <cmath>

namespace AL {
  namespace Math {

    Pose2D Pose2D::operator+ (const Pose2D& pPos2) const
    {
      Pose2D res;
      res.x = x + pPos2.x;
      res.y = y + pPos2.y;
      res.theta = theta + pPos2.theta;
      return res;
    }

    Pose2D Pose2D::operator- (const Pose2D& pPos2) const
    {
      Pose2D res;
      res.x = x - pPos2.x;
      res.y = y - pPos2.y;
      res.theta = theta - pPos2.theta;
      return res;
    }

    Pose2D Pose2D::operator+ () const
    {
      Pose2D res;
      res.x = x;
      res.y = y;
      res.theta = theta;
      return res;
    }

    Pose2D Pose2D::operator- () const
    {
      Pose2D res;
      res.x = -x;
      res.y = -y;
      res.theta = -theta;
      return res;
    }

    Pose2D Pose2D::operator* (const Pose2D& pPos2) const
    {
      Pose2D pOut;
      pOut.x = x + cosf(theta) * pPos2.x - sinf(theta) * pPos2.y;
      pOut.y = y + sinf(theta) * pPos2.x + cosf(theta) * pPos2.y;
      pOut.theta = theta + pPos2.theta;

      return pOut;
    }

    Pose2D& Pose2D::operator*= (const Pose2D& pPos2)
    {
      x += cosf(theta) * pPos2.x - sinf(theta) * pPos2.y;
      y += sinf(theta) * pPos2.x + cosf(theta) * pPos2.y;
      theta += pPos2.theta;

      return *this;
    }

    Pose2D& Pose2D::operator+= (const Pose2D& pPos2)
    {
      x     += pPos2.x;
      y     += pPos2.y;
      theta += pPos2.theta;
      return *this;
    }

    Pose2D& Pose2D::operator-= (const Pose2D& pPos2)
    {
      x     -= pPos2.x;
      y     -= pPos2.y;
      theta -= pPos2.theta;
      return *this;
    }

    bool Pose2D::operator==(const Pose2D& pPos2) const
    {
       if (
         (x == pPos2.x) &&
         (y == pPos2.y) &&
         (theta == pPos2.theta) )
       {
        return true;
      }
      else
      {
        return false;
      }
    }

    bool Pose2D::operator!=(const Pose2D& pPos2) const
    {
      return ! (*this==pPos2);
    }

    float Pose2D::distanceSquared(const Pose2D& pPos) const
    {
      return Math::distanceSquared(*this, pPos);
    }

    float Pose2D::distance(const Pose2D& pPos2) const
    {
      return Math::distance(*this, pPos2);
    }

    std::vector<float> Pose2D::toVector() const
    {
      std::vector<float> returnVector;
      returnVector.resize(3);
      returnVector[0] = x;
      returnVector[1] = y;
      returnVector[2] = theta;

      return returnVector;
    }

    float distanceSquared(
      const Pose2D& pPos1,
      const Pose2D& pPos2)
    {
      return (pPos1.x-pPos2.x)*(pPos1.x-pPos2.x)+(pPos1.y-pPos2.y)*(pPos1.y-pPos2.y);
    }

    float distance(
      const Pose2D& pPos1,
      const Pose2D& pPos2)
    {
      return sqrtf(distanceSquared(pPos1, pPos2));
    }


    bool Pose2D::isNear(
      const Pose2D& pPos2,
      const float&     pEpsilon) const
    {

      if (
        (fabsf(x - pPos2.x) > pEpsilon) ||
        (fabsf(y - pPos2.y) > pEpsilon) ||
        (fabsf(theta - pPos2.theta) > pEpsilon))
      {
        return false;
      }
      else
      {
        return true;
      }
    }

    Pose2D Pose2D::inverse() const
    {
      return Math::Pose2DInverse(*this);
    }

    void Pose2DInverse(
      const Pose2D& pIn,
      Pose2D&       pOut)
    {
      pOut.theta = -pIn.theta;

      float cos = cosf(pOut.theta);
      float sin = sinf(pOut.theta);

      pOut.x = -( pIn.x*cos - pIn.y*sin);
      pOut.y = -( pIn.y*cos + pIn.x*sin);
    }

    Pose2D Pose2DInverse(const Pose2D& pIn)
    {
      Pose2D pOut;
      Pose2DInverse(pIn, pOut);
      return pOut;
    }
  } // end namespace math
} // end namespace AL
