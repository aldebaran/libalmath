/**
* @author Chris Kilner
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved
*/

#include <almath/types/alposition2d.h>

#include "math.h"

#include <alcore/alerror.h>

namespace AL {
  namespace Math {

    Position2D Position2D::operator+ (const Position2D& pPos2) const
    {
      Position2D res;
      res.x = x + pPos2.x;
      res.y = y + pPos2.y;
      return res;
    }

    Position2D Position2D::operator- (const Position2D& pPos2) const
    {
      Position2D res;
      res.x = x - pPos2.x;
      res.y = y - pPos2.y;
      return res;
    }

    Position2D Position2D::operator+ () const
    {
      Position2D res;
      res.x = x;
      res.y = y;
      return res;
    }

    Position2D Position2D::operator- () const
    {
      Position2D res;
      res.x = -x;
      res.y = -y;
      return res;
    }

    Position2D& Position2D::operator+= (const Position2D& pPos2)
    {
      x += pPos2.x;
      y += pPos2.y;
      return *this;
    }

    Position2D& Position2D::operator-= (const Position2D& pPos2)
    {
      x -= pPos2.x;
      y -= pPos2.y;
      return *this;
    }


    bool Position2D::operator!=(const Position2D& pPos2) const
    {
      return !(*this==pPos2);
    }


    bool Position2D::isNear(
      const Position2D& pPos,
      const float&      pEpsilon) const
    {
      if (
        (fabsf(x - pPos.x) > pEpsilon) ||
        (fabsf(y - pPos.y) > pEpsilon))
      {
        return false;
      }
      else
      {
        return true;
      }
    }


    Position2D Position2D::operator* (float pM) const
    {
      Position2D res;
      res.x = x * pM;
      res.y = y * pM;
      return res;
    }

    Position2D operator* (
      const float       pM,
      const Position2D& pPos1)
    {
      return pPos1*pM;
    }

    Position2D Position2D::operator/ (float pM) const
    {
      if (pM == 0.0f)
      {
        throw ALERROR(
          "ALPosition2D",
          "operator/",
          "Division by zeros.");
      }
      return *this * (1.0f/pM);
    }

    Position2D& Position2D::operator*= (float pM)
    {
      x *=pM;
      y *=pM;
      return *this;
    }

    Position2D& Position2D::operator/= (float pM)
    {
      if (pM == 0.0f)
      {
        throw ALERROR(
          "ALPosition2D",
          "operator/=",
          "Division by zeros.");
      }
      *this *= (1.0f/pM);
      return *this;
    }


    float Position2D::distanceSquared(const Position2D& pPos) const {
      return Math::distanceSquared(*this, pPos);
    }

    float Position2D::distance(const Position2D& pPos) const {
      return Math::distance(*this, pPos);
    }

    float Position2D::norm() const {
      return Math::norm(*this);
    }

    Position2D Position2D::normalize() const {
      return Math::normalize(*this);
    }

    float Position2D::crossProduct(const Position2D& pPos) const {
      return Math::crossProduct(*this, pPos);
    }

    bool Position2D::operator==(const Position2D& pPos) const
    {
      if (x == pPos.x && y == pPos.y)
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    std::vector<float> Position2D::toVector() const
    {
      std::vector<float> returnVector;
      returnVector.resize(2);
      returnVector[0] = x;
      returnVector[1] = y;

      return returnVector;
    }


    float distanceSquared(
      const Position2D& pPos1,
      const Position2D& pPos2)
    {
      return (pPos1.x-pPos2.x)*(pPos1.x-pPos2.x)+(pPos1.y-pPos2.y)*(pPos1.y-pPos2.y);
    }

    float distance(
      const Position2D& pPos1,
      const Position2D& pPos2)
    {
      return sqrtf(distanceSquared(pPos1, pPos2));
    }

    float norm(const Position2D& p)
    {
      return sqrtf( (p.x*p.x) + (p.y*p.y) );
    }

    Position2D normalize(const Position2D& p)
    {
      Position2D ret;
      ret = p;

      float tmpNorm = norm(p);
      if (tmpNorm == 0.0f)
      {
        throw ALERROR(
          "ALPosition2D",
          "normalize",
          "Division by zeros.");
      }

      ret /= tmpNorm;
      return ret;
    }

    float crossProduct(
      const Position2D& pPos1,
      const Position2D& pPos2)
    {
      return (pPos1.x*pPos2.y - pPos1.y*pPos2.x);
    }


    void crossProduct(
      const Position2D& pPos1,
      const Position2D& pPos2,
      float&            result)
    {
      result = (pPos1.x*pPos2.y - pPos1.y*pPos2.x);
    }

  } // end namespace math
} // end namespace al

