/**
* @author Chris Kilner
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved
*/

#include <almath/types/alvelocity6d.h>
#include <cmath>
#include <stdexcept>

namespace AL {
  namespace Math {

    Velocity6D Velocity6D::operator+ (const Velocity6D& pVel2) const
    {
      Velocity6D res;
      res.xd  = xd  + pVel2.xd;
      res.yd  = yd  + pVel2.yd;
      res.zd  = zd  + pVel2.zd;
      res.wxd = wxd + pVel2.wxd;
      res.wyd = wyd + pVel2.wyd;
      res.wzd = wzd + pVel2.wzd;
      return res;
    }

    Velocity6D Velocity6D::operator- (const Velocity6D& pVel2) const
    {
      Velocity6D res;
      res.xd  = xd  - pVel2.xd;
      res.yd  = yd  - pVel2.yd;
      res.zd  = zd  - pVel2.zd;
      res.wxd = wxd - pVel2.wxd;
      res.wyd = wyd - pVel2.wyd;
      res.wzd = wzd - pVel2.wzd;
      return res;
    }

    Velocity6D Velocity6D::operator+ () const
    {
      Velocity6D res;
      res.xd  = xd;
      res.yd  = yd;
      res.zd  = zd;
      res.wxd = wxd;
      res.wyd = wyd;
      res.wzd = wzd;
      return res;
    }

    Velocity6D Velocity6D::operator- () const
    {
      Velocity6D res;
      res.xd  = -xd;
      res.yd  = -yd;
      res.zd  = -zd;
      res.wxd = -wxd;
      res.wyd = -wyd;
      res.wzd = -wzd;
      return res;
    }

    bool Velocity6D::isNear(
      const Velocity6D& pVel,
      const float&      pEpsilon) const
    {
      if (
        (fabsf(xd  - pVel.xd)  > pEpsilon) ||
        (fabsf(yd  - pVel.yd)  > pEpsilon) ||
        (fabsf(zd  - pVel.zd)  > pEpsilon) ||
        (fabsf(wxd - pVel.wxd) > pEpsilon) ||
        (fabsf(wyd - pVel.wyd) > pEpsilon) ||
        (fabsf(wzd - pVel.wzd) > pEpsilon))
      {
        return false;
      }
      else
      {
        return true;
      }
    }


    Velocity6D Velocity6D::operator* (const float pVal) const
    {
      Velocity6D res;
      res.xd  = xd * pVal;
      res.yd  = yd * pVal;
      res.zd  = zd * pVal;
      res.wxd = wxd * pVal;
      res.wyd = wyd * pVal;
      res.wzd = wzd * pVal;
      return res;
    }


    Velocity6D Velocity6D::operator/ (const float pVal) const
    {
      if (pVal == 0.0f)
      {
        throw std::runtime_error(
          "ALVelocity6D: operator/ Division by zeros.");
      }
      return  (*this) * (1.0f/pVal);
    }


    Velocity6D& Velocity6D::operator*= (const float pM)
    {
      xd  *= pM;
      yd  *= pM;
      zd  *= pM;
      wxd *= pM;
      wyd *= pM;
      wzd *= pM;

      return *this;
    }


    Velocity6D& Velocity6D::operator/= (const float pM)
    {
      if (pM == 0.0f)
      {
        throw std::runtime_error(
          "ALVelocity6D: operator/= Division by zeros.");
      }
      (*this) *= (1.0f/pM);
      return *this;
    }

    float Velocity6D::norm() const
    {
      return Math::norm(*this);
    }

    Velocity6D Velocity6D::normalize() const
    {
      return Math::normalize(*this);
    }

    std::vector<float> Velocity6D::toVector() const
    {
      std::vector<float> returnVector;
      returnVector.resize(6);

      returnVector[0] = xd;
      returnVector[1] = yd;
      returnVector[2] = zd;

      returnVector[3] = wxd;
      returnVector[4] = wyd;
      returnVector[5] = wzd;

      return returnVector;
    }


    Velocity6D operator* (
      const float       pVal,
      const Velocity6D& pVel)
    {
      return pVel * pVal;
    }


    float norm(const Velocity6D& p)
    {
      // norm of a 6 component vector
      return sqrtf( (p.xd*p.xd) +
                    (p.yd*p.yd) +
                    (p.zd*p.zd) +
                    (p.wxd*p.wxd) +
                    (p.wyd*p.wyd) +
                    (p.wzd*p.wzd));
    }


    Velocity6D normalize(const Velocity6D& p)
    {
      Velocity6D ret;
      ret = p;
      float tmpNorm = norm(p);

      if (tmpNorm == 0.0f)
      {
        throw std::runtime_error(
          "ALVelocity6D: normalize Division by zeros.");
      }

      ret /= tmpNorm;
      return ret;
    }

  } // end namespace Math
} // end namespace AL

