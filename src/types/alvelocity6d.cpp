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


    bool Velocity6D::operator== (const Velocity6D& pVel2) const
    {
      if(
          (xd == pVel2.xd) &&
          (yd == pVel2.yd) &&
          (zd == pVel2.zd) &&
          (wxd == pVel2.wxd) &&
          (wyd == pVel2.wyd) &&
          (wzd == pVel2.wzd)
          )
      {
        return true;
      }
      else
      {
        return false;
      }
    }


    bool Velocity6D::operator!= (const Velocity6D& pVel2) const
    {
      return !(*this==pVel2);
    }


    Velocity6D& Velocity6D::operator*= (const float pVal)
    {
      xd  *= pVal;
      yd  *= pVal;
      zd  *= pVal;
      wxd *= pVal;
      wyd *= pVal;
      wzd *= pVal;

      return *this;
    }


    Velocity6D& Velocity6D::operator/= (const float pVal)
    {
      if (pVal == 0.0f)
      {
        throw std::runtime_error(
          "ALVelocity6D: operator/= Division by zeros.");
      }
      (*this) *= (1.0f/pVal);
      return *this;
    }

    bool Velocity6D::isNear(
      const Velocity6D& pVel2,
      const float&      pEpsilon) const
    {
      if (
        (fabsf(xd  - pVel2.xd)  > pEpsilon) ||
        (fabsf(yd  - pVel2.yd)  > pEpsilon) ||
        (fabsf(zd  - pVel2.zd)  > pEpsilon) ||
        (fabsf(wxd - pVel2.wxd) > pEpsilon) ||
        (fabsf(wyd - pVel2.wyd) > pEpsilon) ||
        (fabsf(wzd - pVel2.wzd) > pEpsilon))
      {
        return false;
      }
      else
      {
        return true;
      }
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


    float norm(const Velocity6D& pVel)
    {
      // norm of a 6 component vector
      return sqrtf( (pVel.xd*pVel.xd) +
                    (pVel.yd*pVel.yd) +
                    (pVel.zd*pVel.zd) +
                    (pVel.wxd*pVel.wxd) +
                    (pVel.wyd*pVel.wyd) +
                    (pVel.wzd*pVel.wzd));
    }


    Velocity6D normalize(const Velocity6D& pVel)
    {
      Velocity6D ret;
      ret = pVel;
      float tmpNorm = norm(pVel);

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

