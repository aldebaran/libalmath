/**
 * @author Nicolas Garcia - ngarcia@aldebaran-robotics.com
 * Aldebaran Robotics (c) 2014 All Rights Reserved
 *
 */

#ifndef LIB_ALMATH_GEOMETRICS_SHAPES_H
#define LIB_ALMATH_GEOMETRICS_SHAPES_H

#include <almath/api.h>
#include <string>
#include <stdexcept>

namespace AL {
namespace Math {

class ALMATH_API Shape3DVisitor;

class ALMATH_API Shape3D {
 public:
  virtual ~Shape3D() {}
  virtual void accept(const Shape3DVisitor &v) const = 0;
};

// Defined by its radius. Centered at (0, 0, 0).
class ALMATH_API Sphere : public Shape3D {
 public:
  Sphere(float pRadius);
  friend bool operator==(const Sphere &lhs, const Sphere &rhs)
  {
    return lhs.fRadius == rhs.fRadius;
  }
  friend bool operator!=(const Sphere &lhs, const Sphere &rhs)
  {
    return !(lhs == rhs);
  }
  virtual void accept(const Shape3DVisitor &v) const;
  float getRadius() const;

 private:
  float fRadius;
};

// Minkowsky sum of a 2D rectangle with a 3D sphere.
// Centered. The normal to the plane is : (0, 0, 1).
// a.k.a Tab before (like a tactile tablet).
class ALMATH_API RoundedRectangle : public Shape3D {
 public:
  RoundedRectangle(float pHalfExtentX, float pHalfExtentY, float pRadius);
  friend bool operator==(const RoundedRectangle &lhs,
                         const RoundedRectangle &rhs)
  {
    return lhs.fHalfExtentX == rhs.fHalfExtentX &&
           lhs.fHalfExtentY == rhs.fHalfExtentY &&
           lhs.fRadius == rhs.fRadius;
  }
  friend bool operator!=(const RoundedRectangle &lhs,
                         const RoundedRectangle &rhs)
  {
    return !(lhs == rhs);
  }
  virtual void accept(const Shape3DVisitor &v) const;
  float getHalfExtentX() const;
  float getHalfExtentY() const;
  float getRadius() const;

 private:
  float fHalfExtentX;
  float fHalfExtentY;
  float fRadius;
};

// 3D convex hull of two spheres, oriented along z axis.
// The centers of the spheres have the coordinates in pill frame :
// upSphere(0, 0, fHalfExtent), downSpere(0, 0, -fhalfExtent).
class ALMATH_API Pill : public Shape3D {
 public:
  Pill(float pHalfExtent, float pRadius);
  friend bool operator==(const Pill &lhs, const Pill &rhs)
  {
    return lhs.fHalfExtent == rhs.fHalfExtent &&
           lhs.fRadius == rhs.fRadius;
  }
  friend bool operator!=(const Pill &lhs, const Pill &rhs)
  {
    return !(lhs == rhs);
  }
  float getHalfExtent() const;
  float getRadius() const;
  virtual void accept(const Shape3DVisitor &v) const;

 private:
  float fHalfExtent;
  float fRadius;

};



// Equation z = 0, normal (0, 0, 1).
class ALMATH_API Plane : public Shape3D {
 public:
  friend bool operator==(const Plane &lhs, const Plane &rhs)
  {
    return true;
  }
  friend bool operator!=(const Plane &lhs, const Plane &rhs)
  {
    return false;
  }
  virtual void accept(const Shape3DVisitor &v) const;
};

// Equation z <= 0, normal (0, 0, 1). (z > 0 is free space).
class ALMATH_API HalfSpace : public Shape3D {
 public:
  friend bool operator==(const HalfSpace &lhs, const HalfSpace &rhs)
  {
    return true;
  }
  friend bool operator!=(const HalfSpace &lhs, const HalfSpace &rhs)
  {
    return false;
  }
  virtual void accept(const Shape3DVisitor &v) const;
};

// Equation z = 0, abs(x) < fHalfExtentX, abs(y) < fHalfExtentY,
class ALMATH_API Rectangle : public Shape3D {
 public:
  Rectangle(float pHalfExtentX, float pHalfExtentY);
  friend bool operator==(const Rectangle &lhs, const Rectangle &rhs)
  {
    return lhs.fHalfExtentX == rhs.fHalfExtentX &&
           lhs.fHalfExtentY == rhs.fHalfExtentY;
  }
  friend bool operator!=(const Rectangle &lhs, const Rectangle &rhs)
  {
    return !(lhs == rhs);
  }
  virtual void accept(const Shape3DVisitor &v) const;
  float getHalfExtentX() const;
  float getHalfExtentY() const;

 private:
  float fHalfExtentX;
  float fHalfExtentY;
};

// Half line oriented along z axis, equation z >= 0, x = 0, y = 0.
class ALMATH_API HalfLine : public Shape3D {
 public:
  friend bool operator==(const HalfLine &lhs, const HalfLine &rhs)
  {
    return true;
  }
  friend bool operator!=(const HalfLine &lhs, const HalfLine &rhs)
  {
    return false;
  }
  virtual void accept(const Shape3DVisitor &v) const;
};

class ALMATH_API Shape3DVisitor {
 public:
  virtual void visit(const Pill &pShape) const = 0;
  virtual void visit(const Sphere &pShape) const = 0;
  virtual void visit(const RoundedRectangle &pShape) const = 0;
  virtual void visit(const Plane &pShape) const = 0;
  virtual void visit(const HalfSpace &pShape) const = 0;
  virtual void visit(const Rectangle &pShape) const = 0;
  virtual void visit(const HalfLine &pShape) const = 0;
  virtual ~Shape3DVisitor() {}
};

class ALMATH_API NotImplementedShape3DVisitor : public Shape3DVisitor {
 public:
  NotImplementedShape3DVisitor(const std::string msg = "not implemented")
      : fMsg(msg) {}
  virtual void visit(const Pill &pShape) const {
    throw std::runtime_error(fMsg);
  }
  virtual void visit(const Sphere &pShape) const {
    throw std::runtime_error(fMsg);
  }
  virtual void visit(const RoundedRectangle &pShape) const {
    throw std::runtime_error(fMsg);
  }
  virtual void visit(const Plane &pShape) const {
    throw std::runtime_error(fMsg);
  }
  virtual void visit(const HalfSpace &pShape) const {
    throw std::runtime_error(fMsg);
  }
  virtual void visit(const Rectangle &pShape) const {
    throw std::runtime_error(fMsg);
  }
  virtual void visit(const HalfLine &pShape) const {
    throw std::runtime_error(fMsg);
  }
  virtual ~NotImplementedShape3DVisitor() {}

 private:
  std::string fMsg;
};

}  // End namespace Math.
}  // End namespace AL.

#endif  // LIB_ALMATH_GEOMETRICS_SHAPE_H
