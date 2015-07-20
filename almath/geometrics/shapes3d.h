/**
 * @author Nicolas Garcia - ngarcia@aldebaran-robotics.com
 * Aldebaran Robotics (c) 2014 All Rights Reserved
 *
 */

#ifndef LIB_ALMATH_GEOMETRICS_SHAPES_H
#define LIB_ALMATH_GEOMETRICS_SHAPES_H

#include <string>
#include <stdexcept>

namespace AL {
namespace Math {

class Shape3DVisitor;

class Shape3D {
 public:
  virtual ~Shape3D() {}
  virtual void accept(const Shape3DVisitor &v) const = 0;
};

// Defined by its radius. Centered at (0, 0, 0).
class Sphere : public Shape3D {
 public:
  Sphere(float pRadius);
  virtual void accept(const Shape3DVisitor &v) const;
  float getRadius() const;

 private:
  float fRadius;
};

// Minkowsky sum of a 2D rectangle with a 3D sphere.
// Centered. The normal to the plane is : (0, 0, 1).
// a.k.a Tab before (like a tactile tablet).
class RoundedRectangle : public Shape3D {
 public:
  RoundedRectangle(float pHalfExtentX, float pHalfExtentY, float pRadius);
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
class Pill : public Shape3D {
 public:
  Pill(float pHalfExtent, float pRadius);
  virtual void accept(const Shape3DVisitor &v) const;
  float getHalfExtent() const;
  float getRadius() const;

 private:
  float fHalfExtent;
  float fRadius;
};

// Equation z = 0, normal (0, 0, 1).
class Plane : public Shape3D {
 public:
  virtual void accept(const Shape3DVisitor &v) const;
};

// Equation z <= 0, normal (0, 0, 1). (z > 0 is free space).
class HalfSpace : public Shape3D {
 public:
  virtual void accept(const Shape3DVisitor &v) const;
};

// Equation z = 0, abs(x) < fHalfExtentX, abs(y) < fHalfExtentY,
class Rectangle : public Shape3D {
 public:
  Rectangle(float pHalfExtentX, float pHalfExtentY);
  virtual void accept(const Shape3DVisitor &v) const;
  float getHalfExtentX() const;
  float getHalfExtentY() const;

 private:
  float fHalfExtentX;
  float fHalfExtentY;
};

// Half line oriented along z axis, equation z >= 0, x = 0, y = 0.
class HalfLine : public Shape3D {
 public:
  virtual void accept(const Shape3DVisitor &v) const;
};

class Shape3DVisitor {
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

class NotImplementedShape3DVisitor : public Shape3DVisitor {
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
