/**
 * @author Nicolas Garcia - ngarcia@aldebaran.com
 * Aldebaran (c) 2014 All Rights Reserved
 *
 */

#include <almath/geometrics/shapes3d.h>

namespace AL {
namespace Math {

Sphere::Sphere(float pRadius) : fRadius(pRadius) {}

void Sphere::accept(const Shape3DVisitor &v) const { v.visit(*this); }

float Sphere::getRadius() const { return fRadius; }

RoundedRectangle::RoundedRectangle(float pHalfExtentX, float pHalfExtentY,
                                   float pRadius)
    : fHalfExtentX(pHalfExtentX),
      fHalfExtentY(pHalfExtentY),
      fRadius(pRadius) {}

void RoundedRectangle::accept(const Shape3DVisitor &v) const { v.visit(*this); }

float RoundedRectangle::getHalfExtentX() const { return fHalfExtentX; }

float RoundedRectangle::getHalfExtentY() const { return fHalfExtentY; }

float RoundedRectangle::getRadius() const { return fRadius; }

void Plane::accept(const Shape3DVisitor &v) const { v.visit(*this); }

Pill::Pill(float pHalfExtent, float pRadius)
    : fHalfExtent(pHalfExtent), fRadius(pRadius) {}

void Pill::accept(const AL::Math::Shape3DVisitor &v) const { v.visit(*this); }

float Pill::getHalfExtent() const { return fHalfExtent; }

float Pill::getRadius() const { return fRadius; }

void HalfSpace::accept(const Shape3DVisitor &v) const { v.visit(*this); }

Rectangle::Rectangle(float pHalfExtentX, float pHalfExtentY)
    : fHalfExtentX(pHalfExtentX), fHalfExtentY(pHalfExtentY) {}

void Rectangle::accept(const Shape3DVisitor &v) const { v.visit(*this); }

float Rectangle::getHalfExtentX() const { return fHalfExtentX; }

float Rectangle::getHalfExtentY() const { return fHalfExtentY; }

void HalfLine::accept(const Shape3DVisitor &v) const { v.visit(*this); }

}  // End namespace Math.
}  // End namespace AL.
