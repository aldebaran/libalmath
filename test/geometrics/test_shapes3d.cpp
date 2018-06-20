/**
 * @author Nicolas Garcia - ngarcia@aldebaran-robotics.com
 * Aldebaran Robotics (c) 2014 All Rights Reserved
 */

#include <memory>
#include <gtest/gtest.h>
#include <almath/geometrics/shapes3d.h>

using namespace AL;
using namespace Math;

class DummyVisitor : public Shape3DVisitor {
 public:
  void visit(const Pill &s) const { fVisited = "Pill"; }

  void visit(const Sphere &s) const { fVisited = "Sphere"; }

  void visit(const RoundedRectangle &s) const { fVisited = "RoundedRectangle"; }

  void visit(const Plane &s) const { fVisited = "Plane"; }

  void visit(const HalfSpace &s) const { fVisited = "HalfSpace"; }

  void visit(const Rectangle &s) const { fVisited = "Rectangle"; }

  void visit(const HalfLine &s) const { fVisited = "HalfLine"; }

  std::string getVisited() const { return fVisited; }

 private:
  mutable std::string fVisited;
};

TEST(Shape3DTest, dummyVisitor) {
  DummyVisitor visitor;

  std::unique_ptr<Shape3D> pill(new Pill(0.f, 0.f));
  pill->accept(visitor);
  EXPECT_EQ("Pill", visitor.getVisited());

  std::unique_ptr<Shape3D> sphere(new Sphere(0.f));
  sphere->accept(visitor);
  EXPECT_EQ("Sphere", visitor.getVisited());

  std::unique_ptr<Shape3D> roundedRectangle(
      new RoundedRectangle(0.f, 0.f, 0.f));
  roundedRectangle->accept(visitor);
  EXPECT_EQ("RoundedRectangle", visitor.getVisited());

  std::unique_ptr<Shape3D> plane(new Plane());
  plane->accept(visitor);
  EXPECT_EQ("Plane", visitor.getVisited());

  std::unique_ptr<Shape3D> halfSpace(new HalfSpace());
  halfSpace->accept(visitor);
  EXPECT_EQ("HalfSpace", visitor.getVisited());

  std::unique_ptr<Shape3D> rectangle(new Rectangle(0.0f, 0.0f));
  rectangle->accept(visitor);
  EXPECT_EQ("Rectangle", visitor.getVisited());

  std::unique_ptr<Shape3D> halfLine(new HalfLine());
  halfLine->accept(visitor);
  EXPECT_EQ("HalfLine", visitor.getVisited());
}
