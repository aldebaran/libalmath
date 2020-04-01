#pragma once
#ifndef LIB_ALMATH_GEOMETRICS_SHAPE_H
#define LIB_ALMATH_GEOMETRICS_SHAPE_H

#include <almath/api.h>
#include <iostream>
#include <memory>

namespace AL {
namespace Math {

class ShapePrivate;

class ALMATH_API Shape {
  std::shared_ptr<const ShapePrivate> _p;
public:
  bool isInside(double x, double y) const;

  // JSON format grammar
  //
  //  <Shape> ::= '"Plane"' | '{"Polygon": ['<List_Points>']}'
  //                        | '{ "Union": ['<List_Shapes>']}'
  //  <Point> ::= '['<number>', '<number>']'
  //  <List_Points> ::= '' | <Point> | <Point>', '<List_Points>
  //  <List_Shapes> ::= '' | <Shape> | <Shape>', '<List_Shapes>
  //
  // JSON format examples
  //
  // * "Plane"
  // * {"Polygon": [[0, 0], [1.0, 0], [0, 0.5]]}
  // * {"Union": [{"Polygon": [[0, 0], [1, 0], [0, 1.0]]},
  //              {"Polygon": [[0, 0], [-1, 0], [0, -1.0]]}]}
  //
  // will throw if a polygon is non-simple or non-convex.
  static Shape parseFromJSON(std::istream &);

  friend ALMATH_API
  void writeAsJSON(std::ostream &os, const Shape &shape);

  // warning: identical polygons may be reported as not equal
  // if their description is different.
  friend ALMATH_API
  bool operator==(const Shape &lhs, const Shape &rhs);
};
void writeAsJSON(std::ostream &os, const Shape &shape);
}
}
#endif
