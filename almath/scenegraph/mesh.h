/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */

#ifndef LIB_ALMATH_SCENEGRAPH_MESH_H
#define LIB_ALMATH_SCENEGRAPH_MESH_H

#include <almath/api.h>
#include <boost/optional.hpp>
#include <string>
#include <vector>

namespace AL {

// Mesh describe a mesh using a set of polygons using in buffers similar to
// Collada "polylist".
//
// The mesh is described using up to 6 arrays
// * a buffer of vertices positions coordinates: positions()
// * a buffer of vertices normals coordinates: normals()
// * optionally a buffer of vertices texture coordinates: texCoords()
// * a buffer of vertices which defines a vertex as a 2/3-tuple of indexes
//   referring to the positions, normals and (optionally) texCoords
//   buffers: vertices()
// * a buffer of polygons vertices counts (number of vertices per polygon),
//   defining how the vertices are groupes per polygon: polygonVerticesCounts()
//
// The face winding is counter-clockwise.
// For instance, for the triangle defined as
//
//   Mesh m;
//   m.normal(0, 0, 1);
//   m.begin(Mesh::TRIANGLES);
//   m.vertex(0, 0.6, 0); // vertex 0
//   m.vertex(-0.5, 0, 2); // vertex 1
//   m.vertex(0, -0.6, 2, // vertex 2
//   m.end();
//
// The 3 vertices are in the order 0->1->2, and the faces are:
//
// interior       exterior
// (invisible)    (visible)
// side           side
//  0               0
//  |\             /|
//  | 1           1 |
//  |/             \|
//  2               2
//
//
// For instance, let see how the following quadrilateron could be described
//
//    0     ^ y
//   /|\    |
//  1 | 3   +--> x
//   \|/
//    2
//
// vertice 0 is at position (0, 0.6, 2)
// vertice 1 is at position (-0.5, 0, 2)
// vertice 2 is at position (0, -0.6, 2)
// vertice 3 is at position (0.5, 0, 2)
// All three vertices have the same normal (0, 0, 1)
//
// m.positions():
// { 0, 0.6, 2,
//  -0.5, 0, 2,
//   0, -0.6, 2,
//   0.5, 0, 2}
// m.normals():
// {0, 0, 1}
//
// If described as one quadrilateron:
// m.vertices():
// {0, 0,   1, 0,   2, 0,   3, 0}
// m.polygonVerticesCounts():
// {4}
//
// If described as two triangles:
// m.vertices():
// {0, 0,   1, 0,   2, 0,   0, 0,   2, 0,   3, 0}
// m.polygonVerticesCounts():
// {3, 3}
//
//
// The data sctructures were loosely adapted from:
// http://www.ics.com/blog/qt-and-opengl-part-1-loading-3d-model-open-asset-import-library-assimp
class ALMATH_API Mesh {
 public:
  Mesh(bool withTexCoords = false);
  bool withTexCoords() const;

  // number of indexes per vertex in the vertices() buffer
  // 2 or 3 depending on withTexCoords()
  size_t verticesStride() const;

  // offset for the vertex position index within a vertex indices 2/3-tuple
  static const size_t positionOffset = 0;
  // offset for the vertex normal index within a vertex indices 2/3-tuple
  static const size_t normalOffset = 1;
  // offset for the vertex texCoord index within a vertex indices 3-tuple
  static const size_t texCoordOffset = 2;

  const std::vector<float> &positions() const;
  size_t positionsNb() const;
  float const *positionPtrAt(size_t index) const;

  const std::vector<float> &normals() const;
  size_t normalsNb() const;
  float const *normalPtrAt(size_t index) const;

  const std::vector<float> &texCoords() const;
  size_t texCoordsNb() const;
  float const *texCoordPtrAt(size_t index) const;

  const std::vector<size_t> &vertices() const;
  size_t verticesNb() const;
  size_t const *vertexPtrAt(size_t index) const;

  const std::vector<size_t> &polygonVerticesCounts() const;
  size_t polygonsNb() const;
  size_t polygonVerticesCountAt(size_t index) const;

  // the following members are useful to build a mesh

  enum Mode { POLYGON, TRIANGLES, QUADS };
  static const size_t NO_INDEX;
  // Add a vertex normal and declare it as the current one. Return its index.
  // Modeled after OpenGl's glNormal.
  size_t normal(float x, float y, float z);
  // Declare the normal at index as the current one.
  // Throw if the index is out range.
  // Modeled after OpenGl's glNormal3v.
  void normal(size_t index);

  // Add a texture coordinate and declare it as the current one. Return its
  // index.
  // Modeled after OpenGl's glTexCoord.
  size_t texCoord(float u, float v);
  // Declare the texture coordinate at index as the current one.
  // Throw if the index is out range.
  // Modeled after OpenGl's glTexCoord3v.
  void texCoord(size_t index);

  // Add a vertex position. Return its index.
  size_t position(float x, float y, float z);

  // Begin a new polygon (if mode is POLYGON) or a new set of triangles or
  // quads (if mode is TRIANGES or QUADS, respectively).
  // Throw if called between begin()/end() calls.
  void begin(Mode mode);

  // create a new vertex using positionIndex and the current normal and
  // texture coordinate indexes.
  // Must be called between begin()/end() calls pair. Throw otherwise.
  // Throw if positionIndex is out range.
  void vertex(size_t positionIndex);
  // create a new vertex using the provided position coordinates and the
  // texture coordinate indexes.
  // Equivalent to m.vertex(m.position(x,y,x))
  // Modeled after glVertex
  void vertex(float x, float y, float z);

  // End the current polygon or set of polygons.
  // Must be called after a begin() call. Throw otherwise.
  // Throw if there is an unfinished polygon.
  void end();

 private:
  // data which describes the mesh
  bool _withTexCoords;
  std::vector<float> _positions;
  std::vector<float> _normals;
  std::vector<float> _texCoords;
  std::vector<size_t> _vertices;
  std::vector<size_t> _vcounts;

  // data which is used when building the mesh
  boost::optional<Mode> _currentMode;
  // vertice count of the polygon currently being constructed
  size_t _currentVcount;
  size_t _currentNormal;
  size_t _currentTexCoord;
};

}  // ends namespace AL

#endif
