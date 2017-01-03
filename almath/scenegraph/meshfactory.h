/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */

#ifndef LIBALMATH_SCENEGRAPH_MESHFACTORY_H
#define LIBALMATH_SCENEGRAPH_MESHFACTORY_H
#include <almath/api.h>

namespace AL {
class Mesh;

ALMATH_API
void addBoxMesh(float xHalfExtent, float yHalfExtent, float zHalfExtent,
                Mesh &data);

ALMATH_API
Mesh createBoxMesh(float xHalfExtent, float yHalfExtent, float zHalfExtent);

ALMATH_API
void addRoundedBoxMesh(float pHalfExtentX, float pHalfExtentY,
                       float pHalfExtentZ, float pRadius, int pNbVertices,
                       Mesh &data);

ALMATH_API
Mesh createRoundedBoxMesh(float pHalfExtentX, float pHalfExtentY,
                          float pHalfExtentZ, float pRadius, int pNbVertices);
}

#endif
