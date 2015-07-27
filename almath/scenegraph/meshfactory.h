/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */

#ifndef LIBALMATH_SCENEGRAPH_MESHFACTORY_H
#define LIBALMATH_SCENEGRAPH_MESHFACTORY_H

namespace AL {
class Mesh;

void addBoxMesh(float xHalfExtent, float yHalfExtent, float zHalfExtent,
                Mesh &data);

Mesh createBoxMesh(float xHalfExtent, float yHalfExtent, float zHalfExtent);

void addRoundedBoxMesh(float pHalfExtentX, float pHalfExtentY,
                       float pHalfExtentZ, float pRadius, int pNbVertices,
                       Mesh &data);

Mesh createRoundedBoxMesh(float pHalfExtentX, float pHalfExtentY,
                          float pHalfExtentZ, float pRadius, int pNbVertices);
}

#endif
