/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */
#include <almath/scenegraph/meshfactory.h>
#include <almath/scenegraph/mesh.h>
#include <stdexcept>
#include <cmath>
#include <stddef.h>
#include <boost/math/constants/constants.hpp>

namespace AL {

void addBoxMesh(float xHE, float yHE, float zHE, Mesh &data) {
  /*****************
         4____5   vert
        /|  /|    coords
      3/_|_/_|6   ^ z
       | / | /    |
       |/__|/     |
       2   1      +---> y
  ******************/

  // vertex positions
  size_t v0 = data.position(xHE, yHE, zHE);  // 0 index is v0
  data.position(xHE, yHE, -zHE);             // 1
  data.position(xHE, -yHE, -zHE);            // 2
  data.position(xHE, -yHE, zHE);             // 3
  data.position(-xHE, -yHE, zHE);            // 4
  data.position(-xHE, yHE, zHE);             // 5
  data.position(-xHE, yHE, -zHE);            // 6
  data.position(-xHE, -yHE, -zHE);           // 7

  // begin inputing vertices
  data.begin(Mesh::QUADS);

  // front face
  data.normal(1.f, 0.f, 0.f);
  data.vertex(v0);
  data.vertex(v0 + 3);
  data.vertex(v0 + 2);
  data.vertex(v0 + 1);

  // back face
  data.normal(-1.f, 0.f, 0.f);
  data.vertex(v0 + 4);
  data.vertex(v0 + 5);
  data.vertex(v0 + 6);
  data.vertex(v0 + 7);

  // top face
  data.normal(0.f, 0.f, 1.f);
  data.vertex(v0 + 5);
  data.vertex(v0 + 4);
  data.vertex(v0 + 3);
  data.vertex(v0);

  // bottom face
  data.normal(0.f, 0.f, -1.f);
  data.vertex(v0 + 7);
  data.vertex(v0 + 6);
  data.vertex(v0 + 1);
  data.vertex(v0 + 2);

  // right face
  data.normal(0.f, 1.f, 0.f);
  data.vertex(v0 + 5);
  data.vertex(v0);
  data.vertex(v0 + 1);
  data.vertex(v0 + 6);

  // left face
  data.normal(0.f, -1.f, 0.f);
  data.vertex(v0 + 3);
  data.vertex(v0 + 4);
  data.vertex(v0 + 7);
  data.vertex(v0 + 2);

  data.end();
}

Mesh createBoxMesh(float xHalfExtent, float yHalfExtent, float zHalfExtent) {
  Mesh mesh;
  addBoxMesh(xHalfExtent, yHalfExtent, zHalfExtent, mesh);
  return mesh;
}

// small helper to hold the index of the postion and the normal of a vertex
class Vert {
 public:
  explicit Vert(size_t position = Mesh::NO_INDEX,
                size_t normal = Mesh::NO_INDEX)
      : p(position), n(normal) {}
  size_t p;
  size_t n;
  Vert operator+(int idx) { return Vert(p + idx, n + idx); }
};

static void xAddVert(const Vert &vert, Mesh &data) {
  data.normal(vert.n);
  data.vertex(vert.p);
}

static Vert xAddMeridian(float pHalfExtentZ, float pRadius, int pNbVertices,
                         float x, float y,
                         float lambda,  // azimuthal angle
                         Mesh &data) {
  Vert vert(data.positionsNb(), data.normalsNb());
  const int nphi = 2 * (pNbVertices + 1);
  const float dphi = 3.14f / nphi;
  float z = pHalfExtentZ;
  for (int iphi = 1; iphi < nphi; ++iphi) {
    float phi = iphi * dphi;  // polar angle
    float rxy = pRadius * std::sin(phi);
    data.normal(std::sin(phi) * std::cos(lambda),
                std::sin(phi) * std::sin(lambda), std::cos(phi));
    data.position(x + rxy * std::cos(lambda), y + rxy * std::sin(lambda),
                  z + pRadius * std::cos(phi));
    if ((iphi % (nphi / 2) == 0) && pHalfExtentZ != 0) {
      z = -pHalfExtentZ;
      data.normal(std::sin(phi) * std::cos(lambda),
                  std::sin(phi) * std::sin(lambda), std::cos(phi));
      data.position(x + rxy * std::cos(lambda), y + rxy * std::sin(lambda),
                    z + pRadius * std::cos(phi));
    }
  }
  return vert;
}

class Sector {
 public:
  Vert north;
  Vert south;
  float x, y;
  int ratio;
  std::vector<Vert> meridians;
};

int xSectorNbAdditionalVertices(float pHalfExtentZ, float pRadius,
                                int pNbArcVertices) {
  assert(pRadius != 0.f);
  return 2 * pNbArcVertices + ((pHalfExtentZ == 0) ? 1 : 2);
}

Sector xAddSector(float pHalfExtentZ, float pRadius, int pNbArcVertices,
                  float x, float y, float pAzimuthStart, float pAzimuthStep,
                  int pAzimuthNbSteps, size_t pNorthNormal, size_t pSouthNormal,
                  Mesh &data) {
  Sector q;
  int n = xSectorNbAdditionalVertices(pHalfExtentZ, pRadius, pNbArcVertices);
  float z = pHalfExtentZ + pRadius;
  q.north = Vert(data.position(x, y, z), pNorthNormal);
  q.south = Vert(data.position(x, y, -z), pSouthNormal);

  Vert pos, ppos;
  for (int j = 0; j < pAzimuthNbSteps; ++j) {
    float lambda = pAzimuthStart + j * pAzimuthStep;
    pos =
        xAddMeridian(pHalfExtentZ, pRadius, pNbArcVertices, x, y, lambda, data);
    assert(n == static_cast<int>(data.positionsNb() - pos.p));
    assert(n == static_cast<int>(data.normalsNb() - pos.n));
    if (ppos.p != Mesh::NO_INDEX) {
      data.begin(Mesh::TRIANGLES);
      xAddVert(q.north, data);
      xAddVert(ppos, data);
      xAddVert(pos, data);
      data.end();
      data.begin(Mesh::QUADS);
      for (int i = 0; i < n - 1; ++i) {
        xAddVert(pos + i, data);
        xAddVert(ppos + i, data);
        xAddVert(ppos + (1 + i), data);
        xAddVert(pos + (1 + i), data);
      }
      data.end();
      data.begin(Mesh::TRIANGLES);
      xAddVert(pos + (n - 1), data);
      xAddVert(ppos + (n - 1), data);
      xAddVert(q.south, data);
      data.end();
    }
    q.meridians.push_back(pos);
    ppos = pos;
  }
  return q;
}

static const float pi = boost::math::constants::pi<float>();

void addRoundedBoxMesh(float pHalfExtentX, float pHalfExtentY,
                       float pHalfExtentZ, float pRadius, int pNbArcVertices,
                       Mesh &data) {
  assert(pHalfExtentX >= 0.f);
  assert(pHalfExtentY >= 0.f);
  assert(pHalfExtentZ >= 0.f);

  if (pRadius == 0.f) {
    addBoxMesh(pHalfExtentX, pHalfExtentY, pHalfExtentZ, data);
    return;
  }
  assert(pRadius > 0.f);
  int n = xSectorNbAdditionalVertices(pHalfExtentZ, pRadius, pNbArcVertices);
  // add normals for the top and and bottom faces
  size_t northNormal = data.normal(0.f, 0.f, 1.f);
  size_t southNormal = data.normal(0.f, 0.f, -1.f);

  int nbSteps = 4 * (pNbArcVertices + 1);
  float step = 2 * pi / nbSteps;
  std::vector<Sector> sectors;
  // first quadrant
  if (pHalfExtentX == 0 && pHalfExtentY == 0) {
    sectors.push_back(xAddSector(pHalfExtentZ, pRadius, pNbArcVertices,
                                 pHalfExtentX, pHalfExtentY, 0.f, step, nbSteps,
                                 northNormal, southNormal, data));
  } else if (pHalfExtentX == 0) {
    sectors.push_back(xAddSector(
        pHalfExtentZ, pRadius, pNbArcVertices, pHalfExtentX, pHalfExtentY, 0.f,
        step, nbSteps / 2 + 1, northNormal, southNormal, data));
    sectors.push_back(xAddSector(
        pHalfExtentZ, pRadius, pNbArcVertices, pHalfExtentX, -pHalfExtentY, pi,
        step, nbSteps / 2 + 1, northNormal, southNormal, data));
  } else if (pHalfExtentY == 0) {
    sectors.push_back(xAddSector(
        pHalfExtentZ, pRadius, pNbArcVertices, pHalfExtentX, pHalfExtentY,
        -pi / 2, step, nbSteps / 2 + 1, northNormal, southNormal, data));
    sectors.push_back(xAddSector(
        pHalfExtentZ, pRadius, pNbArcVertices, -pHalfExtentX, pHalfExtentY,
        pi / 2, step, nbSteps / 2 + 1, northNormal, southNormal, data));
  } else {
    sectors.push_back(xAddSector(
        pHalfExtentZ, pRadius, pNbArcVertices, pHalfExtentX, pHalfExtentY, 0.f,
        step, nbSteps / 4 + 1, northNormal, southNormal, data));
    sectors.push_back(xAddSector(
        pHalfExtentZ, pRadius, pNbArcVertices, -pHalfExtentX, pHalfExtentY,
        pi / 2, step, nbSteps / 4 + 1, northNormal, southNormal, data));
    sectors.push_back(xAddSector(
        pHalfExtentZ, pRadius, pNbArcVertices, -pHalfExtentX, -pHalfExtentY, pi,
        step, nbSteps / 4 + 1, northNormal, southNormal, data));
    sectors.push_back(xAddSector(
        pHalfExtentZ, pRadius, pNbArcVertices, pHalfExtentX, -pHalfExtentY,
        3 * pi / 2, step, nbSteps / 4 + 1, northNormal, southNormal, data));
  }

  assert(sectors.size() > 0u);
  for (size_t i = 0; i < sectors.size(); ++i) {
    size_t ii = (i + 1) % (sectors.size());
    Vert prev = sectors[i].meridians.back();
    Vert next = sectors[ii].meridians.front();
    data.begin(Mesh::QUADS);
    xAddVert(sectors[ii].north, data);
    xAddVert(sectors[i].north, data);
    xAddVert(prev, data);
    xAddVert(next, data);
    for (int j = 0; j < n - 1; ++j) {
      xAddVert(next + j, data);
      xAddVert(prev + j, data);
      xAddVert(prev + (1 + j), data);
      xAddVert(next + (1 + j), data);
    }
    xAddVert(next + (n - 1), data);
    xAddVert(prev + (n - 1), data);
    xAddVert(sectors[i].south, data);
    xAddVert(sectors[ii].south, data);
    data.end();
  }
  if (sectors.size() >= 3) {
    data.begin(Mesh::POLYGON);
    for (size_t i = 0; i < sectors.size(); ++i) {
      xAddVert(sectors[i].north, data);
    }
    data.end();
    data.begin(Mesh::POLYGON);
    for (size_t i = sectors.size(); i > 0; --i) {
      xAddVert(sectors[i - 1u].south, data);
    }
    data.end();
  }
}

Mesh createRoundedBoxMesh(float pHalfExtentX, float pHalfExtentY,
                          float pHalfExtentZ, float pRadius, int pNbVertices) {
  Mesh mesh;
  addRoundedBoxMesh(pHalfExtentX, pHalfExtentY, pHalfExtentZ, pRadius,
                    pNbVertices, mesh);
  return mesh;
}
}
