/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */

#include <almath/scenegraph/mesh.h>
#include <cassert>
#include <stdexcept>
#include <numeric>
#include <limits>
#include <cmath>

namespace AL {
const size_t Mesh::NO_INDEX = std::numeric_limits<size_t>::max();

Mesh::Mesh(bool withTexCoords)
    : _withTexCoords(withTexCoords),
      _currentVcount(0),
      _currentNormal(NO_INDEX),
      _currentTexCoord(NO_INDEX) {}

bool Mesh::withTexCoords() const { return _withTexCoords; }

size_t Mesh::verticesStride() const { return _withTexCoords ? 3u : 2u; }

const std::vector<float> &Mesh::positions() const { return _positions; }

size_t Mesh::positionsNb() const { return _positions.size() / 3u; }

float const *Mesh::positionPtrAt(size_t index) const {
  return &(_positions.at(index * 3u));
}

const std::vector<float> &Mesh::normals() const { return _normals; }

size_t Mesh::normalsNb() const { return _normals.size() / 3u; }

float const *Mesh::normalPtrAt(size_t index) const {
  return &(_normals.at(index * 3u));
}

const std::vector<float> &Mesh::texCoords() const { return _texCoords; }

size_t Mesh::texCoordsNb() const { return _texCoords.size() / 2u; }

float const *Mesh::texCoordPtrAt(size_t index) const {
  return &(_texCoords.at(index * 2u));
}

const std::vector<size_t> &Mesh::vertices() const { return _vertices; }

size_t Mesh::verticesNb() const { return _vertices.size() / verticesStride(); }

size_t const *Mesh::vertexPtrAt(size_t index) const {
  return &(_vertices.at(index * verticesStride()));
}

const std::vector<size_t> &Mesh::polygonVerticesCounts() const {
  return _vcounts;
}

size_t Mesh::polygonsNb() const { return _vcounts.size(); }

size_t Mesh::polygonVerticesCountAt(size_t index) const {
  assert(index < _vcounts.size());
  return _vcounts[index];
}

size_t Mesh::position(float x, float y, float z) {
  _positions.push_back(x);
  _positions.push_back(y);
  _positions.push_back(z);
  return positionsNb() - 1u;
}

size_t Mesh::normal(float x, float y, float z) {
  assert(std::abs(x * x + y * y + z * z - 1.f) <= 1.e-5f);
  _normals.push_back(x);
  _normals.push_back(y);
  _normals.push_back(z);
  _currentNormal = normalsNb() - 1u;
  return _currentNormal;
}

void Mesh::normal(size_t index) {
  if (index >= normalsNb())
    throw std::out_of_range("vertex normal index is out of range");
  _currentNormal = index;
}

size_t Mesh::texCoord(float x, float y) {
  _texCoords.push_back(x);
  _texCoords.push_back(y);
  _currentTexCoord = texCoordsNb() - 1u;
  return _currentTexCoord;
}

void Mesh::texCoord(size_t index) {
  if (index >= texCoordsNb())
    throw std::out_of_range("vertex texture coordinate index is out of range");
  _currentTexCoord = index;
}

void Mesh::begin(Mode mode) {
  if (_currentMode)
    throw std::runtime_error(
        "MeshData::begin called between MeshData::begin and MeshData::end "
        "calls");
  assert(_currentVcount == 0);
  _currentMode = mode;
}

void Mesh::vertex(size_t positionIndex) {
  if (!_currentMode) {
    throw std::runtime_error(
        "MeshData::vertex called outside MeshData::begin/MeshData::end calls "
        "pair");
  }
  if (positionIndex >= positionsNb()) {
    throw std::out_of_range("vertex position index is out of range");
  }
  if (_currentNormal == NO_INDEX) {
    throw std::runtime_error(
        "MeshData::vertex called while the current normal is undefined");
  }
  if (_withTexCoords && _currentTexCoord == NO_INDEX) {
    throw std::runtime_error(
        "MeshData::vertex called while the current texture coordinate is "
        "undefined");
  }
  _vertices.push_back(positionIndex);
  _vertices.push_back(_currentNormal);
  if (_withTexCoords) {
    _vertices.push_back(_currentTexCoord);
  }
  ++_currentVcount;
}

void Mesh::vertex(float x, float y, float z) {
  // check mode before calling position to avoid side effect
  if (!_currentMode)
    throw std::runtime_error(
        "MeshData::vertex called outside MeshData::begin/end calls pair");
  vertex(position(x, y, z));
}

void Mesh::end() {
  if (!_currentMode)
    throw std::runtime_error(
        "MeshData::end called without being preceeded by MeshData::begin call");
  if (_currentVcount != 0) {
    if (*_currentMode == POLYGON) {
      _vcounts.push_back(_currentVcount);
    } else {
      const size_t n = (*_currentMode == TRIANGLES) ? 3 : 4;
      if (_currentVcount % n != 0) {
        throw std::runtime_error(
            "MeshData::end called while there is an incomplete triangle or "
            "quad");
      }
      for (int i = _currentVcount / n; i > 0; --i) {
        _vcounts.push_back(n);
      }
    }
  }
  _currentMode.reset();
  _currentVcount = 0;
  assert(std::accumulate(_vcounts.begin(), _vcounts.end(), 0u) *
             verticesStride() ==
         _vertices.size());
}
}
