/*
 * Copyright 2017 Softbank Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

// glTF-related utilities
//
// See https://github.com/KhronosGroup/glTF/ for a description of glTF.
//
// In brief:
//
// A glTF asset consists of a JSON file describing the structure and
// composition of a scene (possibly with animations) by
// referring to buffer-based binary data (geometry, texture, skinning
// and animation data are binary).
//
// The buffers can be:
// * contained in the JSON, as base64-encoded strings,
// * stored externaly at some URIs,
// * combined with the JSON in a single binary glTF file.
//
// A glTF binary buffer is a sequence of bytes, using a little-endian
// layout.
// The "accessor" element is used to describe the data type
// (eg. 4-tuple of floats) of a buffer view.
//
// The relevant elements are:
// * buffer: "uri", "byteLength"
// * buffeView: "buffer", "byteLength", ["byteOffset", "byteStride", "target"]
// * accessor: "bufferView", "count", "min", "max", "componentType", "type", ["byteOffset"]

// with componentType in
// * 5120 (GL_BYTE)
// * 5121 (GL_UNSIGNED_BYTE)
// * 5122 (GL_SHORT)
// * 5123 (GL_UNSIGNED_SHORT)
// * 5125 (GL_UNSIGNED_INT)
// * 5126 (GL_FLOAT)
//
// and type in:
// * "SCALAR"
// * "VEC2"
// * "VEC3"
// * "VEC4"
// * "MAT2"
// * "MAT3"
// * "MAT4"
//
// Beware, the byteStride suggests that one can interleave bufferViews,
// but I cound not get working.
//
// In C++, binary buffers are typically modeled as stream of char.
//
// Writing C++ types (eg. float) into a binary buffer can be done with:
// 1. a function writing into a ostream:
//    void buffer_put(float, ostream &)
// 2. a function writing into an OutputIterator to char:
//    template<typename OutputIterator>
//    void buffer_put(float, OutputIterator it)
// 3. a custom InputIterator adaptor, converting a sequence of floats into a
//    sequence of char.
//
// Here we use the first technique, with the buffer_put family of
// functions.
#ifndef LIB_ALMATH_SCENEGRAPH_GLTF_H
#define LIB_ALMATH_SCENEGRAPH_GLTF_H

#include <cstdint>
#include <limits>
#include <sstream>
#include <type_traits>
#include <boost/config.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <boost/endian/buffers.hpp>
#include <boost/algorithm/string/join.hpp>
#include <boost/range/adaptor/transformed.hpp>

#include <almath/types/alposition2d.h>
#include <almath/types/alpose2d.h>
#include <Eigen/Geometry>

namespace AL {
namespace Math {
namespace glTF {

enum struct ComponentType {
  GL_BYTE = 5120,
  GL_UNSIGNED_BYTE = 5121,
  GL_SHORT = 5122,
  GL_UNSIGNED_SHORT = 5123,
  GL_UNSIGNED_INT = 5125,
  GL_FLOAT =  5126
};

enum struct DataType {
  SCALAR,
  VEC3,
  VEC4,
  MAT2,
  MAT3,
  MAT4
};


template< DataType element>
struct data_type_traits {
  //BOOST_STATIC_CONSTEXPR size_t number_of_components;
  //static BOOST_CONSTEXPR const char *cstr();
};

template<>
struct data_type_traits<DataType::SCALAR> {
  BOOST_STATIC_CONSTEXPR size_t number_of_components = 1u;
  static BOOST_CONSTEXPR const char *cstr() {return "SCALAR";}
};

template<>
struct data_type_traits<DataType::VEC3> {
  BOOST_STATIC_CONSTEXPR size_t number_of_components = 3u;
  static BOOST_CONSTEXPR const char *cstr() {return "VEC3";}
};

template<>
struct data_type_traits<DataType::VEC4> {
  BOOST_STATIC_CONSTEXPR size_t number_of_components = 4u;
  static BOOST_CONSTEXPR const char *cstr() {return "VEC4";}
};

template<>
struct data_type_traits<DataType::MAT2> {
  BOOST_STATIC_CONSTEXPR size_t number_of_components = 4u;
  static BOOST_CONSTEXPR const char *cstr() {return "MAT2";}
};

// todo: add the other specializations, MAT3, ...

enum struct Mode {
  LINE_STRIP =  3
};

// writes a GL_BYTE (5120) value to a binary stream,
// in the format expected by glTF: 1 byte.
inline void buffer_put(std::ostream &os, char val) {
  static_assert(sizeof(char) == 1u,
                "error: the native char type is not 1 byte wide");
  os.put(val);
}

// writes a GL_UNSIGNED_BYTE (5121) value to a binary stream,
// in the format expected by glTF: 1 byte.
inline void buffer_put(std::ostream &os, unsigned char val) {
  static_assert(sizeof(unsigned char) == 1u,
                "error: the native char type is not 1 byte wide");
  os.put(static_cast<char>(val));
}

// writes a GL_SHORT (5122) value to a binary stream,
// in the format expected by glTF: 2 bytes, little-endian.
inline void buffer_put(std::ostream &os, std::int16_t val) {
  const boost::endian::little_int16_buf_t tmp{val};
  os.write(tmp.data(), sizeof(tmp));
}

// writes a GL_UNSIGNED_SHORT (5123) value to a binary stream,
// in the format expected by glTF: 2 bytes, little-endian
inline void buffer_put(std::ostream &os, std::uint16_t val) {
  const boost::endian::little_uint16_buf_t tmp{val};
  os.write(tmp.data(), sizeof(tmp));
}

// writes a GL_UNSIGNED_INT (5125) value to a binary stream,
// in the format expected by glTF: 4 bytes, little-endian
inline void buffer_put(std::ostream &os, std::uint32_t val) {
  const boost::endian::little_uint32_buf_t tmp{val};
  os.write(tmp.data(), sizeof(tmp));
}

// writes a GL_FLOAT (5126) value to a binary stream,
// in the format expected by glTF: 4 bytes, little-endian, single precision
// IEEE-754 number.
inline void buffer_put(std::ostream &os, float val) {
  static_assert(std::numeric_limits<float>::is_iec559,
                "error: the native float type is not IEEE-754-compliant");
  static_assert(sizeof(float) == 4u,
                "error: the native float type is not 4 bytes wide");
  // Thanks to thoses assertions, we know that float is a single-precision
  // IEEE-754 number.
  // However we don't know
  // * if float is little-endian or big-endian
  // * if float endianness is the same as int endiannes
  //
  // Yet, we'll assume that float endianness and int endianness are
  // identical.
  //
  // Some details from
  // https://en.wikipedia.org/wiki/Endianness#Floating_point:
  //
  //   Theoretically, this means that even standard IEEE floating-point data
  //   written by one machine might not be readable by another.
  //   However, on modern standard computers (i.e., implementing IEEE 754),
  //   one may in practice safely assume that the endianness is the same for
  //   floating-point numbers as for integers, making the conversion
  //   straightforward regardless of data type.
  //   (Small embedded systems using special floating-point formats may be
  //   another matter however).
  //
  // Linux implementation can be checked in:
  // /usr/include/x86_64-linux-gnu/ieee754.h

#ifdef BOOST_LITTLE_ENDIAN
  // "optimized" version
  os.write(reinterpret_cast<const char *>(&val), 4);
#else
  // portable version: works for little and big endian architectures
  const boost::endian::little_uint32_buf_t tmp{
      *reinterpret_cast<const uint32_t*>(&val)};
  os.write(tmp.data, sizeof(tmp));
#endif
}

// Helper class which writes N components (of type T) at a time,
// while also storing the metadata that will be needed to fill the
// glTF accessor to the data.
template <typename T, size_t N>
class BufferAccessor {
public:
  using value_type = std::array<T, N>;
  value_type min;
  value_type max;
  size_t count;
  std::ostream &os;
  BufferAccessor(std::ostream &os) : count(0u), os(os) {
    std::fill(min.begin(), min.end(), std::numeric_limits<T>::max());
    std::fill(max.begin(), max.end(), std::numeric_limits<T>::lowest());
  }
  void put(value_type val) {
    for (auto i = 0u; i < val.size(); ++i) {
      min[i] = std::min(min[i], val[i]);
      max[i] = std::max(max[i], val[i]);
      buffer_put(os, val[i]);
    }
    ++count;
  }
};

// http://www.boost.org/doc/libs/1_59_0/libs/serialization/doc/dataflow.html
// https://stackoverflow.com/questions/7053538/how-do-i-encode-a-string-to-base64-using-only-boost
 inline std::string encode64(const std::string &val) {
    using namespace boost::archive::iterators;
    using It = base64_from_binary<transform_width<std::string::const_iterator, 6, 8>>;
    auto tmp = std::string(It(std::begin(val)), It(std::end(val)));
    return tmp.append((3 - val.size() % 3) % 3, '=');
}

template <typename T>
std::string toJSON(T val) {
  static_assert(std::is_arithmetic<T>::value,
                "toJSON requires T to be an integer,"
                " floating point and boolean type");
  // TODO: support std::string, with escaping.
  // TODO: use SFINAE rather than assertion
  // we use boost::lexical_cast because it handles the stream precision
  return boost::lexical_cast<std::string>(val);
}

// a proxy object to stream  a range as a JSON array
template <typename T, typename IT>
class AsJSONArray {
private:
  const IT _begin;
  const IT _end;

public:
  AsJSONArray(IT b, IT e) : _begin(b), _end(e) {}

  friend std::ostream &operator<<(std::ostream &os, const AsJSONArray &a) {
    // Here is an alternate implementation:
    // os << '['
    //    << boost::join(boost::adaptors::transform(v.min, toJSON<float>), ",")
    //    << ']';
    // A better one would be to use
    // http://en.cppreference.com/w/cpp/experimental/ostream_joiner
    os.put('[');
    auto it = a._begin;
    auto write = [&os](const T &value) mutable {
      os << toJSON(value);
    };
    if (it != a._end) {
      write(*it);
      while (++it != a._end) {
        os.put(',');
        write(*it);
      }
    }
    os.put(']');
    return os;
  }
};

// helper to deduce iterator type.
template<typename T, typename IT>
AsJSONArray<T, IT> asJSONArray(IT b, IT e) {
  return AsJSONArray<T, IT>(b, e);
}

// helper to deduce iterator type.
template<typename T, typename R, typename IT=typename R::const_iterator>
AsJSONArray<T, IT> asJSONArray(const R &range) {
  using std::begin;
  using std::end;
  return AsJSONArray<T, IT>(begin(range), end(range));
}

struct Buffer {
  size_t byteLength;
  std::string uri;
};

inline
Buffer make_base64_Buffer(const std::string &binbuffer) {
  return Buffer{binbuffer.size(),
                std::string("data:application/octet-stream;base64,") +\
                encode64(binbuffer)};
}

inline
std::ostream &operator<<(std::ostream &os, const Buffer &v) {
  os << "{\"byteLength\":" << v.byteLength
     << ",\"uri\":\"" << v.uri << "\""
     << "}";
  return os;
}

struct BufferView {
  size_t buffer;
  size_t byteLength;
  boost::optional<size_t> byteOffset;
  boost::optional<size_t> byteStride;
  // target
};

inline
std::ostream &operator<<(std::ostream &os, const BufferView &v) {
  os << "{\"buffer\":" << v.buffer
     << ",\"byteLength\":" << v.byteLength;
  if (v.byteOffset)
     os << ",\"byteOffset\":" << *v.byteOffset;
  if (v.byteStride)
     os << ",\"byteStride\":" << *v.byteStride;
  os << "}";
  return os;
}

template <typename T, DataType DTYPE>
struct Accessor {
  size_t bufferView;
  ComponentType componentType; // redundant with T
  size_t count;
  static const DataType type = DTYPE;
  std::array<T, data_type_traits<DTYPE>::number_of_components> min, max;// TODO: shall go in a union/variant, maybe depending on DataType?
  boost::optional<size_t> byteOffset;
};

template <typename T, DataType _type>
std::ostream &operator<<(std::ostream &os, const Accessor<T, _type> &v) {

  auto to_cstr = [](DataType type) {
    switch (type) {
    case DataType::SCALAR:
      return "SCALAR";
    case DataType::VEC3:
      return "VEC3";
    case DataType::VEC4:
      return "VEC4";
    case DataType::MAT2:
      return "MAT2";
    case DataType::MAT3:
      return "MAT3";
    case DataType::MAT4:
      return "MAT4";
    }
    std::abort();
    return "";
  };
  os << "{\"bufferView\":" << v.bufferView
     << ",\"componentType\":"<< static_cast<int>(v.componentType)
     << ",\"count\":" << toJSON(v.count)
     << ",\"type\":\"" << to_cstr(v.type) << "\""
     << ",\"min\":["
     << boost::join(boost::adaptors::transform(v.min, toJSON<float>), ",")
     << "]"
     << ",\"max\":" << asJSONArray<T>(v.max);
  if (v.byteOffset)
    os << ",\"byteOffset\":" << *v.byteOffset;
  os << "}";
  return os;
}

struct Mesh {
  // TODO: maybe create a struct for primitives too
  std::vector<std::string> primitives;
  boost::optional<std::string> name;
};

inline
std::ostream &operator<<(std::ostream &os, const Mesh &v) {
  os << "{\"primitives\":["
     << boost::join(v.primitives, ",") << "]";
  if (v.name)
    os << ",\"name\":\"" << *v.name << "\"";
  os << "}";
  return os;
}

struct rst {
  // TODO: find a better way, which should support a full matrix
  boost::optional<Eigen::Quaternionf> rotation;
  boost::optional<Eigen::Vector3f> scale;
  boost::optional<Eigen::Vector3f> translation;
};

struct Node {
  std::vector<size_t> children;
  boost::optional<std::string> name;
  boost::optional<size_t> mesh;
  boost::optional<size_t> camera;
  boost::optional<rst> matrix;
  //
  // rotation, scale, translation
  // matrix

};

inline
std::ostream &operator<<(std::ostream &os, const Node &v) {
  auto prefix = '{';
  if (!v.children.empty()) {
    os << prefix << "\"children\":" << asJSONArray<size_t>(v.children);
    prefix = ',';
  }
  if (v.name) {
    os << prefix << "\"name\":\"" << *v.name << "\"";
    prefix = ',';
  }
  if (v.mesh) {
    os << prefix << "\"mesh\":" << toJSON(*v.mesh);
    prefix = ',';
  }
  if (v.camera) {
    os << prefix << "\"camera\":" << toJSON(*v.camera);
    prefix = ',';
  }
  if (v.matrix) {
    if (v.matrix->rotation) {
     const auto q = *(v.matrix->rotation);
      os << prefix << "\"rotation\":"
         << asJSONArray<float>(
              std::array<float,4>{{q.x(), q.y(), q.z(), q.w()}});
      prefix = ',';
    }
    // TODO: support scale
    if (v.matrix->translation) {
     const auto vec3 = *(v.matrix->translation);
      os << prefix << "\"translation\":"
         << asJSONArray<float>(
              std::array<float,3>{{vec3[0], vec3[1], vec3[2]}});
      prefix = ',';
    }
  }
  if (prefix == '{')
    os << '{';
  os << "}";
  return os;
}

struct Scene {
  std::vector<size_t> nodes;
  boost::optional<std::string> name;
};

inline
std::ostream &operator<<(std::ostream &os, const Scene &v) {
  os << "{\"nodes\":" << asJSONArray<size_t>(v.nodes);
  if (v.name)
    os << ",\"name\":\"" << *v.name << "\"";
  os << "}";
  return os;
}


enum struct Interpolation {
  LINEAR
};

inline std::string to_string(Interpolation val) {
  switch (val) {
  case (Interpolation::LINEAR):
    return "LINEAR";
  }
  std::abort();
}

struct Sampler {
  size_t input;
  size_t output;
  Interpolation interpolation;
};

inline
std::ostream &operator<<(std::ostream &os, const Sampler &v) {
  os << "{\"input\":" << toJSON(v.input)
     << ",\"output\":"<< toJSON(v.output)
     << ",\"interpolation\":\"" << to_string(v.interpolation) << "\""
     << "}";
  return os;
}

struct Target {
  enum struct Path {
    translation,
    rotation
  };

  size_t node;
  Path path;
};

inline
std::string to_string(Target::Path path) {
  switch (path) {
  case Target::Path::translation:
    return "translation";
  case Target::Path::rotation:
    return "rotation";
  }
  std::abort();
}

inline
std::ostream &operator<<(std::ostream &os, const Target &v) {
  os << "{\"node\":" << toJSON(v.node)
     << ",\"path\":\"" << to_string(v.path) << "\"}";
  return os;
}

struct Channel {
  size_t sampler;
  Target target;
};

inline
std::ostream &operator<<(std::ostream &os, const Channel &v) {
  os << "{\"sampler\":" << toJSON(v.sampler)
     << ",\"target\":" << v.target << "}";
  return os;

}

struct Animation {
// channel/samplers
  struct Indexes {
    size_t translation;
    size_t rotation;
  };
  boost::optional<std::string> name;
  std::vector<Sampler> samplers;
  std::vector<Channel> channels;
  size_t add_sampler(const Sampler &v) {
    samplers.push_back(v);
    return samplers.size() - 1u;
  }
  size_t add_channel(const Channel &v) {
    channels.push_back(v);
    return channels.size() - 1u;
  }
  /*
  Indexes addSamplers(size_t time_accessor,
                      size_t translation_accessor,
                      size_t rotation_accessor);
  Indexes addChannels(Indexes, size_t target_node);
  void toJSON(std::ostream &os) const;
  */ //TODO rm?
};

inline
std::ostream &operator<<(std::ostream &os, const Animation &v) {

  {
    std::vector<std::string> tmp;
    for (const auto &a: v.samplers) {
      tmp.push_back(boost::lexical_cast<std::string>(a));
    }
    os << "{\"samplers\":[" << boost::join(tmp, ",") << "]";
  }
  {
    std::vector<std::string> tmp;
    for (const auto &a: v.channels) {
      tmp.push_back(boost::lexical_cast<std::string>(a));
    }
    os << ",\"channels\":[" << boost::join(tmp, ",") << "]";
  }
  os << "}";
  return os;
}

class Document {
private:
  template <typename T>
  size_t _add(std::vector<std::string> &vec, const T &val) {
    vec.push_back(boost::lexical_cast<std::string>(val));
    return vec.size() - 1u;
  }
public:
  std::vector<std::string> buffers;
  std::vector<std::string> bufferViews;
  std::vector<std::string> accessors;
  std::vector<std::string> materials;
  std::vector<std::string> meshes;
  std::vector<std::string> cameras;
  std::vector<std::string> nodes;
  std::vector<std::string> scenes;
  std::vector<Animation> animations;
  boost::optional<size_t> scene;
  size_t add(const Buffer &v) {return _add(buffers, v);}

  size_t add(const BufferView &v) {return _add(bufferViews, v);}

  template<typename T, DataType DTYPE>
  size_t add(const Accessor<T, DTYPE> &v) {return _add(accessors, v);}

  size_t add(const Mesh &v) {return _add(meshes, v);}

  size_t add(const Node &v) {return _add(nodes, v);}

  size_t add(const Scene &v) {return _add(scenes, v);}

  size_t add(const Animation &v) {
    animations.push_back(v);
    return animations.size() - 1u;
  }

  void writeJSON(std::ostream &os) const {
    os << "{\"asset\": {\"version\": \"2.0\"}"
       << "\n,\"buffers\":\n ["
       << boost::join(buffers, "\n ,")
       << "]\n,\"bufferViews\":\n ["
       << boost::join(bufferViews, "\n ,")
       << "]\n,\"accessors\":\n ["
       << boost::join(accessors, "\n ,")
       << "]\n,\"meshes\":\n ["
       << boost::join(meshes, "\n ,") << "]\n";
    if (!materials.empty()) {
      os << ",\"materials\":\n ["
         << boost::join(materials, "\n ,") << "]\n";
    } if (!cameras.empty()) {
      os << ",\"cameras\":\n ["
         << boost::join(cameras, "\n ,") << "]\n";
    } if (!nodes.empty()) {
      os << ",\"nodes\":\n ["
         << boost::join(nodes, "\n ,") << "]\n";
    } if (!scenes.empty()) {
      os << ",\"scenes\":\n ["
         << boost::join(scenes, "\n ,") << "]\n";
    } if (scene) {
      os << ",\"scene\":" << toJSON(*scene) << "\n";
    } if (!animations.empty()) {
      std::vector<std::string> anims;
      for (const auto &a: animations)
        anims.push_back(boost::lexical_cast<std::string>(a));
      os << ",\"animations\":\n ["
         << boost::join(anims, "\n ,") << "]\n";
    }
    os << "\n}";
  }
};




// TODO: move somewhere else

inline
size_t add_base64_bva(glTF::Document &doc,
                     const std::vector<float> &vecs) {
  std::ostringstream bs;
  glTF::BufferAccessor<float, 1> ba(bs);
  for (const auto &vec: vecs) {
    ba.put({vec});
  }
  const auto binbuffer = bs.str();
  const auto binbuffer_length = binbuffer.size();
  const auto b0 = doc.add(glTF::make_base64_Buffer(binbuffer));
  const auto bv0 = doc.add(glTF::BufferView{b0, binbuffer_length});
  const auto a0 = doc.add(glTF::Accessor<float, glTF::DataType::SCALAR>{
      bv0, glTF::ComponentType::GL_FLOAT,
      ba.count, ba.min, ba.max});
  return a0;
}

inline
size_t add_base64_bva(glTF::Document &doc,
                     const std::vector<std::array<float, 3>> &vecs) {
  std::ostringstream bs;
  glTF::BufferAccessor<float, 3> ba(bs);
  for (const auto &vec: vecs) {
    ba.put(vec);
  }
  const auto binbuffer = bs.str();
  const auto binbuffer_length = binbuffer.size();
  const auto b0 = doc.add(glTF::make_base64_Buffer(binbuffer));
  const auto bv0 = doc.add(glTF::BufferView{b0, binbuffer_length});
  const auto a0 = doc.add(glTF::Accessor<float, glTF::DataType::VEC3>{
      bv0, glTF::ComponentType::GL_FLOAT,
      ba.count, ba.min, ba.max});
  return a0;
}

inline
size_t add_path(glTF::Document &doc,
                const std::vector<std::array<float, 3>> &verts,
                boost::optional<std::string> node_name) {
  const auto a0 = add_base64_bva(doc, verts);
  std::stringstream ps;
  ps << "{\"mode\":" << static_cast<int>(glTF::Mode::LINE_STRIP);
  //if (material)
  //  ps << ",\"material\":" << *material;
  ps << ",\"attributes\":{\"POSITION\":" << a0 << "}"
     << "}";
  const auto m0 = doc.add(glTF::Mesh{{ps.str()}});
  const auto n0 = doc.add(glTF::Node{{},
                                     node_name,
                                     m0});
  return n0;
}

inline
size_t add_path(glTF::Document &doc,
                const std::vector<Math::Position2D> &path,
                boost::optional<std::string> node_name) {
  std::vector<std::array<float, 3>> verts;
  verts.reserve(path.size());
  for (const auto &p: path) {
    verts.push_back({p.x, p.y, 0.f});
  }
  return add_path(doc, verts, node_name);
}

inline
void animate(glTF::Document &doc,
             size_t node,
             size_t animation,
             const std::vector<std::pair<float, Math::Pose2D>> &traj) {
  auto &anim = doc.animations.at(animation);
  std::ostringstream time_stream, translation_stream, rotation_stream;
  glTF::BufferAccessor<float, 1> time_ba(time_stream);
  glTF::BufferAccessor<float, 3> translation_ba(translation_stream);
  glTF::BufferAccessor<float, 4> rotation_ba(rotation_stream);
  for (const auto &el: traj) {
    time_ba.put({el.first});
    translation_ba.put({el.second.x, el.second.y, 0.f});
    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(el.second.theta, Eigen::Vector3f::UnitZ());
    rotation_ba.put({q.x(), q.y(), q.z(), q.w()});
  }

  auto binbuffer = time_stream.str();
  auto binbuffer_length = binbuffer.size();
  auto b = doc.add(make_base64_Buffer(binbuffer));
  auto bv = doc.add(glTF::BufferView{b, binbuffer_length});
  const auto time_accessor = doc.add(
      glTF::Accessor<float, glTF::DataType::SCALAR>{
          bv, glTF::ComponentType::GL_FLOAT,
          time_ba.count, time_ba.min, time_ba.max});

  binbuffer = translation_stream.str();
  binbuffer_length = binbuffer.size();
  b = doc.add(make_base64_Buffer(binbuffer));
  bv = doc.add(glTF::BufferView{b, binbuffer_length});
  const auto translation_accessor = doc.add(
      glTF::Accessor<float, glTF::DataType::VEC3>{
          bv, glTF::ComponentType::GL_FLOAT,
          translation_ba.count, translation_ba.min, translation_ba.max});

  binbuffer = rotation_stream.str();
  binbuffer_length = binbuffer.size();
  b = doc.add(make_base64_Buffer(binbuffer));
  bv = doc.add(glTF::BufferView{b, binbuffer_length});
  const auto rotation_accessor = doc.add(
      glTF::Accessor<float, glTF::DataType::VEC4>{
          bv, glTF::ComponentType::GL_FLOAT,
          rotation_ba.count, rotation_ba.min, rotation_ba.max});

  const auto translation_sampler =
      anim.add_sampler(glTF::Sampler{
                         time_accessor,
                         translation_accessor,
                         glTF::Interpolation::LINEAR});
  anim.add_channel(
        glTF::Channel{translation_sampler,
                      glTF::Target{node,
                                   glTF::Target::Path::translation}});

  const auto rotation_sampler =
      anim.add_sampler(glTF::Sampler{
                         time_accessor,
                         rotation_accessor,
                         glTF::Interpolation::LINEAR});
  anim.add_channel(
        glTF::Channel{rotation_sampler,
                      glTF::Target{node,
                                   glTF::Target::Path::rotation}});
}


}
}
}

#endif
