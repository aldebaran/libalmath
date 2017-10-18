/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */

#include <almath/scenegraph/gltf.h>
#include <gtest/gtest.h>
#include <boost/function_output_iterator.hpp>
#include <boost/predef.h>
#include <fstream>

using namespace AL::Math;

template <typename T>
std::string to_str(T val) {
  std::stringstream ss;
  glTF::buffer_put(ss, val);
  return ss.str();
}

std::string make_str(char c0) {
  return std::string({c0});
}

std::string make_str(char c1, char c0) {
  return std::string({c0, c1});
}

std::string make_str(char c3, char c2, char c1, char c0) {
  return std::string({c0, c1, c2, c3});
}

TEST(ALMathglTFTest, buffer_put) {
  EXPECT_EQ(make_str('\x42'), to_str('\x42'));
  EXPECT_EQ(make_str('\x42'), to_str(static_cast<unsigned char>('\x42')));

  EXPECT_EQ(make_str('\x00', '\x2A'),
            to_str(static_cast<std::int16_t>(42)));
  EXPECT_EQ(make_str('\x02', '\x2A'),
            to_str(static_cast<std::int16_t>((1<<9) + 42)));
  EXPECT_EQ(make_str('\x02', '\x2A'),
            to_str(static_cast<std::uint16_t>((1<<9) + 42)));

  EXPECT_EQ(make_str('\x40', '\x04', '\x02', '\x2A'),
            to_str(static_cast<std::uint32_t>((1<<30) + (1<<18) + (1<<9) + 42)));

  // reference values taken from
  // https://randomascii.wordpress.com/2012/01/11/tricks-with-the-floating-point-format/
  EXPECT_EQ(make_str('\0', '\0', '\0', '\0'), to_str(0.f));
  EXPECT_EQ(make_str('\x00', '\x00', '\x00', '\x01'), to_str(1.40129846e-45f));
  EXPECT_EQ(make_str('\x00', '\x80', '\x00', '\x00'), to_str(1.17549435e-38f));
  EXPECT_EQ(make_str('\x3E', '\x4C', '\xCC', '\xCD'), to_str(0.2f));
  EXPECT_EQ(make_str('\x3F', '\x80', '\x00', '\x00'), to_str(1.f));
  EXPECT_EQ(make_str('\x3F', '\xC0', '\x00', '\x00'), to_str(1.5f));
  EXPECT_EQ(make_str('\x3F', '\xE0', '\x00', '\x00'), to_str(1.75f));
  EXPECT_EQ(make_str('\x3F', '\xFF', '\xFF', '\xFF'), to_str(1.99999988f));
  EXPECT_EQ(make_str('\x40', '\x00', '\x00', '\x00'), to_str(2.f));
  EXPECT_EQ(make_str('\x4B', '\x7F', '\xFF', '\xFF'), to_str(16777215.f));
  EXPECT_EQ(make_str('\x7F', '\x7F', '\xFF', '\xFF'), to_str(3.4028234663852886e+38f));

  // this test does pass, even though glTF forbids infinity
  EXPECT_EQ(make_str('\x7F', '\x80', '\x00', '\x00'),
            to_str(std::numeric_limits<float>::infinity()));
}

// On Visual Studio 12, this fails to compile with error
// error C2582: 'operator =' function is unavailable in 'ALMathglTFTest_buffer_put_and_iterators_Test::TestBody::<lambda_79152394941179614da3f6a3b784dfab>'
// I hope that VS 2014 fixed it.
#if !(defined BOOST_COMP_MSVC) || (BOOST_COMP_MSVC >= BOOST_VERSION_NUMBER(14, 0, 0))
TEST(ALMathglTFTest, buffer_put_and_iterators) {
  std::ostringstream os;
  std::vector<float> values({1.f, 2.f});
  std::copy(begin(values), end(values),
            boost::make_function_output_iterator(
              [&os](float val) mutable {glTF::buffer_put(os, val);}));
  EXPECT_EQ(make_str('\x3F', '\x80', '\x00', '\x00') +
            make_str('\x40', '\x00', '\x00', '\x00'),
            os.str());
}
#endif

TEST(ALMathglTFTest, BufferAccessor_alone) {
  std::ostringstream os;
  glTF::BufferAccessor<float, 3> ba3(os);
  EXPECT_EQ(0u, ba3.count);
  EXPECT_EQ(std::numeric_limits<float>::max(), ba3.min[0]);
  EXPECT_EQ(std::numeric_limits<float>::lowest(), ba3.max[0]);

  ba3.put({0.f, 1.f, 2.f});
  EXPECT_EQ(1u, ba3.count);
  EXPECT_EQ(0.f, ba3.min[0]);
  EXPECT_EQ(0.f, ba3.max[0]);
  EXPECT_EQ(1.f, ba3.min[1]);
  EXPECT_EQ(1.f, ba3.max[1]);
  EXPECT_EQ(2.f, ba3.min[2]);
  EXPECT_EQ(2.f, ba3.max[2]);
  auto s3_0 = to_str(0.f) + to_str(1.f) + to_str(2.f);
  EXPECT_EQ(s3_0, os.str());

  ba3.put({1.99999988f, 1.99999988f, 1.99999988f});
  EXPECT_EQ(2u, ba3.count);
  EXPECT_EQ(0.f, ba3.min[0]);
  EXPECT_EQ(1.99999988f, ba3.max[0]);
  EXPECT_EQ(1.f, ba3.min[1]);
  EXPECT_EQ(1.99999988f, ba3.max[1]);
  EXPECT_EQ(1.99999988f, ba3.min[2]);
  EXPECT_EQ(2.f, ba3.max[2]);

  auto s3_1 = to_str(1.99999988f) + to_str(1.99999988f) + to_str(1.99999988f);
  EXPECT_EQ(s3_0 + s3_1, os.str());
}

TEST(ALMathglTFTest, BufferAccessor_interleaved) {
  std::ostringstream os;
  // use 2 accessors to write interleaved data in one buffer.
  glTF::BufferAccessor<float, 3> ba3(os);
  glTF::BufferAccessor<float, 4> ba4(os);

  ba3.put({0.f, 1.f, 2.f});
  auto s3_0 = to_str(0.f) + to_str(1.f) + to_str(2.f);
  EXPECT_EQ(s3_0, os.str());

  ba4.put({1.99999988f, 1.75f, 1.5f, 1.f});
  auto s4_0 = to_str(1.99999988f) + to_str(1.75f) + to_str(1.5f) + to_str(1.f);
  EXPECT_EQ(s3_0 + s4_0, os.str());

  ba3.put({0.f, 1.f, 2.f});
  auto s3_1 = to_str(0.f) + to_str(1.f) + to_str(2.f);
  EXPECT_EQ(s3_0 + s4_0 + s3_1, os.str());

  ba4.put({1.99999988f, 1.75f, 1.5f, 1.f});
  auto s4_1 = to_str(1.99999988f) + to_str(1.75f) + to_str(1.5f) + to_str(1.f);
  EXPECT_EQ(s3_0 + s4_0 + s3_1 + s4_1, os.str());

  EXPECT_EQ(2u, ba3.count);
  EXPECT_EQ(2u, ba4.count);
}

TEST(ALMathglTFTest, JSONArray) {
  std::ostringstream ss;
  std::vector<int> values = {-1, 0, 1};

  ss << glTF::asJSONArray<int>(values.begin(), values.end());
  EXPECT_EQ("[-1,0,1]", ss.str());

  ss.str("");
  ss << glTF::asJSONArray<int>(values);
  EXPECT_EQ("[-1,0,1]", ss.str());
}

TEST(ALMathglTFTest, JSONArray_empty) {
  std::ostringstream ss;
  std::vector<int> values;
  ss << glTF::asJSONArray<int>(values.begin(), values.end());
  EXPECT_EQ("[]", ss.str());
}


std::vector<std::array<float, 3>> make_square(float z) {
  const auto x = 1.f;
  const auto y = 1.f;
  return {{x, y, z}, {-x, y, z}, {-x, -y, z}, {x, -y, z}};
}

void animate(glTF::Document &doc,
             size_t node,
             size_t animation,
             const std::vector<float> &times,
             const std::vector<std::array<float, 3>> &translations) {
  const auto time_accessor = add_base64_bva(doc, times);
  const auto translation_accessor = add_base64_bva(doc, translations);
  auto &anim = doc.animations.at(animation);
  const auto translation_sampler =
      anim.add_sampler(glTF::Sampler{
                         time_accessor,
                         translation_accessor,
                         glTF::Interpolation::LINEAR});
  anim.add_channel(
        glTF::Channel{translation_sampler,
                      glTF::Target{node,
                                   glTF::Target::Path::translation}});
  //anim.
}

TEST(ALMathglTFTest, simple_scene) {
  glTF::Document doc;
  auto verts = make_square(0.f);
  if (!verts.empty())
    verts.push_back(verts.front());

  Eigen::Quaternionf q;
  q = Eigen::AngleAxisf(3.1416f/2, Eigen::Vector3f::UnitZ());
  const auto n0 = add_path(doc, verts,
                           std::string("mynode"));
  doc.cameras.push_back(
        "{\"type\":\"orthographic\",\"orthographic\":{\"xmag\": 40,\"ymag\": 40,\"zfar\": 10,\"znear\": 0.01}}");
  doc.nodes.push_back("{\"translation\": [0, 0, 1],\"camera\": 0}");
  const auto n1 = doc.add(
        glTF::Node{{},
                   std::string("camera"),
                    boost::optional<size_t>(),
                    0,
                    glTF::rst{boost::optional<Eigen::Quaternionf>(),
                              Eigen::Vector3f(0,0,1)}});
  const auto n2 = doc.add(
        glTF::Node{{n0, n1},
                   std::string("root"),
                    boost::optional<size_t>(),
                    boost::optional<size_t>(),
                    glTF::rst{q}});

  doc.add(glTF::Scene{{n2}});

  std::vector<float> times = {0.f, 1.f, 2.f, 3.f};
  std::vector<std::array<float, 3>> translations;
  for (auto &&z : {0.f, 1.f, 1.f, 0.f})
    translations.push_back({0.f, 0.f, z});

  const auto a0 = doc.add(glTF::Animation{});
  animate(doc, n0, a0, times, translations);

  using AL::Math::Pose2D;
  std::vector<std::pair<float, Pose2D>> traj({{0.f, Pose2D{0.f,0.f,0.f}},
                                              {1.f, Pose2D{0.f,0.f,1.f}},
                                              {2.f, Pose2D{0.f,0.f,0.f}},
                                             });
  const auto a1 = doc.add(glTF::Animation{});
  animate(doc, n0, a1, traj);
  doc.materials.push_back("{\"pbrMetallicRoughness\":{\"baseColorFactor\":[0.9,0.9,0.9,1]}}");// path
  doc.materials.push_back("{\"pbrMetallicRoughness\":{\"baseColorFactor\":[0,0,0,1]}}");// robot

  std::ofstream os("/tmp/toto.gltf");
  doc.writeJSON(os);
}
