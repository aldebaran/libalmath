/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */

#include <almath/scenegraph/urdf.h>
#include <almath/scenegraph/urdfrigidbodysystemtraverser.h>
#include <boost/function/function2.hpp>
#include <gtest/gtest.h>
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/version.hpp>
#include <boost/ref.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <array>
#include <memory>
#include "myrigidbodysystembuilder.h"
#include <boost/regex.hpp>

namespace std {
::std::ostream& operator<<(::std::ostream& os, const std::pair<double, double>& v) {
  return os << "(" << v.first << ", " << v.second << ")";
}
}

using namespace AL;
using namespace AL::Math;
typedef AL::urdf::ptree ptree;
using namespace AL::Math::RigidBodySystemBuilder;

namespace {
// small experimentation with boost::multi_index
using namespace boost::multi_index;
class Node {
 public:
  std::string _name;
  Node(const std::string &name) : _name(name) {}
  std::string name() const {
    std::cout << "name() -> " << _name << std::endl;
    return _name;
  }
};
struct NameTag {};  // tag

typedef boost::multi_index_container<
    Node,
    indexed_by<sequenced<>,  // used to keep the original joint ordering
               ordered_unique<tag<NameTag>, const_mem_fun<Node, std::string,
                                                          &Node::name> > > >
    NodesIndex;
}

#define ADD_NODE(name)                                          \
  if (1) {                                                      \
    std::cout << "adding node \"" << name << "\"" << std::endl; \
    nodes.push_back(Node(name));                                \
  }

TEST(Urdf, MultiIndexContainer) {
  // as shown by the logs when running this example,
  // boost::multi_index does not cache the Name keys, it queries
  // them when searching a node and when adding a node
  // (to decide where to add the new nore in its binary tree).
  ::NodesIndex nodes;
  ADD_NODE("a");
  ADD_NODE("b");
  ADD_NODE("c");
  ADD_NODE("d");
  ADD_NODE("e");
  ADD_NODE("f");
  ADD_NODE("g");
  ADD_NODE("h");
  ADD_NODE("i");
  ADD_NODE("j");
  ADD_NODE("k");
  ADD_NODE("l");
  std::cout << "searching node \"l\"" << std::endl;
  nodes.get< ::NameTag>().find("l");
}

// utils to create and manipulate urdf files
void print(const ptree &pt) {
  boost::property_tree::xml_writer_settings<std::string> settings(' ', 4);
  write_xml(std::cout, pt, settings);
}

ptree &addRobot(ptree &pt) { return pt.add_child("robot", ptree()); }

ptree &addLink(ptree &pt, const std::string &name) {
  ptree &link = pt.add_child("link", ptree());
  link.put("<xmlattr>.name", name);
  return link;
}

ptree &addLink(ptree &pt, const std::string &name, double mass, double ixx,
               double ixy, double ixz, double iyy, double iyz, double izz) {
  ptree &link = addLink(pt, name);
  link.put("inertial.mass.<xmlattr>.value", mass);
  link.put("inertial.inertia.<xmlattr>.ixx", ixx);
  link.put("inertial.inertia.<xmlattr>.ixy", ixy);
  link.put("inertial.inertia.<xmlattr>.ixz", ixz);
  link.put("inertial.inertia.<xmlattr>.iyy", iyy);
  link.put("inertial.inertia.<xmlattr>.iyz", iyz);
  link.put("inertial.inertia.<xmlattr>.izz", izz);
  return link;
}

ptree &addLink(ptree &pt, const std::string &name, double mass, double ixx,
               double ixy, double ixz, double iyy, double iyz, double izz,
               const urdf::Array3d &xyz, const urdf::Array3d &rpy) {
  ptree &link = addLink(pt, name, mass, ixx, ixy, ixz, iyy, iyz, izz);
  link.put("inertial.origin.<xmlattr>.xyz", xyz, urdf::Array3dTranslator());
  link.put("inertial.origin.<xmlattr>.rpy", rpy, urdf::Array3dTranslator());
  return link;
}
ptree &addLink(ptree &pt, const std::string &name, double mass) {
  ptree &link = addLink(pt, name, mass, mass, mass, mass, mass, mass, mass);
  return link;
}

ptree &addJoint(ptree &pt, const std::string &parent_link,
                const std::string &child_link, const std::string &name) {
  ptree &joint = pt.add_child("joint", ptree());
  joint.put("<xmlattr>.name", name);
  joint.put("parent.<xmlattr>.link", parent_link);
  joint.put("child.<xmlattr>.link", child_link);
  return joint;
}

ptree &addJoint(ptree &pt, const std::string &parent_link,
                const std::string &child_link, const std::string &name,
                const std::string &type) {
  ptree &joint = addJoint(pt, parent_link, child_link, name);
  joint.put("<xmlattr>.type", type);
  return joint;
}

ptree &addJoint(ptree &pt, const std::string &parent_link,
                const std::string &child_link, const std::string &name,
                const std::string &type, const urdf::Array3d &xyz,
                const urdf::Array3d &rpy) {
  ptree &joint = addJoint(pt, parent_link, child_link, name, type);
  joint.put("origin.<xmlattr>.xyz", xyz, urdf::Array3dTranslator());
  joint.put("origin.<xmlattr>.rpy", rpy, urdf::Array3dTranslator());
  return joint;
}

TEST(Urdf, PropertyTree) {
  EXPECT_EQ(3, ptree("3").get_value<int>());
  EXPECT_EQ(3, ptree(" \n \t \r 3 ").get_value<int>());
  EXPECT_ANY_THROW(ptree("3.3").get_value<int>());
  EXPECT_ANY_THROW(ptree("3 3").get_value<int>());
  EXPECT_EQ(0, ptree("3.3").get_value<int>(0));

  // this specific value requires more that digits10+1 precision
  double d = -1.8364336390987788;
  ptree pt;
  pt.put_value(d);
  EXPECT_EQ(d, pt.get_value<double>());
}

TEST(Urdf, Array3d) {
  {
    ptree pt("1 2.2 3.3");
    urdf::Array3d xyz = pt.get_value<urdf::Array3d>(urdf::Array3dTranslator());
    EXPECT_EQ(xyz[0], 1.);
    EXPECT_EQ(xyz[1], 2.2);
    EXPECT_EQ(xyz[2], 3.3);
  }
  {
    ptree pt(" 1   2.2 \n 3.3 ");
    urdf::Array3d xyz = pt.get_value<urdf::Array3d>(urdf::Array3dTranslator());
    EXPECT_EQ(xyz[0], 1.);
    EXPECT_EQ(xyz[1], 2.2);
    EXPECT_EQ(xyz[2], 3.3);
  }
  {
    ptree pt("1 toto 3.3");
    EXPECT_ANY_THROW(pt.get_value<urdf::Array3d>(urdf::Array3dTranslator()));
  }
  {
    ptree pt("1");
    EXPECT_ANY_THROW(pt.get_value<urdf::Array3d>(urdf::Array3dTranslator()));
  }
  {
    ptree pt("");
    EXPECT_ANY_THROW(pt.get_value<urdf::Array3d>(urdf::Array3dTranslator()));
  }
  {
    ptree pt("");
    pt.put_value(urdf::Array3d({1., 2.2 ,3.3}), urdf::Array3dTranslator());
    // check the serialized data
    EXPECT_EQ(pt.data(), "1 2.2000000000000002 3.2999999999999998");
    // check it round-trips
    urdf::Array3d xyz = pt.get_value<urdf::Array3d>(urdf::Array3dTranslator());
    EXPECT_EQ(xyz[0], 1.);
    EXPECT_EQ(xyz[1], 2.2);
    EXPECT_EQ(xyz[2], 3.3);
  }
}

TEST(Urdf, Pose) {
  urdf::Array3d zeros = {{0, 0, 0}};
  urdf::Array3d exp0 = {{1, 2.2, 3.3}};
  urdf::Array3d mexp0 = {{-1, -2.2, -3.3}};
  urdf::Array3d exp1 = {{11, 12.2, 13.3}};
  // ctor
  {
    urdf::Pose p;
    EXPECT_EQ(zeros, p.xyz());
    EXPECT_EQ(zeros, p.rpy());
    EXPECT_TRUE(is_identity(p));
  }
  {
    urdf::Pose p{{{0, 0, -0}}, {{-0, 0, 0}}};
    EXPECT_EQ(zeros, p.xyz());
    EXPECT_EQ(zeros, p.rpy());
    EXPECT_TRUE(is_identity(p));
  }
  {
    urdf::Pose p{exp0, exp1};
    EXPECT_EQ(exp0, p.xyz());
    EXPECT_EQ(exp1, p.rpy());
    EXPECT_FALSE(is_identity(p));
  }
  {
    urdf::Pose p{zeros, exp1};
    EXPECT_FALSE(is_identity(p));
  }
  {
    urdf::Pose p{exp0, zeros};
    EXPECT_FALSE(is_identity(p));
  }
  // from_ptree
  {
    ptree pt;
    pt.put("<xmlattr>.xyz", "1 2.2 3.3");
    pt.put("<xmlattr>.rpy", "11 12.2 13.3");
    urdf::Pose p = urdf::Pose::from_ptree(pt);
    EXPECT_EQ(exp0, p.xyz());
    EXPECT_EQ(exp1, p.rpy());
  }
  {
    ptree pt;
    pt.put("<xmlattr>.xyz", "1 toto 3.3");
    EXPECT_ANY_THROW(urdf::Pose::from_ptree(pt));
  }
  {
    ptree pt;
    pt.put("<xmlattr>.rpy", "1 toto 3.3");
    EXPECT_ANY_THROW(urdf::Pose::from_ptree(pt));
  }
  {
    ptree pt;
    pt.put("<xmlattr>.xyz", "0 0 -0");
    pt.put("<xmlattr>.rpy", "-0 0 0");
    urdf::Pose p = urdf::Pose::from_ptree(pt);
    EXPECT_EQ(zeros, p.xyz());
    EXPECT_EQ(zeros, p.rpy());
    EXPECT_TRUE(is_identity(p));
  }
  {
    ptree pt;
    urdf::Pose p = urdf::Pose::from_ptree(pt);
    EXPECT_EQ(zeros, p.xyz());
    EXPECT_EQ(zeros, p.rpy());
  }
  // inverse
  {
    urdf::Pose p;
    EXPECT_TRUE(is_identity(p.inverse()));
  }
  {
    urdf::Pose p(exp0, zeros);
    urdf::Pose ip = p.inverse();

    EXPECT_EQ(mexp0, ip.xyz());
    EXPECT_TRUE(AL::urdf::is_zero(ip.rpy()));
  }
  {
    urdf::Pose p(zeros, exp0);
    urdf::Pose ip = p.inverse();

    EXPECT_EQ(zeros, ip.xyz());
    EXPECT_FALSE(AL::urdf::is_zero(ip.rpy()));  // TODO { -1.17549, 2.54225, 2.35153 }
  }
  // operator*(,)
  {
    urdf::Pose p(exp0, exp1);
    {
      EXPECT_TRUE(is_identity(urdf::Pose() * urdf::Pose()));
    }
    {
      urdf::Pose rp = p * urdf::Pose();
      ASSERT_FALSE(is_identity(rp));
      EXPECT_EQ(p.xyz(), rp.xyz());
      EXPECT_EQ(p.rpy(), rp.rpy());
    }
    {
      urdf::Pose rp = urdf::Pose() * p;
      EXPECT_EQ(p.xyz(), rp.xyz());
      EXPECT_EQ(p.rpy(), rp.rpy());
    }
    {
      urdf::Pose rp = urdf::Pose(mexp0, zeros) * p;
      EXPECT_EQ(zeros, rp.xyz());
      EXPECT_EQ(p.rpy(), rp.rpy());
    }
  }
  {
    EXPECT_TRUE(urdf::Pose(exp0, exp1) == urdf::Pose(exp0, exp1));
    EXPECT_FALSE(urdf::Pose(exp0, exp0) == urdf::Pose(exp0, exp1));
    EXPECT_FALSE(urdf::Pose(exp1, exp1) == urdf::Pose(exp0, exp1));

    EXPECT_FALSE(urdf::Pose(exp0, exp1) != urdf::Pose(exp0, exp1));
    EXPECT_TRUE(urdf::Pose(exp0, exp0) != urdf::Pose(exp0, exp1));
    EXPECT_TRUE(urdf::Pose(exp1, exp1) != urdf::Pose(exp0, exp1));
  }
  // to_ptree
  {
    EXPECT_FALSE(urdf::Pose().to_ptree());
  }
  {
    urdf::Pose p(zeros, exp0);
    boost::optional<ptree> pt = p.to_ptree();
    ASSERT_TRUE(pt);
    EXPECT_FALSE(pt->get_child_optional("<xmlattr>.xyz"));
    EXPECT_EQ(p, urdf::Pose::from_ptree(*pt));
  }
  {
    urdf::Pose p(exp0, zeros);
    boost::optional<ptree> pt = p.to_ptree();
    ASSERT_TRUE(pt);
    EXPECT_FALSE(pt->get_child_optional("<xmlattr>.rpy"));
    EXPECT_EQ(p, urdf::Pose::from_ptree(*pt));
  }
  {
    urdf::Pose p(exp0, exp1);
    boost::optional<ptree> pt = p.to_ptree();
    ASSERT_TRUE(pt);
    EXPECT_EQ(p, urdf::Pose::from_ptree(*pt));
  }
}

TEST(Urdf, Joint) {
  ptree pt;
  EXPECT_ANY_THROW(urdf::Joint(pt).name());
  pt.put("<xmlattr>.name", "ab");
  EXPECT_EQ("ab", urdf::Joint(pt).name());

  EXPECT_ANY_THROW(urdf::Joint(pt).type());
  pt.put("<xmlattr>.type", "invalid_joint_type");
  EXPECT_ANY_THROW(urdf::Joint(pt).type());
  pt.put("<xmlattr>.type", "continuous");
  EXPECT_EQ(urdf::Joint::continuous, urdf::Joint(pt).type());

  EXPECT_ANY_THROW(urdf::Joint(pt).parent_link());
  pt.put("parent.<xmlattr>.link", "a");
  EXPECT_EQ("a", urdf::Joint(pt).parent_link());

  EXPECT_ANY_THROW(urdf::Joint(pt).child_link());
  pt.put("child.<xmlattr>.link", "b");
  EXPECT_EQ("b", urdf::Joint(pt).child_link());

  urdf::Array3d zeros = {{0, 0, 0}};
  EXPECT_EQ(zeros, urdf::Joint(pt).origin().xyz());
  EXPECT_EQ(zeros, urdf::Joint(pt).origin().rpy());

  pt.put("origin", "");
  EXPECT_EQ(zeros, urdf::Joint(pt).origin().xyz());
  EXPECT_EQ(zeros, urdf::Joint(pt).origin().rpy());

  pt.put("origin.<xmlattr>.xyz", "1. 2.2 3.3");
  urdf::Array3d exp0 = {{1, 2.2, 3.3}};
  EXPECT_EQ(exp0, urdf::Joint(pt).origin().xyz());
  EXPECT_EQ(zeros, urdf::Joint(pt).origin().rpy());

  urdf::Array3d x_axis = {{1, 0, 0}};
  EXPECT_EQ(x_axis, urdf::Joint(pt).axis());

  pt.put("axis.<xmlattr>.xyz", "1. 2.2 3.3");
  EXPECT_EQ(exp0, urdf::Joint(pt).axis());

  EXPECT_FALSE(urdf::Joint(pt).limit_lower_upper());
  pt.put("limit.<xmlattr>.lower", "2.2");
  EXPECT_EQ(std::make_pair(2.2, 0.), urdf::Joint(pt).limit_lower_upper());
  pt.put("limit.<xmlattr>.upper", "3.3");
  EXPECT_EQ(std::make_pair(2.2, 3.3), urdf::Joint(pt).limit_lower_upper());

  EXPECT_FALSE(urdf::Joint(pt).limit_effort());
  pt.put("limit.<xmlattr>.effort", "4.2");
  EXPECT_EQ(4.2, urdf::Joint(pt).limit_effort());

  EXPECT_FALSE(urdf::Joint(pt).limit_velocity());
  pt.put("limit.<xmlattr>.velocity", "2.4");
  EXPECT_EQ(2.4, urdf::Joint(pt).limit_velocity());

  EXPECT_FALSE(urdf::Joint(pt).mimic());
  pt.put("mimic.<xmlattr>.joint", "another");
  boost::optional<urdf::Mimic> m = urdf::Joint(pt).mimic();
  EXPECT_EQ("another", m->joint());
  EXPECT_TRUE(urdf::Joint(pt).mimic());
  EXPECT_EQ("another", urdf::Joint(pt).mimic()->joint());
  EXPECT_EQ(1., urdf::Joint(pt).mimic()->multiplier());
  pt.put("mimic.<xmlattr>.multiplier", "2.0");
  EXPECT_EQ(2., urdf::Joint(pt).mimic()->multiplier());
  EXPECT_EQ(0., urdf::Joint(pt).mimic()->offset());
  pt.put("mimic.<xmlattr>.offset", "0.5");
  EXPECT_EQ(0.5, urdf::Joint(pt).mimic()->offset());
}

TEST(Urdf, Inertial) {
  urdf::Array3d zeros = {{0, 0, 0}};
  ptree pt;
  EXPECT_EQ(zeros, urdf::Inertial(pt).origin().xyz());
  EXPECT_EQ(zeros, urdf::Inertial(pt).origin().rpy());
  EXPECT_ANY_THROW(urdf::Inertial(pt).mass());
  EXPECT_ANY_THROW(urdf::Inertial(pt).ixx());

  pt.put("origin.<xmlattr>.xyz", "1 2.2 3.3");
  pt.put("mass.<xmlattr>.value", "2.2");
  pt.put("inertia.<xmlattr>.ixx", "0.0");
  pt.put("inertia.<xmlattr>.ixy", "0.1");
  pt.put("inertia.<xmlattr>.ixz", "0.2");
  pt.put("inertia.<xmlattr>.iyy", "1.1");
  pt.put("inertia.<xmlattr>.iyz", "1.2");
  pt.put("inertia.<xmlattr>.izz", "2.2");
  urdf::Array3d exp0 = {{1, 2.2, 3.3}};
  EXPECT_EQ(exp0, urdf::Inertial(pt).origin().xyz());
  EXPECT_EQ(zeros, urdf::Inertial(pt).origin().rpy());
  EXPECT_EQ(2.2, urdf::Inertial(pt).mass());
  EXPECT_EQ(0.0, urdf::Inertial(pt).ixx());
  EXPECT_EQ(0.1, urdf::Inertial(pt).ixy());
  EXPECT_EQ(0.2, urdf::Inertial(pt).ixz());
  EXPECT_EQ(1.1, urdf::Inertial(pt).iyy());
  EXPECT_EQ(1.2, urdf::Inertial(pt).iyz());
  EXPECT_EQ(2.2, urdf::Inertial(pt).izz());
}

TEST(Urdf, Box) {
  ptree pt;
  EXPECT_ANY_THROW(urdf::Box(pt).size());
  urdf::Array3d exp0 = {{1, 2.2, 3.3}};
  pt.put("<xmlattr>.size", "1 2.2 3.3");
  EXPECT_EQ(exp0, urdf::Box(pt).size());
}

TEST(Urdf, Cylinder) {
  ptree pt;
  EXPECT_ANY_THROW(urdf::Cylinder(pt).radius());
  EXPECT_ANY_THROW(urdf::Cylinder(pt).length());

  pt.put("<xmlattr>.radius", "2.2");
  pt.put("<xmlattr>.length", "3.3");
  EXPECT_EQ(2.2, urdf::Cylinder(pt).radius());
  EXPECT_EQ(3.3, urdf::Cylinder(pt).length());
}

TEST(Urdf, Sphere) {
  ptree pt;
  EXPECT_ANY_THROW(urdf::Sphere(pt).radius());
  pt.put("<xmlattr>.radius", "2.2");
  EXPECT_EQ(2.2, urdf::Sphere(pt).radius());
}

TEST(Urdf, Mesh) {
  ptree pt;
  EXPECT_ANY_THROW(urdf::Mesh(pt).filename());
  urdf::Array3d ones = {{1, 1, 1}};
  EXPECT_EQ(ones, urdf::Mesh(pt).scale());

  pt.put("<xmlattr>.filename", "hello");
  EXPECT_EQ("hello", urdf::Mesh(pt).filename());

  pt.put("<xmlattr>.scale", "1 2.2 3.3");
  urdf::Array3d exp0 = {{1, 2.2, 3.3}};
  EXPECT_EQ(exp0, urdf::Mesh(pt).scale());
}

TEST(Urdf, Visual) {
  ptree pt;
  urdf::Array3d zeros = {{0, 0, 0}};
  EXPECT_EQ(zeros, urdf::Visual(pt).origin().xyz());
  EXPECT_EQ(zeros, urdf::Visual(pt).origin().rpy());
  EXPECT_ANY_THROW(urdf::Visual(pt).geometry());

  pt.put("geometry.sphere.<xmlattr>.radius", "2.2");
  urdf::Geometry g = urdf::Visual(pt).geometry();
  EXPECT_ANY_THROW(boost::get<urdf::Box>(g));

  EXPECT_NO_THROW(boost::get<urdf::Sphere>(g));
  EXPECT_EQ(2.2, boost::get<urdf::Sphere>(g).radius());
}

TEST(Urdf, Link) {
  ptree pt;
  EXPECT_ANY_THROW(urdf::Link(pt).name());

  pt.put("<xmlattr>.name", "a");
  EXPECT_EQ("a", urdf::Link(pt).name());

  EXPECT_FALSE(urdf::Link(pt).inertial());

  pt.put("inertial.mass.<xmlattr>.value", "2.2");
  EXPECT_EQ(2.2, urdf::Link(pt).inertial()->mass());

  auto visuals = urdf::Link(pt).visuals();
  EXPECT_TRUE(boost::empty(visuals));
  pt.add_child("visual", ptree()).put("geometry.sphere.<xmlattr>.radius", "2.2");
  visuals = urdf::Link(pt).visuals();
  ASSERT_EQ(1u, size(visuals));

  pt.add_child("visual", ptree()).put("geometry.mesh.<xmlattr>.filename", "hello");
  visuals = urdf::Link(pt).visuals();
  ASSERT_EQ(2u, size(visuals));

  auto it = begin(visuals);
  EXPECT_NO_THROW(boost::get<urdf::Sphere>(it->geometry()));
  ++it;
  EXPECT_NO_THROW(boost::get<urdf::Mesh>(it->geometry()));
}

TEST(Urdf, read_no_link) {
  ptree pt;
  std::unique_ptr<urdf::RobotTree> utree;
  EXPECT_ANY_THROW(utree.reset(new urdf::RobotTree(pt)));
}

TEST(Urdf, read_dupe_link) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "a");
  std::unique_ptr<urdf::RobotTree> utree;
  EXPECT_ANY_THROW(utree.reset(new urdf::RobotTree(robot)));
}

TEST(Urdf, read_dupe_joint) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "b");
  addLink(robot, "c");
  addJoint(robot, "a", "b", "j");
  addJoint(robot, "a", "c", "j");
  std::unique_ptr<urdf::RobotTree> utree;
  EXPECT_ANY_THROW(utree.reset(new urdf::RobotTree(robot)));
}

TEST(Urdf, read_non_existing_child_link) {
  ptree robot;
  addLink(robot, "a");
  addJoint(robot, "a", "b", "j");
  std::unique_ptr<urdf::RobotTree> utree;
  EXPECT_ANY_THROW(utree.reset(new urdf::RobotTree(robot)));
}

TEST(Urdf, read_non_existing_parent_link) {
  ptree robot;
  addLink(robot, "a");
  addJoint(robot, "b", "a", "j");
  std::unique_ptr<urdf::RobotTree> utree;
  EXPECT_ANY_THROW(utree.reset(new urdf::RobotTree(robot)));

}

TEST(Urdf, read_kinematic_loop_0) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "b");
  addJoint(robot, "a", "b", "ab");
  addJoint(robot, "a", "b", "ab_bis");
  std::unique_ptr<urdf::RobotTree> utree;
  EXPECT_ANY_THROW(utree.reset(new urdf::RobotTree(robot)));

}

TEST(Urdf, read_kinematic_loop_1) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "b");
  addJoint(robot, "a", "b", "ab");
  addJoint(robot, "b", "a", "ba");
  std::unique_ptr<urdf::RobotTree> utree;
  EXPECT_ANY_THROW(utree.reset(new urdf::RobotTree(robot)));
}

TEST(Urdf, read_kinematic_loop_2) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "b");
  addLink(robot, "c");
  addJoint(robot, "a", "b", "ab");
  addJoint(robot, "b", "c", "bc");
  addJoint(robot, "a", "c", "ac");
  std::unique_ptr<urdf::RobotTree> utree;
  EXPECT_ANY_THROW(utree.reset(new urdf::RobotTree(robot)));
}

TEST(Urdf, read_several_roots) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "b");
  std::unique_ptr<urdf::RobotTree> utree;
  EXPECT_ANY_THROW(utree.reset(new urdf::RobotTree(robot)));
}

TEST(Urdf, read_joint_and_link_names_do_not_collide) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "b");
  addJoint(robot, "a", "b", "b");
  std::unique_ptr<urdf::RobotTree> utree;
  EXPECT_NO_THROW(utree.reset(new urdf::RobotTree(robot)));
}

TEST(Urdf, read_mimic_self) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "b");
  addJoint(robot, "a", "b", "ab").put("mimic.<xmlattr>.joint", "ab");
  EXPECT_ANY_THROW(new urdf::RobotTree(robot));
}

TEST(Urdf, read_mimic_loop0) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "b");
  addLink(robot, "c");
  addJoint(robot, "a", "b", "ab").put("mimic.<xmlattr>.joint", "ac");
  addJoint(robot, "a", "c", "ac").put("mimic.<xmlattr>.joint", "ab");
  EXPECT_ANY_THROW(new urdf::RobotTree(robot));
}

TEST(Urdf, read_mimic_loop1) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "b");
  addLink(robot, "c");
  addLink(robot, "d");
  addJoint(robot, "a", "b", "ab").put("mimic.<xmlattr>.joint", "ad");
  addJoint(robot, "a", "c", "ac").put("mimic.<xmlattr>.joint", "ab");
  addJoint(robot, "a", "d", "ad").put("mimic.<xmlattr>.joint", "ac");
  EXPECT_ANY_THROW(new urdf::RobotTree(robot));
}

TEST(Urdf, read_mimic_nonexistent) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "b");
  addJoint(robot, "a", "b", "ab").put("mimic.<xmlattr>.joint", "nonexistent");
  EXPECT_ANY_THROW(new urdf::RobotTree(robot));
}

TEST(Urdf, is_mimic_tree_flat_empty) {
  ptree robot;
  addLink(robot, "a");
  urdf::RobotTree tree(robot);
  EXPECT_TRUE(tree.is_mimic_tree_flat());
}

TEST(Urdf, is_mimic_tree_flat_ok) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "b");
  addLink(robot, "c");
  addLink(robot, "d");
  addJoint(robot, "a", "b", "ab");
  addJoint(robot, "a", "c", "ac").put("mimic.<xmlattr>.joint", "ab");
  addJoint(robot, "a", "d", "ad").put("mimic.<xmlattr>.joint", "ab");
  urdf::RobotTree tree(robot);
  EXPECT_TRUE(tree.is_mimic_tree_flat());
}

TEST(Urdf, is_mimic_tree_flat_ko) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "b");
  addLink(robot, "c");
  addLink(robot, "d");
  addJoint(robot, "a", "b", "ab");
  addJoint(robot, "a", "c", "ac").put("mimic.<xmlattr>.joint", "ab");
  addJoint(robot, "a", "d", "ad").put("mimic.<xmlattr>.joint", "ac");
  urdf::RobotTree tree(robot);
  EXPECT_FALSE(tree.is_mimic_tree_flat());
}

TEST(Urdf, getters) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "b");
  addJoint(robot, "a", "b", "ab");
  urdf::RobotTree parser(robot);
  EXPECT_EQ("a", parser.link("a").get<std::string>("<xmlattr>.name"));
  EXPECT_EQ("b", parser.link("b").get<std::string>("<xmlattr>.name"));
  EXPECT_EQ("ab", parser.joint("ab").get<std::string>("<xmlattr>.name"));
  EXPECT_ANY_THROW(parser.link("bad name"));
  EXPECT_ANY_THROW(parser.joint("bad name"));
  EXPECT_EQ("a", parser.root_link());
}

TEST(Urdf, rm_root_joint_throw) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "b");
  addLink(robot, "c");
  addJoint(robot, "a", "b", "ab");
  addJoint(robot, "a", "c", "ac");
  urdf::RobotTree parser(robot);
  EXPECT_ANY_THROW(parser.rm_root_joint());
}

TEST(Urdf, rm_root_joint) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "b");
  addLink(robot, "c");
  addJoint(robot, "a", "b", "ab");
  addJoint(robot, "b", "c", "bc");
  urdf::RobotTree parser(robot);
  EXPECT_EQ("a", parser.root_link());
  parser.rm_root_joint();
  {
    // check the index was updated
    EXPECT_EQ("b", parser.root_link());
    EXPECT_ANY_THROW(parser.link("a"));
    EXPECT_ANY_THROW(parser.joint("ab"));
  }
  {
    // check the XML tree was updated, using a new index
    urdf::RobotTree nrtree(robot);
    EXPECT_EQ("b", nrtree.root_link());
    EXPECT_ANY_THROW(nrtree.link("a"));
    EXPECT_ANY_THROW(nrtree.joint("ab"));
  }
}

TEST(Urdf, rm_leaf_joint) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "b");
  addLink(robot, "c");
  addJoint(robot, "a", "b", "ab");
  addJoint(robot, "b", "c", "bc");
  urdf::RobotTree parser(robot);
  EXPECT_ANY_THROW(parser.rm_leaf_joint("ab"));

  parser.rm_leaf_joint("bc");

  {
    // check the index was updated
    EXPECT_NO_THROW(parser.link("a"));
    EXPECT_NO_THROW(parser.link("b"));
    EXPECT_NO_THROW(parser.joint("ab"));

    EXPECT_ANY_THROW(parser.link("c"));
    EXPECT_ANY_THROW(parser.joint("bc"));
  }
  {
    // check the XML tree was updated, using a new index
    urdf::RobotTree nrtree(robot);
    EXPECT_NO_THROW(nrtree.link("a"));
    EXPECT_NO_THROW(nrtree.link("b"));
    EXPECT_NO_THROW(nrtree.joint("ab"));

    EXPECT_ANY_THROW(nrtree.link("c"));
    EXPECT_ANY_THROW(nrtree.joint("bc"));
  }

}

class my_visitor : public urdf::JointConstVisitor {
 public:
  std::vector<std::string> joints;
  std::string stop;  // we won't walk the tree below a joint named like stop
  my_visitor() {}
  my_visitor(const std::string &stop) : stop(stop) {}

  bool discover(const ptree &joint) {
    const std::string name = joint.get<std::string>("<xmlattr>.name");
    joints.push_back(std::string("discover ") + name);
    if (name == stop) return false;
    return true;
  }

  void finish(const ptree &joint) {
    const std::string name = joint.get<std::string>("<xmlattr>.name");
    joints.push_back(std::string("finish ") + name);
  }
};

TEST(Urdf, walk_joints) {
  ptree robot;
  addLink(robot, "b");
  addLink(robot, "c");
  addLink(robot, "d");
  addLink(robot, "e");
  addLink(robot, "f");
  addLink(robot, "a");
  addJoint(robot, "b", "c", "bc");
  addJoint(robot, "c", "d", "cd");
  addJoint(robot, "d", "e", "de");
  // add prefix to joints names to ensure they are not sorted by name
  addJoint(robot, "a", "b", "y_ab");
  addJoint(robot, "a", "f", "x_af");

  urdf::RobotTree parser(robot);
  EXPECT_EQ("a", parser.root_link());
  // visitor should visit joint "bc" but not the joints below.
  my_visitor visitor("bc");
  parser.traverse_joints(visitor);
  // joints "cd" and "de" shall not be visited
  ASSERT_EQ(2u * 3u, visitor.joints.size());
  EXPECT_EQ("discover y_ab", visitor.joints[0]);
  EXPECT_EQ("discover bc", visitor.joints[1]);
  EXPECT_EQ("finish bc", visitor.joints[2]);
  EXPECT_EQ("finish y_ab", visitor.joints[3]);
  EXPECT_EQ("discover x_af", visitor.joints[4]);
  EXPECT_EQ("finish x_af", visitor.joints[5]);
}

TEST(Urdf, makeJointFixed_continuous) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "b");
  ptree &pt = addJoint(robot, "a", "b", "ab", "continuous");
  pt.put("axis.<xmlattr>.xyz", "1. 2.2 3.3");
  pt.put("limit.<xmlattr>.effort", "1.");
  urdf::RobotTree parser(robot);
  urdf::makeJointFixed(parser, "ab");
  EXPECT_EQ("fixed", pt.get<std::string>("<xmlattr>.type"));
  EXPECT_EQ(0u, pt.count("axis"));
  EXPECT_EQ(0u, pt.count("limit"));
}

TEST(Urdf, makeJointFixed_floating) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "b");
  ptree &pt = addJoint(robot, "a", "b", "ab", "floating");
  urdf::RobotTree parser(robot);
  urdf::makeJointFixed(parser, "ab");
  EXPECT_EQ("fixed", pt.get<std::string>("<xmlattr>.type"));
  EXPECT_EQ(0u, pt.count("axis"));
  EXPECT_EQ(0u, pt.count("limit"));
}

TEST(Urdf, makeContinuousJointsFixed) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "b");
  ptree &pt = addJoint(robot, "a", "b", "ab", "continuous");
  pt.put("axis.<xmlattr>.xyz", "1. 2.2 3.3");
  urdf::RobotTree parser(robot);
  std::vector<std::string> names = urdf::makeContinuousJointsFixed(parser);
  EXPECT_EQ("fixed", pt.get<std::string>("<xmlattr>.type"));
  EXPECT_EQ(0u, pt.count("axis"));
  EXPECT_EQ(std::vector<std::string>({"ab"}), names);
}

TEST(Urdf, squashJointMass_no_mass) {
  ptree robot;
  ptree &a = addLink(robot, "a");
  ptree &b = addLink(robot, "b");
  addJoint(robot, "a", "b", "ab", "fixed");
  urdf::RobotTree parser(robot);
  squashJointMass(parser, "ab");
  EXPECT_EQ(0u, a.count("inertial"));
  EXPECT_EQ(0u, b.count("inertial"));
}

TEST(Urdf, squashJointMass_massless_parent) {
  ptree robot;
  ptree &a = addLink(robot, "a");
  ptree &b = addLink(robot, "b", 1.);
  b.put("inertial.origin.<xmlattr>.xyz", "1 0 0");
  b.put("inertial.origin.<xmlattr>.rpy", "0.1 0 0");
  ptree &ab = addJoint(robot, "a", "b", "ab", "fixed");
  ab.put("origin.<xmlattr>.xyz", "10 0 0");
  ab.put("origin.<xmlattr>.rpy", "1 0 0");
  urdf::RobotTree parser(robot);
  squashJointMass(parser, "ab");
  EXPECT_EQ(1u, a.count("inertial"));
  EXPECT_EQ("1", a.get<std::string>("inertial.mass.<xmlattr>.value"));
  EXPECT_EQ("1", a.get<std::string>("inertial.inertia.<xmlattr>.ixx"));
  EXPECT_EQ("11 0 0", a.get<std::string>("inertial.origin.<xmlattr>.xyz"));
  EXPECT_EQ("1.0999999999999999 -0 0",
            a.get<std::string>("inertial.origin.<xmlattr>.rpy"));
  EXPECT_EQ(0u, b.count("inertial"));
}

TEST(Urdf, squashJointMass_mass) {
  // this model is defined such that the two center of mass are at the same
  // point.
  ptree robot;
  ptree &a = addLink(robot, "a", 1.);
  a.put("inertial.origin.<xmlattr>.xyz", "1 0 0");
  ptree &b = addLink(robot, "b", 10.);
  b.put("inertial.origin.<xmlattr>.xyz", "10 0 0");
  ptree &ab = addJoint(robot, "a", "b", "ab", "fixed");
  ab.put("origin.<xmlattr>.xyz", "-9 0 0");
  urdf::RobotTree parser(robot);
  squashJointMass(parser, "ab");
  EXPECT_EQ(1u, a.count("inertial"));
  EXPECT_EQ("11", a.get<std::string>("inertial.mass.<xmlattr>.value"));
  EXPECT_EQ("11", a.get<std::string>("inertial.inertia.<xmlattr>.ixx"));
  EXPECT_EQ("1 0 0", a.get<std::string>("inertial.origin.<xmlattr>.xyz"));
  EXPECT_EQ(0u, a.count("inertial.origin.<xmlattr>.rpy"));
  EXPECT_EQ(0u, b.count("inertial"));
}

TEST(Urdf, squashFixedJointsMass_wheels) {
  // create a model describing a Juliette base with wheels.
  ptree robot;
  urdf::Array3d xyz = {{0.00220451, 0, -0.185729}};
  urdf::Array3d rpy = {{0, 0, 0}};
  ptree &knee_pt =
      addLink(robot, "KneePitch_link", 11.6015, 0.129229, 0.000401632,
              -0.0168449, 0.122839, -1.01666e-08, 0.108168, xyz, rpy);
  std::array<std::string, 3> wheels = {
      {"WheelB_link", "WheelFL_link", "WheelFR_link"}};
  for (const std::string &wheel : wheels) {
    addLink(robot, wheel, 1.58028, 0.00299756, -6.3068e-07, -5.56903e-06,
            0.00303229, 9.0408e-07, 0.00296585);
  }
  xyz = {{-0.17, 0, -0.264}};
  rpy = {{0, 0, -3.14159}};
  addJoint(robot, "KneePitch_link", "WheelB_link", "WheelB_joint", "fixed", xyz,
           rpy);

  xyz = {{0.09, 0.155, -0.264}};
  rpy = {{0, -0, 1.07517}};
  addJoint(robot, "KneePitch_link", "WheelFL_link", "WheelFL_joint", "fixed",
           xyz, rpy);

  xyz = {{0.09, -0.155, -0.264}};
  rpy = {{0, 0, -1.07517}};
  addJoint(robot, "KneePitch_link", "WheelFR_link", "WheelFR_joint", "fixed",
           xyz, rpy);

  // the squash the wheels inertia
  urdf::RobotTree result(robot);
  squashFixedJointsMass(result);

  // and check the resulting base inertia
  urdf::Inertial knee = *urdf::Link(knee_pt).inertial();
  double eps = 1e-8;
  EXPECT_DOUBLE_EQ(16.34234, knee.mass());
  EXPECT_NEAR(0.234826379, knee.ixx(), eps);
  EXPECT_NEAR(0.00040169217, knee.ixy(), eps);
  EXPECT_NEAR(-0.016547268, knee.ixz(), eps);
  EXPECT_NEAR(0.22372286, knee.iyy(), eps);
  EXPECT_NEAR(-6.27106e-08, knee.iyz(), eps);
  EXPECT_NEAR(0.26422024, knee.izz(), eps);
  xyz = {{0.00253198, 9.11813e-10, -0.208435}};
  EXPECT_NEAR(0.00253198, knee.origin().xyz()[0], eps);
  EXPECT_NEAR(9.11813e-10, knee.origin().xyz()[1], eps);
  EXPECT_NEAR(-0.20843507, knee.origin().xyz()[2], eps);
  rpy = {{0, 0, 0}};
  EXPECT_EQ(rpy, knee.origin().rpy());
}

TEST(Urdf, makeMasslessJointsFixed) {
  // this model is defined such that the two center of mass are at the same
  // point.
  ptree robot;
  ptree &a = addLink(robot, "a", 1.);
  ptree &b = addLink(robot, "b", 1.);
  ptree &c = addLink(robot, "c");
  ptree &d = addLink(robot, "d", 1.);
  ptree &e = addLink(robot, "e");
  ptree &ab = addJoint(robot, "a", "b", "ab", "floating");
  ptree &bc = addJoint(robot, "b", "c", "bc", "floating");
  ptree &cd = addJoint(robot, "c", "d", "cd", "fixed");
  ptree &de = addJoint(robot, "d", "e", "de", "floating");
  urdf::RobotTree parser(robot);
  std::vector<std::string> names = makeMasslessJointsFixed(parser);

  // mass moved from link "d" to link "c"
  EXPECT_EQ(1u, a.count("inertial"));
  EXPECT_EQ(1u, b.count("inertial"));
  EXPECT_EQ(1u, c.count("inertial"));
  EXPECT_EQ(0u, d.count("inertial"));
  EXPECT_EQ(0u, e.count("inertial"));

  // joint "de" was converted to fixed
  EXPECT_EQ(urdf::Joint::floating, urdf::Joint(ab).type());
  EXPECT_EQ(urdf::Joint::floating, urdf::Joint(bc).type());
  EXPECT_EQ(urdf::Joint::fixed, urdf::Joint(cd).type());
  EXPECT_EQ(urdf::Joint::fixed, urdf::Joint(de).type());

  EXPECT_EQ(std::vector<std::string>({"de"}), names);
}

TEST(Urdf, removeSubTreeIfJoint) {
  ptree robot;

  addLink(robot, "a");
  addLink(robot, "b");
  addLink(robot, "c");
  addLink(robot, "d");
  addLink(robot, "e");
  addJoint(robot, "a", "b", "ab_useless");
  addJoint(robot, "b", "c", "bc_useless");
  addJoint(robot, "b", "d", "bd");
  addJoint(robot, "a", "e", "ae");

  urdf::RobotTree parser(robot);
  auto pred = [](const ptree &joint) {
    return boost::regex_match(urdf::name(joint), boost::regex(".*_useless$"));
  };
  auto names = removeSubTreeIfJoint(parser, pred);

  EXPECT_ANY_THROW(parser.joint("ab_useless")); // removed because it matches
  EXPECT_ANY_THROW(parser.link("b"));
  EXPECT_ANY_THROW(parser.joint("bc_useless")); // removed because it matches
  EXPECT_ANY_THROW(parser.link("c"));
  EXPECT_ANY_THROW(parser.joint("bd")); // removed because an ancestor matches
  EXPECT_ANY_THROW(parser.link("d"));
  EXPECT_NO_THROW(parser.joint("ae")); // no removed
  EXPECT_NO_THROW(parser.link("e"));

  EXPECT_EQ(std::vector<std::string>({"ab_useless", "bc_useless", "bd"}),
            names);
}

TEST(Urdf, dot_printer) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "b");
  addLink(robot, "c");
  addLink(robot, "d");
  addJoint(robot, "a", "b", "ab");
  addJoint(robot, "b", "c", "bc");
  addJoint(robot, "a", "d", "ad");
  urdf::RobotTree parser(robot);
  EXPECT_EQ("a", parser.root_link());
  std::ostringstream ss;
  {
    urdf::UrdfDotPrinterVisitor visitor(ss);
    parser.traverse_joints(boost::ref(visitor));
  }
  // note that the root link is not listed in the output
  std::string exp =
      "a -> b [label=ab];\n"
      "b -> c [label=bc];\n"
      "a -> d [label=ad];\n";
  EXPECT_EQ(exp, ss.str());
  if (false) {
    // create a valid .dot file
    std::ofstream f("/tmp/urdf_dot_printer.dot");
    f << "digraph g {\n" << ss.str() << "}";
    // then display it with
    // dot -Tpdf -O /tmp/urdf_dot_printer.dot
    // evince /tmp/urdf_dot_printer.dot.pdf
  }
}

TEST(Urdf, define_as_root_link_ko_floating) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "b");
  addLink(robot, "c");
  addJoint(robot, "a", "b", "ab", "floating");
  addJoint(robot, "b", "c", "bc", "revolute");
  urdf::RobotTree parser(robot);
  EXPECT_ANY_THROW(parser.define_as_root_link("c"));
}

TEST(Urdf, define_as_root_link) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "b");  // initial root
  addLink(robot, "c");
  addLink(robot, "d");  // new root
  addLink(robot, "e");
  addJoint(robot, "b", "a", "ba", "revolute");
  addJoint(robot, "b", "c", "bc", "revolute");
  addJoint(robot, "c", "d", "cd", "revolute");
  addJoint(robot, "d", "e", "de", "revolute");

  urdf::RobotTree parser(robot);

  parser.define_as_root_link("d");

  EXPECT_EQ("d", parser.root_link());

  // unchanged
  urdf::Joint ba(parser.joint("ba"));
  EXPECT_EQ("b", ba.parent_link());
  EXPECT_EQ("a", ba.child_link());

  // got flipped
  urdf::Joint bc(parser.joint("bc"));
  EXPECT_EQ("c", bc.parent_link());
  EXPECT_EQ("b", bc.child_link());

  // got flipped
  urdf::Joint cd(parser.joint("cd"));
  EXPECT_EQ("d", cd.parent_link());
  EXPECT_EQ("c", cd.child_link());

  // unchanged
  urdf::Joint de(parser.joint("de"));
  EXPECT_EQ("d", de.parent_link());
  EXPECT_EQ("e", de.child_link());

  // to be sure the boost multiindex of joints got
  // updated, let traverse the kinematic tree
  {
    std::ostringstream ss;
    {
      urdf::UrdfDotPrinterVisitor visitor(ss);
      parser.traverse_joints(boost::ref(visitor));
    }

    // note that the root link is not listed in the output
    std::string exp =
        "d -> c [label=cd];\n"
        "c -> b [label=bc];\n"
        "b -> a [label=ba];\n"
        "d -> e [label=de];\n";
    EXPECT_EQ(exp, ss.str());
  }
}

TEST(Urdf, define_as_root_link_axis_unitx) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "b");
  ptree &ab = addJoint(robot, "a", "b", "ab", "revolute");

  urdf::RobotTree parser(robot);
  parser.define_as_root_link("b");

  urdf::Array3d flipped_axis{{-1, 0, 0}};
  EXPECT_EQ(flipped_axis, urdf::Joint(ab).axis());
}

TEST(Urdf, define_as_root_link_axis_munitx) {
  ptree robot;
  addLink(robot, "a");
  addLink(robot, "b");
  ptree &ab = addJoint(robot, "a", "b", "ab", "revolute");
  urdf::Array3d axis{{-1, 0, 0}};
  ab.put("axis.<xmlattr>.xyz", axis, urdf::Array3dTranslator());

  urdf::RobotTree parser(robot);
  parser.define_as_root_link("b");

  urdf::Array3d flipped_axis{{1, 0, 0}};
  EXPECT_EQ(flipped_axis, urdf::Joint(ab).axis());
  EXPECT_FALSE(ab.get_child_optional("axis"));
}

TEST(Urdf, define_as_root_link_transport) {
  ptree robot;
  urdf::Array3d zero = {{0, 0, 0}};
  urdf::Array3d xyz_ia = {{1, 2, 3}};
  urdf::Array3d xyz_ib = {{4, 5, 6}};
  urdf::Array3d xyz_j = {{10, 20, 30}};
  urdf::Pose tr(xyz_ia, zero);
  urdf::Pose itr = tr.inverse();
  ptree &a = addLink(robot, "a", 1, 1, 0, 0, 1, 0, 1, xyz_ia, zero);
  ptree &b = addLink(robot, "b", 1, 1, 0, 0, 1, 0, 1, xyz_ib, zero);
  ptree &ab = addJoint(robot, "a", "b", "ab", "revolute", xyz_j, zero);

  urdf::RobotTree parser(robot);
  parser.define_as_root_link("b");

  // link b's frame is unchanged
  urdf::Pose tr_ib{xyz_ib, zero};
  EXPECT_EQ(tr_ib, urdf::Link(b).inertial()->origin());

  // link a's frame has moved
  urdf::Pose tr_j = urdf::Pose{xyz_j, zero};
  urdf::Pose tr_ia_new = tr_j.inverse() * urdf::Pose{xyz_ia, zero};
  urdf::Array3d xyz_ia_new{{-9, -18, -27}};
  EXPECT_EQ(xyz_ia_new, tr_ia_new.xyz());
  EXPECT_EQ(zero, tr_ia_new.rpy());
  EXPECT_EQ(tr_ia_new, urdf::Link(a).inertial()->origin());

  // joint ab
  urdf::Pose identity{};
  EXPECT_EQ(identity, urdf::Joint(ab).origin());
}

TEST(Urdf, transport_root_link_frame) {
  ptree robot;
  urdf::Array3d xyz = {{1, 2, 3}};
  urdf::Array3d rpy = {{0, 0, 0.5}};
  urdf::Pose tr(xyz, rpy);
  urdf::Pose itr = tr.inverse();

  urdf::Pose identity;

  ptree &a = addLink(robot, "a", 1, 1, 0, 0, 1, 0, 1, xyz, rpy);
  addLink(robot, "b");
  ptree &ab = addJoint(robot, "a", "b", "ab", "revolute");
  a.put("visual.origin.<xmlattr>.xyz", "1 2 3");
  a.put("collision.origin.<xmlattr>.xyz", "1 2 3");

  urdf::RobotTree parser(robot);
  parser.transport_root_link_frame(tr);

  EXPECT_EQ(identity, urdf::Link(a).inertial()->origin());
  EXPECT_FALSE(urdf::Link(a).inertial()->pt.get_child_optional("origin"));

  EXPECT_EQ(itr, urdf::Pose(urdf::Joint(ab).origin()));

  urdf::Pose exp = itr * urdf::Pose{xyz, {{0, 0, 0}}};
  EXPECT_EQ(exp, urdf::Pose::from_ptree(a.get_child("visual.origin")));
  EXPECT_EQ(exp, urdf::Pose::from_ptree(a.get_child("collision.origin")));
}

TEST(Urdf, transform_filenames) {
  auto lmpath = "link.visual.geometry.mesh.<xmlattr>.filename";
  auto lmbefore = "file:///usr/share/local/juliette/KneePitch.mesh";
  auto lmafter = "file:///juliette/KneePitch.dae";

  auto rtpath = "material.texture.<xmlattr>.filename";
  auto ltpath = "link.visual.material.texture.<xmlattr>.filename";
  auto tbefore = "file:///usr/share/local/mytexture.png";
  auto tafter = "file:///mytexture.png";

  ptree robot;
  robot.put(lmpath, lmbefore);
  robot.put(rtpath, tbefore);
  robot.put(ltpath, tbefore);

  auto op = [](std::string in) {
    // Beware of double '\' escaping: one for C++, one for regex
    // Note: I don't know why I need to escape the ':' in the format
    return boost::regex_replace(
          in,
          boost::regex("(^file:///usr/share/local/)|(\\.mesh$)"),
          "(?1file\\:///)(?2\\.dae)",
          boost::match_default | boost::format_all);
  };

  urdf::robot::transform_filenames(robot, op);

  EXPECT_EQ(lmafter, robot.get<std::string>(lmpath));
  EXPECT_EQ(tafter, robot.get<std::string>(rtpath));
  EXPECT_EQ(tafter, robot.get<std::string>(ltpath));
}

TEST(Urdf, transform_filenames_tolerate_the_ros_parser_output) {
  // check we're tolerant with the ros parser which creates "illegal"
  // texture elements with no filename
  ptree robot;
  robot.put_child("material.texture", ptree());
  robot.put_child("link.visual.material.texture", ptree());
  robot.put_child("link.visual.geometry", ptree());
  auto op = [](std::string in) { return in; };
  EXPECT_NO_THROW(urdf::robot::transform_filenames(robot, op));
}

TEST(Urdf, transform_filenames_example) {
  auto lmpath = "link.visual.geometry.mesh.<xmlattr>.filename";
  auto lmbefore = "file:///usr/share/local/juliette/KneePitch.mesh";
  auto lmafter = "file:///usr/share/local/juliette/KneePitch.dae";

  ptree robot;
  robot.put(lmpath, lmbefore);

  auto op = [](std::string in) {
    // Beware of double '\' escaping: one for C++, one for regex
    return boost::regex_replace(in, boost::regex("\\.mesh$"), ".dae");
  };
  urdf::robot::transform_filenames(robot, op);

  EXPECT_EQ(lmafter, robot.get<std::string>(lmpath));
}

TEST(Urdf, xml_altering_example) {
  // get an istream to xml content
  std::stringstream is(
        "<robot name=\"my robot\">\n"
        "  <link name=\"Torso_link\"/>\n"
        "  <link name=\"HeadYaw_link\"/>\n"
        "  <joint name=\"HeadYaw\" type=\"revolute\">\n"
        "    <origin xyz=\"0 0 0.1265\" rpy=\"0 0 0\" />\n"
        "    <axis xyz=\"0 0 1\" />\n"
        "    <parent link=\"Torso_link\" />\n"
        "    <child link=\"HeadYaw_link\" />\n"
        "    <limit effort=\"1.547\" velocity=\"8.26797\" lower=\"-2.08567\" upper=\"2.08567\" />\n"
        "  </joint>\n"
        "</robot>\n");
  // parse it to a ptree
  boost::property_tree::ptree root;
  boost::property_tree::read_xml(is, root, boost::property_tree::xml_parser::trim_whitespace);
  // find the robot node
  boost::property_tree::ptree &robot = root.get_child("robot");
  // create the kinematic tree index
  urdf::RobotTree robotTree(robot);
  // use the index to alter the ptree in memory
  urdf::makeJointFixed(robotTree, "HeadYaw");
  // write the modified ptree back as xml
  boost::property_tree::xml_writer_settings<std::string> settings(' ', 4);
  boost::property_tree::write_xml(std::cout, root, settings);
}

TEST_F(RigidBodySystemBuilderTest, buildFromUrdf_none) {
  ptree pt;
  ptree &robot = addRobot(pt);
  addLink(robot, "a", 1.);

  buildRigidBodySystemFromUrdf(f, pt);
  ASSERT_EQ(1u, b.links.size());
  EXPECT_EQ("a", b.links[0].new_body);
  EXPECT_EQ(JointType::FreeFlyer, b.links[0].joint_type);
  ASSERT_EQ(0u, b.staticframes.size());
}

TEST_F(RigidBodySystemBuilderTest, buildFromUrdf_floating) {
  ptree pt;
  ptree &robot = addRobot(pt);
  addLink(robot, "a", 1.);
  addLink(robot, "b", 2.);
  addJoint(robot, "a", "b", "ab", "floating");

  buildRigidBodySystemFromUrdf(f, pt);
  ASSERT_EQ(2u, b.links.size());
  EXPECT_EQ("b", b.links[1].new_body);
  EXPECT_EQ(JointType::FreeFlyer, b.links[1].joint_type);
  ASSERT_EQ(0u, b.staticframes.size());
}

TEST_F(RigidBodySystemBuilderTest, buildFromUrdf_rx) {
  ptree pt;
  ptree &robot = addRobot(pt);
  addLink(robot, "a", 1.);
  addLink(robot, "b", 2.);
  addJoint(robot, "a", "b", "ab", "revolute")
      .put("axis.<xmlattr>.xyz", "1 0 0");

  buildRigidBodySystemFromUrdf(f, pt);
  ASSERT_EQ(2u, b.links.size());
  EXPECT_EQ("b", b.links[1].new_body);
  EXPECT_EQ(JointType::Rx, b.links[1].joint_type);
  ASSERT_EQ(0u, b.staticframes.size());
}

TEST_F(RigidBodySystemBuilderTest, buildFromUrdf_ry) {
  ptree pt;
  ptree &robot = addRobot(pt);
  addLink(robot, "a", 1.);
  addLink(robot, "b", 2.);
  addJoint(robot, "a", "b", "ab", "revolute")
      .put("axis.<xmlattr>.xyz", "0 1 0");

  buildRigidBodySystemFromUrdf(f, pt);
  ASSERT_EQ(2u, b.links.size());
  EXPECT_EQ("b", b.links[1].new_body);
  EXPECT_EQ(JointType::Ry, b.links[1].joint_type);
  ASSERT_EQ(0u, b.staticframes.size());
}

TEST_F(RigidBodySystemBuilderTest, buildFromUrdf_rz) {
  ptree pt;
  ptree &robot = addRobot(pt);
  addLink(robot, "a", 1.);
  addLink(robot, "b", 2.);
  addJoint(robot, "a", "b", "ab", "revolute")
      .put("axis.<xmlattr>.xyz", "0 0 1");

  buildRigidBodySystemFromUrdf(b, pt);
  ASSERT_EQ(2u, b.links.size());
  EXPECT_EQ("b", b.links[1].new_body);
  EXPECT_EQ(JointType::Rz, b.links[1].joint_type);
  ASSERT_EQ(0u, b.staticframes.size());
}

TEST_F(RigidBodySystemBuilderTest, buildFromUrdf_unsupported_axis) {
  ptree pt;
  ptree &robot = addRobot(pt);
  addLink(robot, "a", 1.);
  addLink(robot, "b", 2.);
  addJoint(robot, "a", "b", "ab", "revolute")
      .put("axis.<xmlattr>.xyz", "1 1 1");
  EXPECT_ANY_THROW(buildRigidBodySystemFromUrdf(f, pt));
}

TEST_F(RigidBodySystemBuilderTest, buildFromUrdf_fixed) {
  ptree pt;
  ptree &robot = addRobot(pt);
  addLink(robot, "a", 1.);
  addLink(robot, "b", 2.);
  addLink(robot, "c");
  addJoint(robot, "a", "b", "ab", "fixed");     // "fixed"
  addJoint(robot, "b", "c", "bc", "floating");  // converted to "fixed"

  buildRigidBodySystemFromUrdf(f, pt);

  ASSERT_EQ(1u, b.links.size());
  EXPECT_EQ("a", b.links[0].new_body);
  EXPECT_EQ(JointType::FreeFlyer, b.links[0].joint_type);
  ASSERT_EQ(2u, b.staticframes.size());
  EXPECT_EQ("b", b.staticframes[0].new_static_frame);
  EXPECT_EQ("c", b.staticframes[1].new_static_frame);
}

TEST_F(RigidBodySystemBuilderTest, buildFromUrdf_fixed_parent) {
  ptree pt;
  ptree &robot = addRobot(pt);
  addLink(robot, "a", 1.);
  addLink(robot, "b");
  addLink(robot, "c", 2.);
  addJoint(robot, "a", "b", "ab", "fixed");
  addJoint(robot, "b", "c", "bc", "floating");  // problem: "fixed" parent

  EXPECT_ANY_THROW(buildRigidBodySystemFromUrdf(f, pt));

  ASSERT_EQ(1u, b.links.size());
  EXPECT_EQ("a", b.links[0].new_body);
  EXPECT_EQ(JointType::FreeFlyer, b.links[0].joint_type);
  ASSERT_EQ(1u, b.staticframes.size());
  EXPECT_EQ("b", b.staticframes[0].new_static_frame);
}

TEST_F(RigidBodySystemBuilderTest, buildFromUrdf_unsupported_type) {
  ptree pt;
  ptree &robot = addRobot(pt);
  addLink(robot, "a", 1.);
  addLink(robot, "b", 2.);
  addJoint(robot, "a", "b", "ab", "planar");
  EXPECT_ANY_THROW(buildRigidBodySystemFromUrdf(f, pt));
}

TEST_F(RigidBodySystemBuilderTest, buildFromUrdf_link) {
  ptree pt;
  ptree &robot = addRobot(pt);
  addLink(robot, "a", 1.);
  buildRigidBodySystemFromUrdf(f, pt);
  ASSERT_EQ(1u, b.links.size());
  EXPECT_EQ("", b.links[0].parent_body);
  EXPECT_EQ("a", b.links[0].new_body);
  EXPECT_EQ(1., b.links[0].body_mass.mass);
  EXPECT_EQ(MyBuilder::Vector3::Zero(), b.links[0].body_mass.center_of_mass);
  EXPECT_EQ(MyBuilder::Matrix3::Ones(),
            b.links[0].body_mass.rotational_inertia);
}
