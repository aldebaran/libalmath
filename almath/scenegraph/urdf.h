/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */

#ifndef LIB_ALMATH_SCENEGRAPH_URDF_H
#define LIB_ALMATH_SCENEGRAPH_URDF_H

#include <string>
#include <iosfwd>
#include <memory>
#include <array>
#include <boost/property_tree/ptree_fwd.hpp>
#include <boost/optional.hpp>
#include <boost/function/function_fwd.hpp>
#include <boost/ref.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/mem_fun.hpp>
#include <boost/multi_index/global_fun.hpp>
#include <boost/multi_index/sequenced_index.hpp>
#include <set>
#include <vector>

#include <boost/property_tree/stream_translator.hpp>

namespace boost {
namespace property_tree {

// hack around ptree bug #10188 [1]: for floating point numbers they don't
// serialize with enough precision:
// they use digits10+1 instead of max_digits10
// [1] https://svn.boost.org/trac/boost/ticket/10188
template <typename Ch, typename Traits>
struct customize_stream<Ch, Traits, double, void> {
  static void insert(std::basic_ostream<Ch, Traits> &s, const double &e) {
    s.precision(std::numeric_limits<double>::max_digits10);
    s << e;
  }
  static void extract(std::basic_istream<Ch, Traits> &s, double &e) {
    s >> e;
    if (!s.eof()) {
      s >> std::ws;
    }
  }
};
}
}

namespace AL {

namespace urdf {

typedef boost::property_tree::ptree ptree;

namespace detail {
class UrdfTreeP;
}

// return the name attribute of a joint or link element
std::string name(const ptree &pt);

// return the parent link of a joint element
std::string parent_link(const ptree &pt);

// return the child link of a joint element
std::string child_link(const ptree &pt);

// a parser for urdf xml files [1].
//
// Contrary to the urdf parser from ROS [2], [3], this parser:
//
// * preserves joint ordering defined in XML file. This is useful if one
//   wants to use this ordering to define the joint indexes in a
//   state vector.
//
// * provides access to the raw XML elements, so that you can leverage the
//   parser to handle unofficial URDF extensions and
//   to modify the XML tree and write it back while altering the document as
//   little as possible.
//
// * does not systematically convert roll-pitch-yaw angles to quaternions,
//   which saves us some bugs and better fits the "alter the document as
//   little as possible" philosophy.
//
//  [1] http://wiki.ros.org/urdf/XML
//  [2] https://github.com/ros/robot_model
//  [3] https://github.com/ros/urdfdom_headers
//
// The UrdfTree class acts as an index for the boost::property_tree of the
// (URDF) XML document.
// Given the ptree of the root XML element, it walks the document and
// indexes the ptree elements for the URDF joints and links.
//
// The class is akin to a set of iterators: it does not copy nor own the
// XML tree, and may be invalidated if the XML tree changes.
//
// The user shall ensure that the XML root and the joint and link ptree
// elements stays alive and valid for the lifetime of this UrdfTree.
// For instance the user may use direct access to the ptree elements to alter
// the mass of a link, but shall no delete the link, otherwise calls to
// UrdfTree::link("my_link_name") would use dandling pointers and return
// invalid references.
//
// On the other hand, some UrdfTree functions enable the safe removal of
// joints and links.
class UrdfTree {
 public:
  // read a XML tree given the XML root element.
  // The XML tree is not copied, the user shall ensure that the reference
  // is valid for the whole life of the UrdfTree object.
  UrdfTree(ptree &pt);
  ~UrdfTree();
  UrdfTree(UrdfTree const &) = delete;
  void operator=(UrdfTree const &other) = delete;

  const ptree &link(const std::string &name) const;
  ptree &link(const std::string &name);

  const ptree &joint(const std::string &name) const;
  ptree &joint(const std::string &name);

  const std::string &root_link() const;

  void rm_root_joint();

  class JointConstVisitor {
   public:
    // a false return value stops the walk for the current branch
    virtual bool discover(const ptree &) { return true; }
    virtual void finish(const ptree &) {}
  };
  class JointVisitor {
   public:
    virtual bool discover(ptree &) { return true; }
    virtual void finish(ptree &) {}
  };

  // Traverse the kinematic tree with a depth first traversal.
  // Siblings joints are visited using the ordering from the urdf file.
  void traverse_joints(JointConstVisitor &visitor) const;
  void traverse_joints(JointVisitor &visitor);

 private:
  std::unique_ptr<detail::UrdfTreeP> _p;
};

typedef UrdfTree::JointConstVisitor JointConstVisitor;
typedef UrdfTree::JointVisitor JointVisitor;

// Convenience wrapper classes around URDF ptree elements

typedef std::array<double, 3> Array3d;

// helper to convert ptree to/from Array3d
struct Array3dTranslator {
  typedef std::string internal_type;
  typedef std::array<double, 3> external_type;

  boost::optional<external_type> get_value(const internal_type &str);
  boost::optional<internal_type> put_value(const external_type &v);
};

// Convenience wrapper around an URDF pose/transform (origin XML element)
class Pose {
 public:
  boost::optional<const ptree &> pt;
  Pose();
  Pose(const ptree &pt);
  Pose(boost::optional<const ptree &> pt);
  Array3d xyz() const;
  Array3d rpy() const;
};

// Convenience wrapper around an URDF joint XML element
class Joint {
 public:
  enum Type { revolute, continuous, prismatic, fixed, floating, planar };
  const ptree &pt;
  Joint(const ptree &pt);
  std::string name() const;
  Type type() const;
  std::string parent_link() const;
  std::string child_link() const;
  Pose origin() const;
  Array3d axis() const;
};

// Convenience wrapper around an URDF inertial XML element
class Inertial {
 public:
  const ptree &pt;
  Inertial(const ptree &pt);
  Pose origin() const;
  double mass() const;
  double ixx() const;
  double ixy() const;
  double ixz() const;
  double iyy() const;
  double iyz() const;
  double izz() const;
};

// Convenience wrapper around an URDF link XML element
class Link {
 public:
  const ptree &pt;
  Link(const ptree &pt);
  std::string name() const;
  boost::optional<Inertial> inertial() const;
};

// Utils

// make the type of the joint named "name" equal to "fixed",
// and erase the joint axis, if any.
void makeJointFixed(UrdfTree &parser, const std::string &name);

// make the type of the joint named "name" equal to "floating",
// and erase the joint axis, if any.
void makeJointFloating(UrdfTree &parser, const std::string &name);

// apply makeJointFixed to all joints of type "continuous".
// Return the names of the joints whose type was changed.
std::vector<std::string> makeContinuousJointsFixed(UrdfTree &parser);

// Squash a joint's child link mass into its parent link mass.
// The child link mass element (if any) is erased.
void squashJointMass(UrdfTree &parser, const std::string &name);

// apply squashJointMass to all fixed joints.
void squashFixedJointsMass(UrdfTree &parser);

// Squash all fixed joint's child link mass into their parent link mass and
// then convert all massless joints into fixed joints.
// The point is to avoid massless mobile joints, which have no physical
// meaning.
// Return the names of the joints whose type was changed.
std::vector<std::string> makeMasslessJointsFixed(UrdfTree &parser);

// a visitor which prints the URDF kinematic tree as a dot graph
// when visiting its joints.
class UrdfDotPrinterVisitor : public UrdfTree::JointConstVisitor {
  const char tab;
  int depth;
  bool do_indent;
  std::ostream &os;

 public:
  UrdfDotPrinterVisitor(std::ostream &os);

 public:
  bool discover(const ptree &joint);
  void finish(const ptree &joint);
};
}
}
#endif
