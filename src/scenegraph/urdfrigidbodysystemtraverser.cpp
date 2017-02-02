/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */
#include <almath/scenegraph/urdfrigidbodysystemtraverser.h>
#include <almath/scenegraph/urdfeigen.h>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

namespace Builder = AL::RigidBodySystemBuilder;
typedef Builder::Interface<double>::Pose Pose;
typedef Builder::Interface<double>::BodyMass BodyMass;

namespace AL {

static const urdf::Array3d unit_x = {{1., 0., 0.}};
static const urdf::Array3d unit_y = {{0., 1., 0.}};
static const urdf::Array3d unit_z = {{0., 0., 1.}};

class UrdfRigidBodySystemTraverser : public urdf::RobotTree::JointConstVisitor {
 public:
  UrdfRigidBodySystemTraverser(Builder::Interface<double> &builder,
                               const urdf::RobotTree &parser)
      : builder(builder), parser(parser), removed_root_link(false) {}

  bool _discover(const std::string &parent_link_name,
                 const std::string &link_name, const Pose &origin,
                 Builder::JointType type, const urdf::ptree &link_pt,
                 const boost::optional<std::string> &joint_name);
  bool discover(const urdf::ptree &joint_el);

 private:
  Builder::Interface<double> &builder;
  const urdf::RobotTree &parser;
  boost::optional<std::string> urdf_root_link;
  bool removed_root_link;
};

bool UrdfRigidBodySystemTraverser::_discover(
    const std::string &parent_link_name, const std::string &link_name,
    const Pose &origin, Builder::JointType type, const urdf::ptree &link_pt,
    const boost::optional<std::string> &joint_name =
        boost::optional<std::string>()) {
  urdf::Link link(link_pt);
  if (!link.inertial()) return false;  // stop exploring this branch
  BodyMass mass;
  Math::urdfInertialToEigen(*(link.inertial()), mass.mass, mass.center_of_mass,
                            mass.rotational_inertia);
  if (joint_name)
    builder.add(parent_link_name, link_name, origin, type, mass, *joint_name);
  else
    builder.add(parent_link_name, link_name, origin, type, mass);
  return true;
}

bool UrdfRigidBodySystemTraverser::discover(const urdf::ptree &joint_el) {
  urdf::Joint joint(joint_el);
  Builder::JointType type;
  switch (joint.type()) {
    case urdf::Joint::fixed: {
      builder.add(joint.parent_link(), joint.child_link(),
                  Math::toEigenTransform(joint.origin()), joint.name());
      return true;
    }
    case urdf::Joint::prismatic:
    case urdf::Joint::planar:
      throw std::runtime_error("unsupported joint type");
    case urdf::Joint::revolute:
    case urdf::Joint::continuous: {
      const urdf::Array3d axis = joint.axis();
      if (axis == unit_x) {
        type = Builder::JointType::Rx;
        break;
      }
      if (axis == unit_y) {
        type = Builder::JointType::Ry;
        break;
      }
      if (axis == unit_z) {
        type = Builder::JointType::Rz;
        break;
      }
      throw std::runtime_error("unsupported joint axis");
    }
    case urdf::Joint::floating: {
      type = Builder::JointType::FreeFlyer;
      break;
    }
  }
  _discover(joint.parent_link(), joint.child_link(),
            Math::toEigenTransform(joint.origin()), type,
            parser.link(joint.child_link()), joint.name());
  return true;
}

void buildRigidBodySystemFromUrdf(
    RigidBodySystemBuilder::Interface<double> &builder, urdf::ptree &pt,
    bool remove_root_joint, bool make_continuous_joints_fixed) {
  urdf::RobotTree parser(pt.get_child("robot"));
  if (remove_root_joint) parser.rm_root_joint();
  if (make_continuous_joints_fixed) makeContinuousJointsFixed(parser);
  makeMasslessJointsFixed(parser);
  UrdfRigidBodySystemTraverser visitor(builder, parser);
  visitor._discover("", parser.root_link(), Pose::Identity(),
                    Builder::JointType::FreeFlyer,
                    parser.link(parser.root_link()));
  parser.traverse_joints(visitor);
}

void buildRigidBodySystemFromUrdf(
    RigidBodySystemBuilder::Interface<double> &builder, std::istream &is,
    bool remove_root_joint, bool make_continuous_joints_fixed) {
  AL::urdf::ptree pt;
  boost::property_tree::xml_parser::read_xml(is, pt);
  buildRigidBodySystemFromUrdf(builder, pt, remove_root_joint,
                               make_continuous_joints_fixed);
}
}
