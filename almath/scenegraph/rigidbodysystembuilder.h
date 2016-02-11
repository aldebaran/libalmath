/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */

// Interface for a RigidBodySystemBuilder (as in the "builder design pattern")
// Implement this interface in order to initialize a rigid bodies simulator
// or to export a rigid body system in some description format.
//
// When converting from a description format to another, a
// RigidBodySystemBuilder specific to the output format, is often used with
// a "kinematic tree traverser", which is specific to the input format.

#ifndef LIB_ALMATH_SCENEGRAPH_RIGIDBODYSYSTEMBUILDER_H
#define LIB_ALMATH_SCENEGRAPH_RIGIDBODYSYSTEMBUILDER_H

#include <string>
#include <set>
#include <Eigen/Geometry>
#include <stdexcept>
#include <boost/format.hpp>
#include <almath/scenegraph/bodymass.h>

namespace AL {
namespace RigidBodySystemBuilder {

enum struct JointType { FreeFlyer, Rx, Ry, Rz };

class Config {
 public:
  Config() : no_static_frame(false) {}
  bool no_static_frame;
  std::string joint_name_format = "%1%_joint";
  std::string galilean_frame;

  // map body name name to joint name
  std::string joint_name(const std::string &body_name) const {
    return boost::str(boost::format(joint_name_format) % body_name);
  }
};

template <typename T>
struct LinkData {
  typedef T Scalar;
  typedef Eigen::Transform<T, 3, Eigen::AffineCompact, Eigen::DontAlign> Pose;
  typedef Math::BodyMass<Scalar> BodyMass;
  typedef typename BodyMass::Vector3 Vector3;
  typedef typename BodyMass::Matrix3 Matrix3;

  std::string parent_body;
  std::string new_body;
  std::string new_joint;
  Pose pose_parent_new;
  JointType joint_type;
  BodyMass body_mass;

  template <typename S>
  LinkData<S> cast() const {
    return LinkData<S>{parent_body, new_body,
                       new_joint,   pose_parent_new.template cast<S>(),
                       joint_type,  body_mass.template cast<S>()};
  }
};

template <typename T>
struct StaticFrameData {
  typedef T Scalar;
  typedef Eigen::Transform<T, 3, Eigen::AffineCompact, Eigen::DontAlign> Pose;

  std::string parent_frame;
  std::string new_static_frame;
  std::string new_static_transform;
  Pose pose_parent_new;

  template <typename S>
  StaticFrameData<S> cast() const {
    return StaticFrameData<S>{parent_frame, new_static_frame,
                              new_static_transform,
                              pose_parent_new.template cast<S>()};
  }
};

// Interface for a rigid body system builder.
template <typename T>
class Interface {
 public:
  typedef T Scalar;
  typedef LinkData<Scalar> Link;
  typedef typename Link::BodyMass BodyMass;
  typedef typename BodyMass::Vector3 Vector3;
  typedef typename BodyMass::Matrix3 Matrix3;
  typedef StaticFrameData<Scalar> StaticFrame;
  typedef typename StaticFrame::Pose Pose;
  virtual ~Interface() {}

  virtual const Config &config() const = 0;
  virtual void addLink(Link link) = 0;
  virtual void addStaticFrame(StaticFrame sframe) = 0;

  // add a link
  void add(const std::string &parent_body, const std::string &new_body,
           const Pose &H_parent_joint, JointType joint_type,
           const BodyMass &mass, const std::string &new_joint) {
    addLink(Link{parent_body, new_body, new_joint, H_parent_joint, joint_type,
                 mass});
  }
  void add(const std::string &parent_body, const std::string &new_body,
           const Pose &H_parent_joint, JointType joint_type,
           const BodyMass &mass) {
    add(parent_body, new_body, H_parent_joint, joint_type, mass,
        this->config().joint_name(new_body));
  }

  // add a static frame
  void add(const std::string &parent_frame, const std::string &new_static_frame,
           const Pose &H_parent_new, const std::string &new_static_transform) {
    if (this->config().no_static_frame) return;
    addStaticFrame(StaticFrame{parent_frame, new_static_frame,
                               new_static_transform, H_parent_new});
  }

  void add(const std::string &parent_frame, const std::string &new_static_frame,
           const Pose &H_parent_new) {
    add(parent_frame, new_static_frame, H_parent_new,
        this->config().joint_name(new_static_frame));
  }
};

#define TYPEDEF_Interface_TYPES                           \
  typedef typename Interface<T>::Scalar Scalar;           \
  typedef typename Interface<T>::Link Link;               \
  typedef typename Interface<T>::BodyMass BodyMass;       \
  typedef typename Interface<T>::Vector3 Vector3;         \
  typedef typename Interface<T>::Matrix3 Matrix3;         \
  typedef typename Interface<T>::StaticFrame StaticFrame; \
  typedef typename Interface<T>::Pose Pose

// Interface for a rigid body system builder.
template <typename T>
class Decorator : public Interface<T> {
 public:
  TYPEDEF_Interface_TYPES;
  Decorator(Interface<T> &builder) : _builder(builder) {}
  virtual ~Decorator() {}

  const Config &config() const { return _builder.config(); }

  virtual void addLink(Link link) { _builder.addLink(link); }

  virtual void addStaticFrame(StaticFrame sframe) {
    _builder.addStaticFrame(sframe);
  }

 protected:
  Interface<T> &_builder;
};

template <typename TD, typename TB>
class Adapter : public Interface<TD> {
 public:
  Adapter(Interface<TB> &builder) : _builder(builder) {}
  virtual ~Adapter() {}

  const Config &config() const { return _builder.config(); }

  virtual void addLink(LinkData<TD> link) {
    _builder.addLink(link.template cast<TB>());
  }

  virtual void addStaticFrame(StaticFrameData<TD> sframe) {
    _builder.addStaticFrame(sframe.template cast<TB>());
  }

 protected:
  Interface<TB> &_builder;
};

template <typename T>
class NamesChecker : public Decorator<T> {
 public:
  TYPEDEF_Interface_TYPES;

  NamesChecker(Interface<T> &builder) : Decorator<T>(builder) {}

  void addLink(Link link) {
    if (!xIsKnownBody(link.parent_body)) {
      std::stringstream ss;
      ss << "cannot add body \"" << link.new_body
         << "\": unknown parent body \"" << link.parent_body << "\"";
      throw std::runtime_error(ss.str());
    }
    if (xIsKnownFrame(link.new_body)) {
      std::stringstream ss;
      ss << "cannot add body \"" << link.new_body << "\":"
         << " there is already a frame with this name";
      throw std::runtime_error(ss.str());
    }
    if (xIsKnownTransform(link.new_joint)) {
      std::stringstream ss;
      ss << "cannot add joint \"" + link.new_joint + "\":"
         << " there is already a transform with this name";
      throw std::runtime_error(ss.str());
    }
    this->_builder.addLink(link);
    _known_bodies.insert(link.new_body);
    _known_joints.insert(link.new_joint);
  }

  void addStaticFrame(StaticFrame sframe) {
    if (!xIsKnownFrame(sframe.parent_frame)) {
      throw std::runtime_error(
          "cannot add static frame \"" + sframe.new_static_frame +
          "\": unknown parent frame \"" + sframe.parent_frame + "\"");
    }

    if (xIsKnownFrame(sframe.new_static_frame)) {
      throw std::runtime_error("cannot add static frame \"" +
                               sframe.new_static_frame +
                               "\": there is already a frame with this name");
    }
    if (xIsKnownTransform(sframe.new_static_transform)) {
      throw std::runtime_error(
          "cannot add static transform \"" + sframe.new_static_transform +
          "\": there is already a transform with this name");
    }
    this->_builder.addStaticFrame(sframe);
    _known_sframes.insert(sframe.new_static_frame);
    _known_stransforms.insert(sframe.new_static_transform);
  }

 private:
  // sets of known names. Preloaded with the galilean frame
  std::set<std::string> _known_bodies =
      std::set<std::string>({this->_builder.config().galilean_frame});
  std::set<std::string> _known_joints;
  std::set<std::string> _known_sframes;
  std::set<std::string> _known_stransforms;

  bool xIsKnownBody(const std::string &name) const {
    return _known_bodies.find(name) != _known_bodies.end();
  }

  // bodies and static frames
  bool xIsKnownFrame(const std::string &name) const {
    return xIsKnownBody(name) ||
           (_known_sframes.find(name) != _known_sframes.end());
  }

  bool xIsKnownJoint(const std::string &name) const {
    return _known_joints.find(name) != _known_joints.end();
  }

  // joints and static transforms
  bool xIsKnownTransform(const std::string &name) const {
    return xIsKnownJoint(name) ||
           (_known_stransforms.find(name) != _known_stransforms.end());
  }
};

template <typename T>
class InertiaEraser : public Decorator<T> {
 public:
  TYPEDEF_Interface_TYPES;
  InertiaEraser(Interface<T> &builder) : Decorator<T>(builder) {}
  void addLink(Link link) {
    link.body_mass.rotational_inertia = Matrix3::Zero();
    this->_builder.addLink(link);
  }
};

#undef TYPEDEF_Interface_TYPES
}
}

#endif
