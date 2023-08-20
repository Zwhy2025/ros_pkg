/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#include "kdl_parser/kdl_parser.hpp"

#include <string>
#include <vector>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <kdl/frames_io.hpp>

#ifdef HAS_ROS
#include <ros/console.h>
#else
// forward ROS warnings and errors to stderr
#define ROS_DEBUG(...) fprintf(stdout, __VA_ARGS__);
#define ROS_ERROR(...) fprintf(stderr, __VA_ARGS__);
#define ROS_WARN(...) fprintf(stderr, __VA_ARGS__);
#endif

#ifdef HAS_URDF
#include <urdf/model.h>
#include <urdf/urdfdom_compatibility.h>
#endif

namespace kdl_parser
{
// construct vector
KDL::Vector toKdl(urdf::Vector3 v)
{
  return KDL::Vector(v.x, v.y, v.z);
}

// construct rotation
KDL::Rotation toKdl(urdf::Rotation r)
{
  return KDL::Rotation::Quaternion(r.x, r.y, r.z, r.w);
}

// construct pose
KDL::Frame toKdl(urdf::Pose p)
{
  return KDL::Frame(toKdl(p.rotation), toKdl(p.position));
}

// construct joint
KDL::Joint toKdl(urdf::JointSharedPtr jnt)
{
  KDL::Frame F_parent_jnt = toKdl(jnt->parent_to_joint_origin_transform);

  switch (jnt->type) {
    case urdf::Joint::FIXED: {
        return KDL::Joint(jnt->name, KDL::Joint::None);
      }
    case urdf::Joint::REVOLUTE: {
        KDL::Vector axis = toKdl(jnt->axis);
        return KDL::Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis, KDL::Joint::RotAxis);
      }
    case urdf::Joint::CONTINUOUS: {
        KDL::Vector axis = toKdl(jnt->axis);
        return KDL::Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis, KDL::Joint::RotAxis);
      }
    case urdf::Joint::PRISMATIC: {
        KDL::Vector axis = toKdl(jnt->axis);
        return KDL::Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis, KDL::Joint::TransAxis);
      }
    default: {
        ROS_WARN("Converting unknown joint type of joint '%s' into a fixed joint", jnt->name.c_str());
        return KDL::Joint(jnt->name, KDL::Joint::None);
      }
  }
  return KDL::Joint();
}

// construct inertia
KDL::RigidBodyInertia toKdl(urdf::InertialSharedPtr i)
{
  KDL::Frame origin = toKdl(i->origin);

  // the mass is frame independent
  double kdl_mass = i->mass;

  // kdl and urdf both specify the com position in the reference frame of the link
  KDL::Vector kdl_com = origin.p;

  // kdl specifies the inertia matrix in the reference frame of the link,
  // while the urdf specifies the inertia matrix in the inertia reference frame
  KDL::RotationalInertia urdf_inertia =
    KDL::RotationalInertia(i->ixx, i->iyy, i->izz, i->ixy, i->ixz, i->iyz);

  // Rotation operators are not defined for rotational inertia,
  // so we use the RigidBodyInertia operators (with com = 0) as a workaround
  KDL::RigidBodyInertia kdl_inertia_wrt_com_workaround =
    origin.M * KDL::RigidBodyInertia(0, KDL::Vector::Zero(), urdf_inertia);

  // Note that the RigidBodyInertia constructor takes the 3d inertia wrt the com
  // while the getRotationalInertia method returns the 3d inertia wrt the frame origin
  // (but having com = Vector::Zero() in kdl_inertia_wrt_com_workaround they match)
  KDL::RotationalInertia kdl_inertia_wrt_com =
    kdl_inertia_wrt_com_workaround.getRotationalInertia();

  return KDL::RigidBodyInertia(kdl_mass, kdl_com, kdl_inertia_wrt_com);
}

// 递归函数，用于将 URDF 链接结构转换为 KDL 树
    bool addChildrenToTree(urdf::LinkConstSharedPtr root, KDL::Tree & tree) {
        // 获取当前链接的子链接
        std::vector<urdf::LinkSharedPtr> children = root->child_links;
        // 打印调试信息，显示当前链接的名称和子链接的数量
        ROS_DEBUG("Link %s had %zu children", root->name.c_str(), children.size());

        // 构造惯性对象，如果链接具有惯性信息，则转换为 KDL 格式
        KDL::RigidBodyInertia inert(0);
        if (root->inertial) {
            inert = toKdl(root->inertial);
        }

        // 将 URDF 关节转换为 KDL 关节
        KDL::Joint jnt = toKdl(root->parent_joint);

        // 构造 KDL 段，包括链接名称、关节、原点到关节的转换和惯性
        KDL::Segment sgm(root->name, jnt, toKdl(
                root->parent_joint->parent_to_joint_origin_transform), inert);

        // 将构造的段添加到 KDL 树中
        tree.addSegment(sgm, root->parent_joint->parent_link_name);

        // 递归添加所有子链接到 KDL 树中
        for (size_t i = 0; i < children.size(); i++) {
            if (!addChildrenToTree(children[i], tree)) {
                return false; // 如果添加子链接失败，则返回 false
            }
        }
        return true; // 所有子链接成功添加，返回 true
    }


bool treeFromFile(const std::string & file, KDL::Tree & tree)
{
  const urdf::ModelInterfaceSharedPtr robot_model = urdf::parseURDFFile(file);
  return kdl_parser::treeFromUrdfModel(*robot_model, tree);
}


bool treeFromParam(const std::string & param, KDL::Tree & tree)
{
#if defined(HAS_ROS) && defined(HAS_URDF)
  urdf::Model robot_model;
  if (!robot_model.initParam(param)){
    ROS_ERROR("Could not generate robot model");
    return false;
  }
  return treeFromUrdfModel(robot_model, tree);
#else
  return false;
#endif
}


bool treeFromString(const std::string & xml, KDL::Tree & tree)
{
  const urdf::ModelInterfaceSharedPtr robot_model = urdf::parseURDF(xml);
  if (!robot_model) {
    ROS_ERROR("Could not generate robot model");
    return false;
  }
  return kdl_parser::treeFromUrdfModel(*robot_model, tree);
}

bool treeFromXml(const tinyxml2::XMLDocument * xml_doc, KDL::Tree & tree)
{
  if (!xml_doc) {
    ROS_ERROR("Could not parse the xml document");
    return false;
  }

  tinyxml2::XMLPrinter printer;
  xml_doc->Print(&printer);

  return treeFromString(printer.CStr(), tree);
}

bool treeFromXml(TiXmlDocument * xml_doc, KDL::Tree & tree)
{
  if (!xml_doc) {
    ROS_ERROR("Could not parse the xml document");
    return false;
  }

  std::stringstream ss;
  ss << *xml_doc;

  return treeFromString(ss.str(), tree);
}

bool treeFromUrdfModel(const urdf::ModelInterface & robot_model, KDL::Tree & tree)
{
  if (!robot_model.getRoot()) {
    return false;
  }

  tree = KDL::Tree(robot_model.getRoot()->name);

  // warn if root link has inertia. KDL does not support this
  if (robot_model.getRoot()->inertial) {
    ROS_WARN("The root link %s has an inertia specified in the URDF, but KDL does not "
      "support a root link with an inertia.  As a workaround, you can add an extra "
      "dummy link to your URDF.", robot_model.getRoot()->name.c_str());
  }

  //  遍历所有子链接 links
  for (size_t i = 0; i < robot_model.getRoot()->child_links.size(); i++) {
    if (!addChildrenToTree(robot_model.getRoot()->child_links[i], tree)) {
      return false;
    }
  }

  return true;
}

}  // namespace kdl_parser
