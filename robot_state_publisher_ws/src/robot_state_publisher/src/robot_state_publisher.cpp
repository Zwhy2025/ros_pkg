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

#include <kdl/frames_io.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_kdl/tf2_kdl.h>

#include "robot_state_publisher/robot_state_publisher.h"

using namespace std;
using namespace ros;

namespace robot_state_publisher {

    RobotStatePublisher::RobotStatePublisher(const KDL::Tree& tree, const urdf::Model& model)
            : model_(model)
    {
        // 遍历树并将段添加到 segments_
        addChildren(tree.getRootSegment());
    }

// 将子段添加到正确的映射中
    void RobotStatePublisher::addChildren(const KDL::SegmentMap::const_iterator segment)
    {

        // 获取当前段的根名称
        const std::string& root = GetTreeElementSegment(segment->second).getName();

        ROS_INFO("Adding segment from %s", root.c_str());
        // 获取当前段的子段信息
        const std::vector<KDL::SegmentMap::const_iterator>& children = GetTreeElementChildren(segment->second);

        // 遍历子段
        for (unsigned int i = 0; i < children.size(); i++) {
            // 获取子段信息
            const KDL::Segment& child = GetTreeElementSegment(children[i]->second);

            // 创建 SegmentPair 对象，包含了根、子段名称和段信息
            SegmentPair s(GetTreeElementSegment(children[i]->second), root, child.getName());

            // 检查子段的关节类型
            if (child.getJoint().getType() == KDL::Joint::None) {
                // 如果关节类型为 None，则检查是否是浮动关节
                if (model_.getJoint(child.getJoint().getName()) && model_.getJoint(child.getJoint().getName())->type == urdf::Joint::FLOATING) {
                    ROS_INFO("Floating joint. Not adding segment from %s to %s. This TF can not be published based on joint_states info", root.c_str(), child.getName().c_str());
                } else {
                    // 将固定关节添加到 segments_fixed_ 映射中
                    segments_fixed_.insert(make_pair(child.getJoint().getName(), s));
                    ROS_DEBUG("Adding fixed segment from %s to %s", root.c_str(), child.getName().c_str());
                }
            } else {
                // 将移动关节添加到 segments_ 映射中
                segments_.insert(make_pair(child.getJoint().getName(), s));
                ROS_DEBUG("Adding moving segment from %s to %s", root.c_str(), child.getName().c_str());
            }

            // 递归调用，处理子段的子段
            addChildren(children[i]);
        }
    }


// 发布可移动的变换
    void RobotStatePublisher::publishTransforms(const map<string, double>& joint_positions, const Time& time, const std::string& tf_prefix)
    {
        ROS_DEBUG("Publishing transforms for moving joints");
        std::vector<geometry_msgs::TransformStamped> tf_transforms;

        // 循环遍历所有关节
        for (map<string, double>::const_iterator jnt=joint_positions.begin(); jnt != joint_positions.end(); jnt++) {
            std::map<std::string, SegmentPair>::const_iterator seg = segments_.find(jnt->first);
            if (seg != segments_.end()) {
                geometry_msgs::TransformStamped tf_transform = tf2::kdlToTransform(seg->second.segment.pose(jnt->second));
                tf_transform.header.stamp = time;
                tf_transform.header.frame_id = tf::resolve(tf_prefix, seg->second.root);
                tf_transform.child_frame_id = tf::resolve(tf_prefix, seg->second.tip);
                tf_transforms.push_back(tf_transform);
            }
            else {
                ROS_WARN_THROTTLE(10, "Joint state with name: \"%s\" was received but not found in URDF", jnt->first.c_str());
            }
        }
        tf_broadcaster_.sendTransform(tf_transforms);
    }

// 发布固定的变换
    void RobotStatePublisher::publishFixedTransforms(const std::string& tf_prefix, bool use_tf_static)
    {
        ROS_DEBUG("Publishing transforms for fixed joints");
        std::vector<geometry_msgs::TransformStamped> tf_transforms;
        geometry_msgs::TransformStamped tf_transform;

        // 循环遍历所有固定段
        for (map<string, SegmentPair>::const_iterator seg=segments_fixed_.begin(); seg != segments_fixed_.end(); seg++) {
            geometry_msgs::TransformStamped tf_transform = tf2::kdlToTransform(seg->second.segment.pose(0));
            tf_transform.header.stamp = ros::Time::now();
            if (!use_tf_static) {
                tf_transform.header.stamp += ros::Duration(0.5);
            }
            tf_transform.header.frame_id = tf::resolve(tf_prefix, seg->second.root);
            tf_transform.child_frame_id = tf::resolve(tf_prefix, seg->second.tip);
            tf_transforms.push_back(tf_transform);
        }
        if (use_tf_static) {
            static_tf_broadcaster_.sendTransform(tf_transforms);
        }
        else {
            tf_broadcaster_.sendTransform(tf_transforms);
        }
    }

}
