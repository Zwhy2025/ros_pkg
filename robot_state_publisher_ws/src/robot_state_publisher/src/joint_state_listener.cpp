#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "robot_state_publisher/robot_state_publisher.h"
#include "robot_state_publisher/joint_state_listener.h"

using namespace std;
using namespace ros;
using namespace KDL;
using namespace robot_state_publisher;

JointStateListener::JointStateListener(const KDL::Tree &tree, const MimicMap &m, const urdf::Model &model)
        : state_publisher_(tree, model), mimic_(m) {
    ros::NodeHandle n_tilde("~");
    ros::NodeHandle n;

    // 设置发布频率
    double publish_freq;
    n_tilde.param("publish_frequency", publish_freq, 50.0);
    // 设置是否使用 /tf_static 拉取静态转换广播器
    n_tilde.param("use_tf_static", use_tf_static_, true);
    // ignore_timestamp_ == true, 接受关节状态消息，无论其时间戳
    n_tilde.param("ignore_timestamp", ignore_timestamp_, false);
    // 从最近的命名空间获取 tf_prefix 参数
    std::string tf_prefix_key;
    n_tilde.searchParam("tf_prefix", tf_prefix_key);
    n_tilde.param(tf_prefix_key, tf_prefix_, std::string(""));
    publish_interval_ = ros::Duration(1.0 / max(publish_freq, 1.0));

    // 设置 tcpNoNelay 告诉订阅者要求连接的发布者在其端设置 TCP_NODELAY
    // 这可以防止一些关节状态消息被捆绑在一起，增加其中一个消息的延迟。
    ros::TransportHints transport_hints;
    transport_hints.tcpNoDelay(true);
    // 订阅关节状态
    joint_state_sub_ = n.subscribe("joint_states", 1, &JointStateListener::callbackJointState, this, transport_hints);

    // 触发发布固定关节
    // 如果使用静态转换广播器，这将是一次性触发器，只运行一次
    timer_ = n_tilde.createTimer(publish_interval_, &JointStateListener::callbackFixedJoint, this, use_tf_static_);
}

JointStateListener::~JointStateListener() {}

// 固定关节的回调函数
void JointStateListener::callbackFixedJoint(const ros::TimerEvent &e) {
    (void) e;
    state_publisher_.publishFixedTransforms(tf_prefix_, use_tf_static_);
}

// 关节状态的回调函数
void JointStateListener::callbackJointState(const JointStateConstPtr &state) {
    if (state->name.size() != state->position.size()) {
        if (state->position.empty()) {
            const int throttleSeconds = 300;
            // 警告：关节状态消息中的位置为空，将被忽略。此消息将在指定秒数后重新出现。
            ROS_WARN_THROTTLE(throttleSeconds,
                              "Robot state publisher ignored a JointState message about joint(s) "
                              "\"%s\"(,...) whose position member was empty. This message will "
                              "not reappear for %d seconds.", state->name[0].c_str(),
                              throttleSeconds);
        } else {
            // 错误：忽略了无效的关节状态消息
            ROS_ERROR("Robot state publisher ignored an invalid JointState message");
        }
        return;
    }

    // 检查是否向后移动了时间（例如，播放背包文件时）
    ros::Time now = ros::Time::now();
    if (last_callback_time_ > now) {
        // 强制重新发布关节转换
        ROS_WARN("Moved backwards in time (probably because ROS clock was reset), re-publishing joint transforms!");
        last_publish_time_.clear();
    }
    ros::Duration warning_threshold(30.0);
    if ((state->header.stamp + warning_threshold) < now) {
        // 警告：收到的关节状态是过时的
        ROS_WARN_THROTTLE(10, "Received JointState is %f seconds old.", (now - state->header.stamp).toSec());
    }
    last_callback_time_ = now;

    // 确定最近发布的关节
    ros::Time last_published = now;
    for (unsigned int i = 0; i < state->name.size(); i++) {
        ros::Time t = last_publish_time_[state->name[i]];
        last_published = (t < last_published) ? t : last_published;
    }
    // 注意：如果首次看到关节，则last_published为零。

    // 检查是否需要发布
    if (ignore_timestamp_ || state->header.stamp >= last_published + publish_interval_) {
        // 从状态消息获取关节位置
        map<string, double> joint_positions;
        for (unsigned int i = 0; i < state->name.size(); i++) {
            joint_positions.insert(make_pair(state->name[i], state->position[i]));
        }

        // 模拟关节位置
        for (MimicMap::iterator i = mimic_.begin(); i != mimic_.end(); i++) {
            if (joint_positions.find(i->second->joint_name) != joint_positions.end()) {
                double pos = joint_positions[i->second->joint_name] * i->second->multiplier + i->second->offset;
                joint_positions.insert(make_pair(i->first, pos));
            }
        }

        // 发布转换
        state_publisher_.publishTransforms(joint_positions, state->header.stamp, tf_prefix_);

        // 在关节映射中存储发布时间
        for (unsigned int i = 0; i < state->name.size(); i++) {
            last_publish_time_[state->name[i]] = state->header.stamp;
        }
    }
}

// ----------------------------------
// ----- 主函数 ---------------------
// ----------------------------------
int main(int argc, char **argv) {
    // 初始化ros
    ros::init(argc, argv, "robot_state_publisher");
    NodeHandle node;

    ///////////////////////////////////////// 开始弃用警告
    std::string exe_name = argv[0];
    std::size_t slash = exe_name.find_last_of("/");
    if (slash != std::string::npos) {
        exe_name = exe_name.substr(slash + 1);
    }
    if (exe_name == "state_publisher") {
        ROS_WARN("已弃用 'state_publisher' 可执行文件。请改用 'robot_state_publisher'");
    }
    ///////////////////////////////////////// 结束弃用警告

    // 从参数服务器获取机器人描述的位置
    urdf::Model model;
    if (!model.initParam("robot_description"))
        return -1;

    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree)) {
        ROS_ERROR("从xml机器人描述中提取kdl树失败");
        return -1;
    }

    MimicMap mimic;

    for (std::map<std::string, urdf::JointSharedPtr>::iterator i = model.joints_.begin();
         i != model.joints_.end(); i++) {
        if (i->second->mimic) {
            mimic.insert(make_pair(i->first, i->second->mimic));
        }
    }

    JointStateListener state_publisher(tree, mimic, model);
    ros::spin();

    return 0;
}
