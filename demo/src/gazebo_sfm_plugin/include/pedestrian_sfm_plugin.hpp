/***********************************************************
 *
 * @file: pedestrian_sfm_plugin.hpp
 * @brief: 用社交力模型实现行人仿真的 Gazebo 插件头文件（ROS2 版本）
 * @author: Yang Haodong
 * @update: 2023-03-15 修改为 ROS2 版本
 * @version: 1.1
 *
 * 版权所有 (c) 2023, Yang Haodong
 * 保留所有权利.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef PEDESTRIANSFM_GAZEBO_PLUGIN_H
#define PEDESTRIANSFM_GAZEBO_PLUGIN_H

// ------------------------------
// C++ 标准库
// ------------------------------
#include <algorithm>
#include <memory> // 添加：用于智能指针
#include <string>
#include <vector>

// ------------------------------
// ROS2 相关头文件（替换了 ROS1 头文件）
// 删除：原来使用的 ROS1 头文件
// <geometry_msgs/PoseStamped.h>、<geometry_msgs/Twist.h> 和 <ros/ros.h>
// 新增：使用 ROS2 的对应头文件，并添加 rclcpp 用于节点管理
#include <geometry_msgs/msg/pose_stamped.hpp> // 新增：ROS2 的 PoseStamped 消息
#include <geometry_msgs/msg/twist.hpp>        // 新增：ROS2 的 Twist 消息
#include <nav_msgs/msg/odometry.hpp>          // 添加里程计消息头文件
#include <rclcpp/rclcpp.hpp>                  // 新增：ROS2 的 rclcpp 节点

// TF2 相关头文件
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// ------------------------------
// Gazebo 头文件（保持不变）
// ------------------------------
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/system.hh>

// ------------------------------
// Social Force Model 头文件（保持不变，lightsfm 集成在项目中）
// ------------------------------
#include <lightsfm/sfm.hpp>

// ------------------------------
// 消息/服务部分
// 删除：原来 ROS1 使用 <gazebo_sfm_plugin/ped_state.h>
// 新增：ROS2 生成的服务头文件（假设服务定义位于 srv 目录下）
#include <gazebo_sfm_plugin/srv/ped_state.hpp>

namespace gazebo {

class GZ_PLUGIN_VISIBLE PedestrianSFMPlugin : public ModelPlugin {
public:
  /**
   * @brief 构造函数：构造一个 Gazebo 插件实例
   */
  PedestrianSFMPlugin();

  /**
   * @brief 析构函数：释放插件资源
   */
  ~PedestrianSFMPlugin();

  /**
   * @brief 加载插件，初始化模型、ROS2 节点和其它参数
   * @param _model 父模型的指针
   * @param _sdf 插件对应的 SDF 元素指针
   */
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  /**
   * @brief 初始化社交力模型，读取参数并设置初始状态
   */
  virtual void Reset();

private:
  /**
   * @brief 每个仿真更新周期调用的函数，用于更新状态和发布消息
   * @param _info 更新周期信息
   */
  void OnUpdate(const common::UpdateInfo &_info);

  /**
   * @brief 服务回调函数，处理外部请求并返回行人状态
   * @param request 请求数据（使用 ROS2 的 std::shared_ptr 形式）
   * @param response 响应数据（使用 ROS2 的 std::shared_ptr 形式）
   */
  void OnStateCallBack(
      const std::shared_ptr<gazebo_sfm_plugin::srv::PedState::Request> request,
      std::shared_ptr<gazebo_sfm_plugin::srv::PedState::Response> response);

  /**
   * @brief 辅助函数：检测最近的障碍物
   */
  void handleObstacles();

  /**
   * @brief 辅助函数：检测附近的行人（其他 actor）
   */
  void handlePedestrians();

private:
  // ------------------------------
  // ROS2 节点和通信部分（全部替换了 ROS1 的 NodeHandle 和相关类型）
  // 删除：ROS1 中的 std::unique_ptr<ros::NodeHandle> node_;
  // 新增：使用 rclcpp::Node::SharedPtr 节点指针
  rclcpp::Node::SharedPtr node_;

  // 发布者（ROS2 的 Publisher 类型）
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
      odom_pub_; // 添加里程计发布器

  // 服务服务器（ROS2 的 Service 类型）
  rclcpp::Service<gazebo_sfm_plugin::srv::PedState>::SharedPtr state_server_;

  // 用社交力模型表示的行人代理
  sfm::Agent sfm_actor_;

  // 行人组中其它模型名称列表
  std::vector<std::string> group_names_;

  // 检测到的其它行人列表
  std::vector<sfm::Agent> other_actors_;

  // 时间延迟
  double time_delay_;
  bool time_init_;

  // 检测附近行人的最大距离
  double people_dist_;

  // 状态初始化标志
  bool pose_init_;

  // 上一次位置（用于计算速度）
  double last_pose_x_, last_pose_y_;

  // 当前状态变量
  double px_, py_, pz_, vx_, vy_, theta_;

  // 指向父 actor 的指针
  physics::ActorPtr actor_;

  // 指向世界的指针
  physics::WorldPtr world_;

  // 指向 SDF 元素的指针
  sdf::ElementPtr sdf_;

  // 记录 actor 的速度
  ignition::math::Vector3d velocity_;

  // 连接列表（用于管理更新回调）
  std::vector<event::ConnectionPtr> connections_;

  // 动画因子，用于协调位移与行走动画
  double animation_factor_ = 1.0;

  // 上一次更新的时间
  common::Time last_update_;

  // 忽略的模型列表，用于过滤不需要检测的障碍
  std::vector<std::string> ignore_models_;

  // 自定义轨迹信息，用于动画控制
  physics::TrajectoryInfoPtr trajectory_info_;
};

} // namespace gazebo
#endif
