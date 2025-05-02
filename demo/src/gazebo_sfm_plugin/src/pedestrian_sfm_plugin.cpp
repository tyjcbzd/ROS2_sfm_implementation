/***********************************************************
 *
 * @file: pedestrian_sfm_plugin.cpp
 * @brief: 用社交力模型实现行人仿真的 Gazebo 插件（ROS2 版本）
 * @author: Yang Haodong
 * @update: 针对 ROS2 进行修改
 *
 * 版权所有 (c) 2023, Yang Haodong
 * 保留所有权利.
 *
 **********************************************************/
#include <chrono>
#include <cstdio>
#include <functional>
#include <string>
#include <thread>

// 新增：ROS2 必要的头文件
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// 保留：项目内部的插件头文件，假设内部代码也已适配ROS2
#include <pedestrian_sfm_plugin.hpp>

#define WALKING_ANIMATION "walking"

using namespace gazebo;
// 注册 Gazebo 插件，宏定义保持不变
GZ_REGISTER_MODEL_PLUGIN(PedestrianSFMPlugin)

//
// 构造函数和析构函数，基本保持不变
//
PedestrianSFMPlugin::PedestrianSFMPlugin()
    : pose_init_(false), time_delay_(0.0), time_init_(false) {}

PedestrianSFMPlugin::~PedestrianSFMPlugin() {}

//
// Load() 函数：插件加载时调用，初始化模型、ROS2 节点、发布者、服务等
//
void PedestrianSFMPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // 保存 SDF 配置和模型指针
  sdf_ = _sdf;
  actor_ = boost::dynamic_pointer_cast<physics::Actor>(_model);
  world_ = actor_->GetWorld();

  // ------------------------------
  // ROS2 节点初始化部分
  // ------------------------------
  // 删除：ROS1 使用 ros::isInitialized() 和 ros::init() 的代码
  // 新增：使用 rclcpp::ok() 判断和 rclcpp::init() 初始化 ROS2 环境
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  // 创建 ROS2 节点，替换 ROS1 中的 ros::NodeHandle
  std::string node_name =
      "gazebo_sfm_" + actor_->GetName(); // 修正：使用唯一节点名称
  node_ = rclcpp::Node::make_shared(node_name);

  // ------------------------------
  // 创建发布者（ROS2 API）
  // ------------------------------
  std::string pose_topic = "/" + actor_->GetName() + "/pose";
  pose_pub_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);

  std::string twist_topic = "/" + actor_->GetName() + "/twist";
  vel_pub_ =
      node_->create_publisher<geometry_msgs::msg::Twist>(twist_topic, 10);

  // 添加里程计发布器
  std::string odom_topic = "/" + actor_->GetName() + "/odom";
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);

  // ------------------------------
  // 创建服务（ROS2 API）
  // ------------------------------
  std::string service_name = actor_->GetName() + "_state";
  state_server_ = node_->create_service<gazebo_sfm_plugin::srv::PedState>(
      service_name, std::bind(&PedestrianSFMPlugin::OnStateCallBack, this,
                              std::placeholders::_1, std::placeholders::_2));

  // ------------------------------
  // 以下部分保持基本不变：读取碰撞属性、配置链路等
  // ------------------------------
  std::map<std::string, ignition::math::Vector3d> scales;
  std::map<std::string, ignition::math::Pose3d> offsets;

  if (sdf_->HasElement("collision")) {
    auto elem = sdf_->GetElement("collision");
    while (elem) {
      auto name = elem->Get<std::string>();

      if (elem->HasAttribute("scale")) {
        auto scale = elem->Get<ignition::math::Vector3d>("scale");
        scales[name] = scale;
      }
      if (elem->HasAttribute("pose")) {
        auto pose = elem->Get<ignition::math::Pose3d>("pose");
        offsets[name] = pose;
      }
      elem = elem->GetNextElement("collision");
    }
  }
  // 对每个链路进行初始化并设置碰撞属性
  for (const auto &link : actor_->GetLinks()) {
    link->Init();
    if (scales.empty())
      continue;
    for (const auto &collision : link->GetCollisions()) {
      auto name = collision->GetName();
      if (scales.find(name) != scales.end()) {
        auto boxShape = boost::dynamic_pointer_cast<gazebo::physics::BoxShape>(
            collision->GetShape());
        if (boxShape)
          boxShape->SetSize(boxShape->Size() * scales[name]);
      }
      if (offsets.find(name) != offsets.end())
        collision->SetInitialRelativePose(offsets[name] +
                                          collision->InitialRelativePose());
    }
  }

  // ------------------------------
  // 时间延迟处理
  // ------------------------------
  // 删除：原来使用 sleep() 的代码
  // 新增：使用 std::this_thread::sleep_for 代替 sleep
  if (sdf_->HasElement("time_delay")) {
    time_delay_ = sdf_->GetElement("time_delay")->Get<double>();
    std::unique_ptr<std::thread> t_time_delay(new std::thread([this]() {
      if (!time_init_) {
        std::this_thread::sleep_for(std::chrono::duration<double>(time_delay_));
        time_init_ = true;
      }
    }));
    std::unique_ptr<std::thread> d_time_delay = std::move(t_time_delay);
    d_time_delay->detach();
  } else {
    time_init_ = true;
  }

  // ------------------------------
  // 绑定世界更新回调函数
  // ------------------------------
  connections_.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&PedestrianSFMPlugin::OnUpdate, this, std::placeholders::_1)));

  // 初始化社交力学模型
  Reset();
}

//
// Reset() 函数：初始化社交力模型参数和 actor 状态
//
void PedestrianSFMPlugin::Reset() {
  sfm_actor_.id = actor_->GetId();

  if (sdf_->HasElement("cycle"))
    sfm_actor_.cyclicGoals = sdf_->GetElement("cycle")->Get<bool>();

  if (sdf_->HasElement("trajectory")) {
    sdf::ElementPtr model_elem =
        sdf_->GetElement("trajectory")->GetElement("goalpoint");
    while (model_elem) {
      ignition::math::Pose3d g = model_elem->Get<ignition::math::Pose3d>();
      sfm::Goal goal;
      goal.center.set(g.Pos().X(), g.Pos().Y());
      goal.radius = 0.3;
      sfm_actor_.goals.push_back(goal);
      model_elem = model_elem->GetNextElement("goalpoint");
    }
  }

  auto skel_anims = actor_->SkeletonAnimations();
  if (skel_anims.find(WALKING_ANIMATION) == skel_anims.end()) {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  } else {
    // 初始化动画轨迹
    trajectory_info_.reset(new physics::TrajectoryInfo());
    trajectory_info_->type = WALKING_ANIMATION;
    trajectory_info_->duration = 1.0;
    actor_->SetCustomTrajectory(trajectory_info_);
  }

  // 初始化位置、速度等信息
  ignition::math::Vector3d pos = actor_->WorldPose().Pos();
  ignition::math::Vector3d rpy = actor_->WorldPose().Rot().Euler();
  sfm_actor_.position.set(pos.X(), pos.Y());
  sfm_actor_.yaw = utils::Angle::fromRadian(rpy.Z());
  ignition::math::Vector3d linvel = actor_->WorldLinearVel();
  sfm_actor_.velocity.set(linvel.X(), linvel.Y());
  sfm_actor_.linearVelocity = linvel.Length();
  ignition::math::Vector3d angvel = actor_->WorldAngularVel();
  sfm_actor_.angularVelocity = angvel.Z();

  ignition::math::Pose3d actor_pose = actor_->WorldPose();
  actor_pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z());
  actor_->SetWorldPose(actor_pose);

  // 读取最大速度参数
  if (sdf_->HasElement("velocity"))
    sfm_actor_.desiredVelocity = sdf_->Get<double>("velocity");
  else
    sfm_actor_.desiredVelocity = 0.8;

  // 读取算法权重参数
  if (sdf_->HasElement("goal_weight"))
    sfm_actor_.params.forceFactorDesired = sdf_->Get<double>("goal_weight");
  if (sdf_->HasElement("obstacle_weight"))
    sfm_actor_.params.forceFactorObstacle =
        sdf_->Get<double>("obstacle_weight");
  if (sdf_->HasElement("social_weight"))
    sfm_actor_.params.forceFactorSocial = sdf_->Get<double>("social_weight");
  if (sdf_->HasElement("group_gaze_weight"))
    sfm_actor_.params.forceFactorGroupGaze =
        sdf_->Get<double>("group_gaze_weight");
  if (sdf_->HasElement("group_coh_weight"))
    sfm_actor_.params.forceFactorGroupCoherence =
        sdf_->Get<double>("group_coh_weight");
  if (sdf_->HasElement("group_rep_weight"))
    sfm_actor_.params.forceFactorGroupRepulsion =
        sdf_->Get<double>("group_rep_weight");

  // 读取动画因子参数
  if (sdf_->HasElement("animation_factor"))
    animation_factor_ = sdf_->Get<double>("animation_factor");
  else
    animation_factor_ = 4.5;

  // 读取行人间距参数
  if (sdf_->HasElement("people_distance"))
    people_dist_ = sdf_->Get<double>("people_distance");
  else
    people_dist_ = 5.0;

  // 读取群组信息
  if (sdf_->HasElement("group")) {
    sfm_actor_.groupId = sfm_actor_.id;
    sdf::ElementPtr model_elem = sdf_->GetElement("group")->GetElement("model");
    while (model_elem) {
      group_names_.push_back(model_elem->Get<std::string>());
      model_elem = model_elem->GetNextElement("model");
    }
    sfm_actor_.groupId = sfm_actor_.id;
  } else {
    sfm_actor_.groupId = -1;
  }

  // 读取忽略的障碍物
  if (sdf_->HasElement("ignore_obstacles")) {
    sdf::ElementPtr model_elem =
        sdf_->GetElement("ignore_obstacles")->GetElement("model");
    while (model_elem) {
      ignore_models_.push_back(model_elem->Get<std::string>());
      model_elem = model_elem->GetNextElement("model");
    }
  }
  // 将当前 actor 名称加入忽略列表
  ignore_models_.push_back(actor_->GetName());

  // 遍历世界中的所有模型，添加同类行人至忽略列表
  for (unsigned int i = 0; i < world_->ModelCount(); ++i) {
    physics::ModelPtr model = world_->ModelByIndex(i);
    if (model->GetId() != actor_->GetId() &&
        ((int)model->GetType() == (int)actor_->GetType()))
      ignore_models_.push_back(model->GetName());
  }
}

//
// handleObstacles() 函数：检测最近的障碍物,包括model及其model link的墙壁
//
void PedestrianSFMPlugin::handleObstacles() {
  double min_dist = 10000.0;
  ignition::math::Vector3d closest_obs;
  sfm_actor_.obstacles1.clear();

  // 检查行人周围的障碍物（模型和墙壁链接）
  ignition::math::Vector3d actorPos = actor_->WorldPose().Pos();

  // 遍历所有模型
  for (unsigned int i = 0; i < world_->ModelCount(); ++i) {
    physics::ModelPtr model = world_->ModelByIndex(i);
    std::string model_name = model->GetName();
    bool is_wall_model = (model_name.find("wall") != std::string::npos ||
                          model_name.find("Wall") != std::string::npos);
    bool is_ignored_model =
        (std::find(ignore_models_.begin(), ignore_models_.end(),
                   model->GetName()) != ignore_models_.end());
    bool is_actor_model = ((int)model->GetType() == (int)actor_->GetType());

    // 处理未被忽略的非行人模型
    if (!is_actor_model && !is_ignored_model) {
      // 处理顶级模型作为障碍物（对所有模型或墙壁模型）
      ignition::math::Vector3d modelPos = model->WorldPose().Pos();
      double max_x = model->BoundingBox().Max().X();
      double min_x = model->BoundingBox().Min().X();
      double max_y = model->BoundingBox().Max().Y();
      double min_y = model->BoundingBox().Min().Y();
      double max_z = model->BoundingBox().Max().Z();
      double min_z = model->BoundingBox().Min().Z();
      ignition::math::Vector3d closest_point;
      double closest_weight = 0.8;
      closest_point.X() = ignition::math::clamp(
          closest_weight * actorPos.X() + (1 - closest_weight) * modelPos.X(),
          min_x, max_x);
      closest_point.Y() = ignition::math::clamp(
          closest_weight * actorPos.Y() + (1 - closest_weight) * modelPos.Y(),
          min_y, max_y);
      closest_point.Z() = ignition::math::clamp(
          closest_weight * actorPos.Z() + (1 - closest_weight) * modelPos.Z(),
          min_z, max_z);
      ignition::math::Vector3d offset = closest_point - actorPos;
      double model_dist = offset.Length();

      // 如果是墙壁模型，优先将其视为障碍物
      if (is_wall_model && model_dist < min_dist) {
        min_dist = model_dist;
        closest_obs = closest_point;
      }
      // 对于非墙壁模型，仍然视为普通障碍物
      else if (!is_wall_model && model_dist < min_dist) {
        min_dist = model_dist;
        closest_obs = closest_point;
      }
    }

    // 对所有模型（包括被忽略的模型）检查内部的墙壁链接
    if (!is_actor_model) { // 只要不是行人模型即可
      auto links = model->GetLinks();
      for (const auto &link : links) {
        std::string link_name = link->GetName();
        // 检查链接名称是否包含"Wall"或"wall"，支持各种命名格式如Wall_100,
        // Wall_clone等
        if (link_name.find("Wall") != std::string::npos ||
            link_name.find("wall") != std::string::npos) {

          // 计算行人与墙壁链接之间的最近点
          physics::CollisionPtr collision;

          // 获取链接的第一个碰撞体（一般情况下每个链接的主要碰撞体已足够）
          if (!link->GetCollisions().empty()) {
            collision = link->GetCollisions()[0];

            // 处理链接的定义（大小/形状）- 通过碰撞体的边界框获取
            auto linkBox = collision->BoundingBox();
            double link_max_x = linkBox.Max().X();
            double link_min_x = linkBox.Min().X();
            double link_max_y = linkBox.Max().Y();
            double link_min_y = linkBox.Min().Y();
            double link_max_z = linkBox.Max().Z();
            double link_min_z = linkBox.Min().Z();

            // 处理链接的状态（位置）- 通过边界框坐标计算当前位置
            ignition::math::Vector3d link_closest;
            link_closest.X() =
                ignition::math::clamp(actorPos.X(), link_min_x, link_max_x);
            link_closest.Y() =
                ignition::math::clamp(actorPos.Y(), link_min_y, link_max_y);
            link_closest.Z() =
                ignition::math::clamp(actorPos.Z(), link_min_z, link_max_z);

            // 计算距离
            ignition::math::Vector3d link_offset = link_closest - actorPos;
            double link_dist = link_offset.Length();

            // 更新最近障碍物 - 墙壁链接
            if (link_dist < min_dist) {
              min_dist = link_dist;
              closest_obs = link_closest;
            }
          }
        }
      }
    }
  }

  utils::Vector2d ob(closest_obs.X(), closest_obs.Y());
  sfm_actor_.obstacles1.push_back(ob);
}

//
// handlePedestrians() 函数：检测附近的行人（其他 actor）
//
void PedestrianSFMPlugin::handlePedestrians() {
  other_actors_.clear();
  for (unsigned int i = 0; i < world_->ModelCount(); ++i) {
    physics::ModelPtr model = world_->ModelByIndex(i);
    if (((int)model->GetType() == (int)actor_->GetType()) &&
        model->GetId() != actor_->GetId()) {
      ignition::math::Pose3d model_pose = model->WorldPose();
      ignition::math::Vector3d pos =
          model_pose.Pos() - actor_->WorldPose().Pos();
      if (pos.Length() < people_dist_) {
        sfm::Agent ped;
        ped.id = model->GetId();
        ped.position.set(model_pose.Pos().X(), model_pose.Pos().Y());
        ignition::math::Vector3d rpy = model_pose.Rot().Euler();
        ped.yaw = utils::Angle::fromRadian(rpy.Z());
        ped.radius = sfm_actor_.radius;
        ignition::math::Vector3d linvel = model->WorldLinearVel();
        ped.velocity.set(linvel.X(), linvel.Y());
        ped.linearVelocity = linvel.Length();
        ignition::math::Vector3d angvel = model->WorldAngularVel();
        ped.angularVelocity = angvel.Z();
        if (sfm_actor_.groupId != -1) {
          std::vector<std::string>::iterator it;
          it = find(group_names_.begin(), group_names_.end(), model->GetName());
          if (it != group_names_.end())
            ped.groupId = sfm_actor_.groupId;
          else
            ped.groupId = -1;
        }
        other_actors_.push_back(ped);
      }
    }
  }
}

//
// OnUpdate() 函数：每个仿真更新周期调用，更新位置、计算运动、发布 ROS2 消息
//
void PedestrianSFMPlugin::OnUpdate(const common::UpdateInfo &_info) {
  if (time_init_) {
    if (!pose_init_)
      last_update_ = _info.simTime;
    double dt = (_info.simTime - last_update_).Double();
    ignition::math::Pose3d actor_pose = actor_->WorldPose();

    // 更新障碍物和行人信息
    handleObstacles();
    handlePedestrians();

    // 计算社交力学模型
    sfm::SFM.computeForces(sfm_actor_, other_actors_);
    sfm::SFM.updatePosition(sfm_actor_, dt);

    // 根据社交力学模型更新朝向
    utils::Angle h = sfm_actor_.yaw;
    utils::Angle add = utils::Angle::fromRadian(IGN_PI_2);
    h = h + add;
    double yaw = h.toRadian();

    ignition::math::Vector3d rpy = actor_pose.Rot().Euler();
    utils::Angle current = utils::Angle::fromRadian(rpy.Z());
    double diff = (h - current).toRadian();
    if (std::fabs(diff) > IGN_DTOR(10)) {
      current = current + utils::Angle::fromRadian(diff * 0.005);
      yaw = current.toRadian();
    }

    // 更新 actor 位置和朝向
    actor_pose.Pos().X(sfm_actor_.position.getX());
    actor_pose.Pos().Y(sfm_actor_.position.getY());
    actor_pose.Pos().Z(1.0);
    actor_pose.Rot() = ignition::math::Quaterniond(IGN_PI_2, 0, yaw);

    double distance_traveled =
        (actor_pose.Pos() - actor_->WorldPose().Pos()).Length();
    actor_->SetWorldPose(actor_pose);
    actor_->SetScriptTime(actor_->ScriptTime() +
                          distance_traveled * animation_factor_);

    // ------------------------------
    // 使用 ROS2 发布消息
    // ------------------------------
    geometry_msgs::msg::PoseStamped current_pose;
    current_pose.header.frame_id = "map";
    current_pose.header.stamp = node_->get_clock()->now();
    current_pose.pose.position.x = sfm_actor_.position.getX();
    current_pose.pose.position.y = sfm_actor_.position.getY();
    current_pose.pose.position.z = 1.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, sfm_actor_.yaw.toRadian());
    tf2::convert(q, current_pose.pose.orientation);

    geometry_msgs::msg::Twist current_vel;
    if (!pose_init_) {
      pose_init_ = true;
      last_pose_x_ = current_pose.pose.position.x;
      last_pose_y_ = current_pose.pose.position.y;
      current_vel.linear.x = 0;
      current_vel.linear.y = 0;
    } else {
      double dt = (_info.simTime - last_update_).Double();
      double vx = (current_pose.pose.position.x - last_pose_x_) / dt;
      double vy = (current_pose.pose.position.y - last_pose_y_) / dt;
      last_pose_x_ = current_pose.pose.position.x;
      last_pose_y_ = current_pose.pose.position.y;
      current_vel.linear.x = vx;
      current_vel.linear.y = vy;
    }

    // 发布位姿和速度消息
    pose_pub_->publish(current_pose);
    vel_pub_->publish(current_vel);

    // 发布里程计消息
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "map";
    odom.child_frame_id = actor_->GetName() + "/base_link";
    odom.header.stamp = node_->get_clock()->now();

    // 设置位置
    odom.pose.pose = current_pose.pose;

    // 设置速度
    odom.twist.twist = current_vel;

    // 设置协方差（这里使用一个简单的对角矩阵）
    for (int i = 0; i < 36; i++) {
      odom.pose.covariance[i] = 0.0;
      odom.twist.covariance[i] = 0.0;
    }
    // 位置协方差
    odom.pose.covariance[0] = 0.01;  // x
    odom.pose.covariance[7] = 0.01;  // y
    odom.pose.covariance[14] = 0.01; // z
    odom.pose.covariance[21] = 0.01; // rotation x
    odom.pose.covariance[28] = 0.01; // rotation y
    odom.pose.covariance[35] = 0.01; // rotation z

    // 速度协方差
    odom.twist.covariance[0] = 0.01;  // linear x
    odom.twist.covariance[7] = 0.01;  // linear y
    odom.twist.covariance[14] = 0.01; // linear z
    odom.twist.covariance[21] = 0.01; // angular x
    odom.twist.covariance[28] = 0.01; // angular y
    odom.twist.covariance[35] = 0.01; // angular z

    odom_pub_->publish(odom);

    // 更新本地存储的状态变量
    px_ = current_pose.pose.position.x;
    py_ = current_pose.pose.position.y;
    pz_ = current_pose.pose.position.z;
    vx_ = current_vel.linear.x;
    vy_ = current_vel.linear.y;
    theta_ = yaw;
    last_update_ = _info.simTime;
  }
}

//
// OnStateCallBack() 函数：处理服务请求，返回行人状态
//
void PedestrianSFMPlugin::OnStateCallBack( // 修正：移除返回类型 bool
    const std::shared_ptr<gazebo_sfm_plugin::srv::PedState::Request> req,
    std::shared_ptr<gazebo_sfm_plugin::srv::PedState::Response> resp) {
  // ROS2服务回调不需要返回布尔值
  if (req->name == actor_->GetName()) {
    resp->px = px_;
    resp->py = py_;
    resp->pz = pz_;
    resp->vx = vx_;
    resp->vy = vy_;
    resp->theta = theta_;
    // 不需要返回true
  }
  // 即使不匹配也不要返回false，在ROS2中服务回调直接修改响应对象
}
