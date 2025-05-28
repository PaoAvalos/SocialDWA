/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <dwa_local_planner/dwa_planner_ros.h>
#include <Eigen/Core>
#include <cmath>
#include <people_msgs/People.h>  // Include for human detection

#include <ros/console.h>
#include <pluginlib/class_list_macros.hpp>
#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include <nav_core/parameter_magic.h>

// Register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dwa_local_planner::DWAPlannerROS, nav_core::BaseLocalPlanner)

namespace dwa_local_planner {

  void DWAPlannerROS::reconfigureCB(DWAPlannerConfig &config, uint32_t level) {
      if (setup_ && config.restore_defaults) {
        config = default_config_;
        config.restore_defaults = false;
      }
      if (!setup_) {
        default_config_ = config;
        setup_ = true;
      }

      // Update generic local planner params
      base_local_planner::LocalPlannerLimits limits;
      limits.max_vel_trans = config.max_vel_trans;
      limits.min_vel_trans = config.min_vel_trans;
      limits.max_vel_x = config.max_vel_x;
      limits.min_vel_x = config.min_vel_x;
      limits.max_vel_y = config.max_vel_y;
      limits.min_vel_y = config.min_vel_y;
      limits.max_vel_theta = config.max_vel_theta;
      limits.min_vel_theta = config.min_vel_theta;
      limits.acc_lim_x = config.acc_lim_x;
      limits.acc_lim_y = config.acc_lim_y;
      limits.acc_lim_theta = config.acc_lim_theta;
      limits.acc_lim_trans = config.acc_lim_trans;
      limits.xy_goal_tolerance = config.xy_goal_tolerance;
      limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
      limits.prune_plan = config.prune_plan;
      limits.trans_stopped_vel = config.trans_stopped_vel;
      limits.theta_stopped_vel = config.theta_stopped_vel;
      planner_util_.reconfigureCB(limits, config.restore_defaults);

      // Update dwa specific configuration
      dp_->reconfigure(config);
      
      // Update human-aware parameters
      human_radius_threshold_ = config.human_radius_threshold;
      human_speed_factor_ = config.human_speed_factor;
      human_scaling_factor_ = config.human_scaling_factor;
      direct_penalty_ = config.direct_penalty;
      heading_tolerance_ = config.heading_tolerance;
  }

  DWAPlannerROS::DWAPlannerROS() : initialized_(false),
      odom_helper_("odom"), setup_(false) {
  }

  void DWAPlannerROS::initialize(
      std::string name,
      tf2_ros::Buffer* tf,
      costmap_2d::Costmap2DROS* costmap_ros) {
    if (!isInitialized()) {
      ros::NodeHandle private_nh("~/" + name);
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ros_->getRobotPose(current_pose_);

      // Initialize human detection subscriber
      human_sub_ = private_nh.subscribe("/detected_humans", 1, &DWAPlannerROS::humanCallback, this);

      // Make sure to update the costmap we'll use for this cycle
      costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
      planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

      // Create the actual planner that we'll use
      dp_ = boost::shared_ptr<DWAPlanner>(new DWAPlanner(name, &planner_util_));

      if (private_nh.getParam("odom_topic", odom_topic_)) {
        odom_helper_.setOdomTopic(odom_topic_);
      }
      
      // Load human-aware parameters with defaults
      private_nh.param("human_radius_threshold", human_radius_threshold_, 2.0);
      private_nh.param("human_speed_factor", human_speed_factor_, 0.5);
      private_nh.param("human_scaling_factor", human_scaling_factor_, 0.1);
      private_nh.param("direct_penalty", direct_penalty_, 10.0);
      private_nh.param("heading_tolerance", heading_tolerance_, 0.3);

      initialized_ = true;

      // Warn about deprecated parameters
      nav_core::warnRenamedParameter(private_nh, "max_vel_trans", "max_trans_vel");
      nav_core::warnRenamedParameter(private_nh, "min_vel_trans", "min_trans_vel");
      nav_core::warnRenamedParameter(private_nh, "max_vel_theta", "max_rot_vel");
      nav_core::warnRenamedParameter(private_nh, "min_vel_theta", "min_rot_vel");
      nav_core::warnRenamedParameter(private_nh, "acc_lim_trans", "acc_limit_trans");
      nav_core::warnRenamedParameter(private_nh, "theta_stopped_vel", "rot_stopped_vel");

      dsrv_ = new dynamic_reconfigure::Server<DWAPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<DWAPlannerConfig>::CallbackType cb = [this](auto& config, auto level){ reconfigureCB(config, level); };
      dsrv_->setCallback(cb);
    } else {
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }
  
  bool DWAPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if (!isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    // When we get a new plan, clear any latch on goal tolerances
    latchedStopRotateController_.resetLatching();
    ROS_INFO("Got new plan");
    return dp_->setPlan(orig_global_plan);
  }

  bool DWAPlannerROS::isGoalReached() {
    if (!isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    if (!costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    if (latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
      ROS_INFO("Goal reached");
      return true;
    } else {
      return false;
    }
  }

  void DWAPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, l_plan_pub_);
  }

  void DWAPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, g_plan_pub_);
  }

  DWAPlannerROS::~DWAPlannerROS() {
    // Clean up
    delete dsrv_;
  }

  void DWAPlannerROS::humanCallback(const people_msgs::People::ConstPtr& msg) {
    boost::mutex::scoped_lock lock(human_mutex_);
    humans_ = *msg;
  }

  double DWAPlannerROS::calculateHumanCost(const geometry_msgs::PoseStamped& robot_pose) {
    double total_cost = 0.0;
    if (humans_.people.empty()) return total_cost;

    // Get current robot position
    const double rx = robot_pose.pose.position.x;
    const double ry = robot_pose.pose.position.y;
    tf2::Quaternion q(
        robot_pose.pose.orientation.x,
        robot_pose.pose.orientation.y,
        robot_pose.pose.orientation.z,
        robot_pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Process each detected human
    for (const auto& human : humans_.people) {
      const double hx = human.position.x;
      const double hy = human.position.y;
      
      // Calculate distance to human
      const double dx = hx - rx;
      const double dy = hy - ry;
      const double distance = std::hypot(dx, dy);
      
      // Direct avoidance penalty
      if (distance < human_radius_threshold_) {
        // Distance-based cost
        total_cost += (human_radius_threshold_ - distance) * human_scaling_factor_;
        
        // Heading penalty
        const double heading_to_human = std::atan2(dy, dx);
        if (std::abs(heading_to_human - yaw) < heading_tolerance_) {
          total_cost += direct_penalty_;
        }
      }
      
      // Velocity-based cost
      const double human_speed = std::hypot(human.velocity.x, human.velocity.y);
      const double dynamic_inflation = 0.5 + human_speed * human_speed_factor_;
      if (distance < dynamic_inflation) {
        total_cost += (dynamic_inflation - distance) * human_speed * human_scaling_factor_;
      }
    }
    return total_cost;
  }

  bool DWAPlannerROS::dwaComputeVelocityCommands(geometry_msgs::PoseStamped &global_pose, geometry_msgs::Twist& cmd_vel) {
    if (!isInitialized()) {
      ROS_ERROR("Planner not initialized");
      return false;
    }

    geometry_msgs::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    // Calculate human-aware cost
    const double human_cost = calculateHumanCost(global_pose);
    
    // Velocity adaptation based on human proximity
    DWAPlannerConfig config = dp_->getConfig();
    double adaptive_max_vel = config.max_vel_x;
    if (human_cost > 0) {
      adaptive_max_vel = config.max_vel_x / (1.0 + human_cost * human_scaling_factor_);
      adaptive_max_vel = std::max(config.min_vel_x, adaptive_max_vel);  // Clamp to minimum
      ROS_DEBUG("Adaptive velocity: %.2f (cost: %.2f)", adaptive_max_vel, human_cost);
    }
    
    // Set adaptive velocity in planner
    dp_->setMaxVelX(adaptive_max_vel);

    // Compute trajectory with human-aware modifications
    geometry_msgs::PoseStamped drive_cmds;
    drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID();
    base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds);

    // Pass along drive commands
    cmd_vel.linear.x = drive_cmds.pose.position.x;
    cmd_vel.linear.y = drive_cmds.pose.position.y;
    cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

    // Handle failure cases
    std::vector<geometry_msgs::PoseStamped> local_plan;
    if (path.cost_ < 0) {
      ROS_DEBUG("DWA planner failed to find valid plan");
      publishLocalPlan(local_plan);
      return false;
    }

    ROS_DEBUG("Found valid velocity command: (%.2f, %.2f, %.2f)", 
              cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // Publish local plan
    for (unsigned int i = 0; i < path.getPointsSize(); ++i) {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      geometry_msgs::PoseStamped p;
      p.header.frame_id = costmap_ros_->getGlobalFrameID();
      p.header.stamp = ros::Time::now();
      p.pose.position.x = p_x;
      p.pose.position.y = p_y;
      p.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, p_th);
      tf2::convert(q, p.pose.orientation);
      local_plan.push_back(p);
    }
    publishLocalPlan(local_plan);
    return true;
  }

  bool DWAPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    if (!costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }
    
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if (!planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
      ROS_ERROR("Could not get local plan");
      return false;
    }

    if (transformed_plan.empty()) {
      ROS_WARN("Received empty transformed plan");
      return false;
    }
    ROS_DEBUG("Received transformed plan with %zu points", transformed_plan.size());

    // Update plan with human-aware costs
    dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan, costmap_ros_->getRobotFootprint());

    if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)) {
      // Handle goal reached
      std::vector<geometry_msgs::PoseStamped> local_plan;
      publishGlobalPlan(transformed_plan);
      publishLocalPlan(local_plan);
      base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
      return latchedStopRotateController_.computeVelocityCommandsStopRotate(
          cmd_vel,
          limits.getAccLimits(),
          dp_->getSimPeriod(),
          &planner_util_,
          odom_helper_,
          current_pose_,
          [this](auto pos, auto vel, auto vel_samples) { 
            return dp_->checkTrajectory(pos, vel, vel_samples); 
          });
    } else {
      // Normal planning with human-aware DWA
      bool isOk = dwaComputeVelocityCommands(current_pose_, cmd_vel);
      if (isOk) {
        publishGlobalPlan(transformed_plan);
      } else {
        ROS_WARN("DWA planner failed to produce path");
        std::vector<geometry_msgs::PoseStamped> empty_plan;
        publishGlobalPlan(empty_plan);
      }
      return isOk;
    }
  }
};