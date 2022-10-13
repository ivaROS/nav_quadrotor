
/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef NAV_QUADROTOR_ROTORS_CONTROL_EXTENDABLE_LEE_POSITION_CONTROLLER_NODE_H
#define NAV_QUADROTOR_ROTORS_CONTROL_EXTENDABLE_LEE_POSITION_CONTROLLER_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <rotors_control/common.h>
#include <rotors_control/lee_position_controller.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>

namespace rotors_control {
  
  class ExtendableLeePositionControllerNode {
  public:
    ExtendableLeePositionControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~ExtendableLeePositionControllerNode();
    
    virtual void InitializeParams();
    virtual void Publish();
    
  protected:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    LeePositionController lee_position_controller_;
    
    std::string namespace_;
    
    // subscribers
    ros::Subscriber cmd_trajectory_sub_;
    ros::Subscriber cmd_multi_dof_joint_trajectory_sub_;
    ros::Subscriber cmd_pose_sub_;
    ros::Subscriber odometry_sub_;
    
    ros::Publisher motor_velocity_reference_pub_;
    
    mav_msgs::EigenTrajectoryPointDeque commands_;
    std::deque<ros::Duration> command_waiting_times_;
    ros::Timer command_timer_;
    ros::Time next_command_time_;
    
    bool controller_active_;
    
    virtual void Reset();
    virtual void TimedCommandCallback(const ros::TimerEvent& e);
    virtual void PostTimedCommandCallback(const ros::TimerEvent& e){}
        
    virtual void MultiDofJointTrajectoryCallback(
      const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);
    
    virtual void CommandPoseCallback(
      const geometry_msgs::PoseStampedConstPtr& pose_msg);
    
    virtual void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
    
    virtual void PostOdometryCallback(const mav_msgs::ActuatorsPtr&) {}
    virtual void PreOdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {}
    
    template<typename T> 
    auto Lock(T& m) -> decltype(boost::unique_lock<T>(m))
    { 
      return boost::unique_lock<T>(m); 
    }
    //typedef boost::mutex::scoped_lock Lock;
    boost::recursive_mutex trajectory_mutex_;
    boost::mutex controller_mutex_;
    
  };
}

#endif // NAV_QUADROTOR_ROTORS_CONTROL_EXTENDABLE_LEE_POSITION_CONTROLLER_NODE_H
