
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

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include <nav_quadrotor/rotors_control/extendable_lee_position_controller_node.h>

#include <rotors_control/parameters_ros.h>

namespace rotors_control {
  
  ExtendableLeePositionControllerNode::ExtendableLeePositionControllerNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
  :nh_(nh),
  private_nh_(private_nh),
  controller_active_(false) {
    InitializeParams();
    
    cmd_pose_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_POSE, 1,
      &ExtendableLeePositionControllerNode::CommandPoseCallback, this);
    
    cmd_multi_dof_joint_trajectory_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &ExtendableLeePositionControllerNode::MultiDofJointTrajectoryCallback, this);
    
    odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                  &ExtendableLeePositionControllerNode::OdometryCallback, this);
    
    motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);
    
    command_timer_ = nh_.createTimer(ros::Duration(0), &ExtendableLeePositionControllerNode::TimedCommandCallback, this,
                                     true, false);
  }
  
  ExtendableLeePositionControllerNode::~ExtendableLeePositionControllerNode() { }
  
  void ExtendableLeePositionControllerNode::InitializeParams() {
    
    // Read parameters from rosparam.
    GetRosParameter(private_nh_, "position_gain/x",
                    lee_position_controller_.controller_parameters_.position_gain_.x(),
                    &lee_position_controller_.controller_parameters_.position_gain_.x());
    GetRosParameter(private_nh_, "position_gain/y",
                    lee_position_controller_.controller_parameters_.position_gain_.y(),
                    &lee_position_controller_.controller_parameters_.position_gain_.y());
    GetRosParameter(private_nh_, "position_gain/z",
                    lee_position_controller_.controller_parameters_.position_gain_.z(),
                    &lee_position_controller_.controller_parameters_.position_gain_.z());
    GetRosParameter(private_nh_, "velocity_gain/x",
                    lee_position_controller_.controller_parameters_.velocity_gain_.x(),
                    &lee_position_controller_.controller_parameters_.velocity_gain_.x());
    GetRosParameter(private_nh_, "velocity_gain/y",
                    lee_position_controller_.controller_parameters_.velocity_gain_.y(),
                    &lee_position_controller_.controller_parameters_.velocity_gain_.y());
    GetRosParameter(private_nh_, "velocity_gain/z",
                    lee_position_controller_.controller_parameters_.velocity_gain_.z(),
                    &lee_position_controller_.controller_parameters_.velocity_gain_.z());
    GetRosParameter(private_nh_, "attitude_gain/x",
                    lee_position_controller_.controller_parameters_.attitude_gain_.x(),
                    &lee_position_controller_.controller_parameters_.attitude_gain_.x());
    GetRosParameter(private_nh_, "attitude_gain/y",
                    lee_position_controller_.controller_parameters_.attitude_gain_.y(),
                    &lee_position_controller_.controller_parameters_.attitude_gain_.y());
    GetRosParameter(private_nh_, "attitude_gain/z",
                    lee_position_controller_.controller_parameters_.attitude_gain_.z(),
                    &lee_position_controller_.controller_parameters_.attitude_gain_.z());
    GetRosParameter(private_nh_, "angular_rate_gain/x",
                    lee_position_controller_.controller_parameters_.angular_rate_gain_.x(),
                    &lee_position_controller_.controller_parameters_.angular_rate_gain_.x());
    GetRosParameter(private_nh_, "angular_rate_gain/y",
                    lee_position_controller_.controller_parameters_.angular_rate_gain_.y(),
                    &lee_position_controller_.controller_parameters_.angular_rate_gain_.y());
    GetRosParameter(private_nh_, "angular_rate_gain/z",
                    lee_position_controller_.controller_parameters_.angular_rate_gain_.z(),
                    &lee_position_controller_.controller_parameters_.angular_rate_gain_.z());
    GetVehicleParameters(private_nh_, &lee_position_controller_.vehicle_parameters_);
    lee_position_controller_.InitializeParameters();
  }
  void ExtendableLeePositionControllerNode::Publish() {
  }
  
  void ExtendableLeePositionControllerNode::Reset() {
    auto lock = Lock(trajectory_mutex_);

    // Clear all pending commands.
    controller_active_ = false;
    command_timer_.stop();
    commands_.clear();
    command_waiting_times_.clear();
    next_command_time_ = ros::Time(0);
  }
  
  void ExtendableLeePositionControllerNode::CommandPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& pose_msg) {
    auto lock = Lock(trajectory_mutex_);
    
    Reset();
    // Clear all pending commands.
//     command_timer_.stop();
//     commands_.clear();
//     command_waiting_times_.clear();
    
    mav_msgs::EigenTrajectoryPoint eigen_reference;
    mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
    commands_.push_front(eigen_reference);
    
    {
      auto lock = Lock(controller_mutex_);
      lee_position_controller_.SetTrajectoryPoint(commands_.front());
    }
    commands_.pop_front();
    controller_active_ = true;
    }
    
    void ExtendableLeePositionControllerNode::MultiDofJointTrajectoryCallback(
      const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
      auto lock = Lock(trajectory_mutex_);
      
      Reset();
      // Clear all pending commands.
//       command_timer_.stop();
//       commands_.clear();
//       command_waiting_times_.clear();
//       next_command_time_ = ros::Time(0);
      
      const size_t n_commands = msg->points.size();
      
      if(n_commands < 1){
        ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
        return;
      }
      
      auto cur_time = ros::Time::now();
      
      ros::Duration time_from_trajectory_start = (cur_time > msg->header.stamp) ? cur_time - msg->header.stamp : ros::Duration(0);
      
      mav_msgs::EigenTrajectoryPoint eigen_reference;
      if(n_commands == 1) // || time_from_trajectory_start<=ros::Duration(0)) What was this second part for?
      {
        mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
        commands_.push_front(eigen_reference);
        command_waiting_times_.push_back(ros::Duration(0));
      }
      
      for (size_t i = 1; i < n_commands; ++i) {
        const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = msg->points[i-1];
        const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = msg->points[i];
        
        if(current_reference.time_from_start < time_from_trajectory_start)
        {
          continue;
        }
        
        ros::Duration delta_t;
        if(reference_before.time_from_start < time_from_trajectory_start)
        {
          mav_msgs::eigenTrajectoryPointFromMsg(reference_before, &eigen_reference);
          
          //NOTE: technically, this command is outdated, so perhaps it would be better not to add it?
          // The best option would probably be to interpolate or something
          commands_.push_back(eigen_reference);
          command_waiting_times_.push_back(ros::Duration(0));
          
          delta_t = current_reference.time_from_start - time_from_trajectory_start;
        }
        else
        {
          delta_t = current_reference.time_from_start - reference_before.time_from_start;
        }
        
        ROS_DEBUG_STREAM("Trajectory delta_t=" << delta_t);
        
        mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);
        
        commands_.push_back(eigen_reference);
        command_waiting_times_.push_back(delta_t);
      }
      
      ros::Duration first_command_delay = command_waiting_times_.front();
      if(first_command_delay == ros::Duration(0))
      {
        // We can trigger the first command immediately.
        {
          auto lock = Lock(controller_mutex_);
          lee_position_controller_.SetTrajectoryPoint(commands_.front());
        }
        commands_.pop_front();
        command_waiting_times_.pop_front();
      }
      
      if (commands_.size() > 0) {
        command_timer_.setPeriod(command_waiting_times_.front());
        next_command_time_ = cur_time + command_waiting_times_.front();
        command_waiting_times_.pop_front();
        command_timer_.start();
      }
      controller_active_ = true;
      }
      
      void ExtendableLeePositionControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {
        {
          auto lock = Lock(trajectory_mutex_);
          
          if(commands_.empty()){
            ROS_WARN("Commands empty, this should not happen here");
            return;
          }
          
          //const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
          {
            auto lock = Lock(controller_mutex_);
            lee_position_controller_.SetTrajectoryPoint(commands_.front());
          }
          ROS_DEBUG_STREAM_NAMED("timed_cmd_cb.timer_info", "[Timer callback] Expected: " << e.current_expected << ", Actual: " << e.current_real);
          
          ROS_DEBUG_STREAM_NAMED("timed_cmd_cb.command", "[Timer callback] Command = " << commands_.front().toString());          
          
          commands_.pop_front();
          command_timer_.stop();
          if(!command_waiting_times_.empty()){
            command_timer_.setPeriod(command_waiting_times_.front());
            next_command_time_ = e.current_real + command_waiting_times_.front();
            command_waiting_times_.pop_front();
            command_timer_.start();
          }
        }
        PostTimedCommandCallback(e);
      }
      
      void ExtendableLeePositionControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
        
        ROS_INFO_ONCE("LeePositionController got first odometry message.");
        PreOdometryCallback(odometry_msg);
        
        Eigen::VectorXd ref_rotor_velocities;
        {
          auto lock = Lock(controller_mutex_);
          if(!controller_active_)
          {
            return;
          }
          EigenOdometry odometry;
          eigenOdometryFromMsg(odometry_msg, &odometry);
          lee_position_controller_.SetOdometry(odometry);
          lee_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);
        }
        
        // Todo(ffurrer): Do this in the conversions header.
        mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
        
        actuator_msg->angular_velocities.clear();
        for (int i = 0; i < ref_rotor_velocities.size(); i++)
        {
          actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
        }
        actuator_msg->header.stamp = odometry_msg->header.stamp;
        
        motor_velocity_reference_pub_.publish(actuator_msg);
        
        PostOdometryCallback(actuator_msg);
      }
      
}

// int main(int argc, char** argv) {
//   ros::init(argc, argv, "lee_position_controller_node");
//   
//   ros::NodeHandle nh;
//   ros::NodeHandle private_nh("~");
//   rotors_control::ExtendableLeePositionControllerNode lee_position_controller_node(nh, private_nh);
//   
//   ros::spin();
//   
//   return 0;
// }
