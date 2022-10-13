#ifndef NAV_QUADROTOR_SAFETY_CONTROLLER_NODE_H
#define NAV_QUADROTOR_SAFETY_CONTROLLER_NODE_H

#include <nav_quadrotor/rotors_control/extendable_lee_position_controller_node.h>

namespace rotors_control {
  
  class SafetyControllerNode : public ExtendableLeePositionControllerNode
  {
  public:
    SafetyControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh):
      ExtendableLeePositionControllerNode(nh, private_nh),
      hold_pose_(false)
    {
      double max_delay=0.2;
      private_nh.getParam("max_delay", max_delay);
      max_time_between_cmds_ = ros::Duration(max_delay);
      
      ROS_INFO_STREAM("Max Delay = " << max_time_between_cmds_.toSec() << "s");
      
      motor_velocity_reference_in_ = private_nh_.subscribe("commands_in", 5, &SafetyControllerNode::MotorVelocityCallback, this);
    }
    
  protected:
    virtual void CommandPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) 
    {
      {
        auto lock = Lock(status_mutex_);
        hold_pose_ = true;
      }
      
      ExtendableLeePositionControllerNode::CommandPoseCallback(pose_msg);
    }
    
    virtual void OdometryCallback(const nav_msgs::Odometry::ConstPtr& odometry_msg)
    {
      bool hold_pose=false;
      
      geometry_msgs::PoseStamped::Ptr des_pose;
      
      {
        auto lock = Lock(status_mutex_);
        if(hold_pose_)
        {
          hold_pose = true;
        }
        else if(odometry_msg->header.stamp - time_of_last_cmds_ > max_time_between_cmds_)
        {
          des_pose = boost::make_shared<geometry_msgs::PoseStamped>();
          des_pose->pose = odometry_msg->pose.pose;
          des_pose->header = odometry_msg->header;
          
          hold_pose_ = true;
          hold_pose = true;
        }
      }
      
      if(hold_pose)
      {
        //ROS_INFO("MAINTAIN POSE!");
        if(des_pose)
        {
          ExtendableLeePositionControllerNode::CommandPoseCallback(des_pose);
          ROS_INFO_STREAM("Safety Controller detected lack of commands, holding at present pose");
        }
        
        ExtendableLeePositionControllerNode::OdometryCallback(odometry_msg);
      }
      else
      {
        //ROS_INFO("DON'T MAINTAIN POSE!");
      }
      
    }
    
    virtual void MotorVelocityCallback(const mav_msgs::Actuators::ConstPtr& cmds)
    {
      bool non_zero_cmds = false;
      for(const auto& cmd : cmds->angular_velocities)
      {
        if(cmd != 0)
        {
          non_zero_cmds = true;
          break;
        }
      }
      
      if(!non_zero_cmds)
      {
        ROS_DEBUG("Another controller has started, but it hasn't issued any nonzero commands, so still holding pose");
        return;
      }
      bool stop_holding = false;
      {
        auto lock = Lock(status_mutex_);
        time_of_last_cmds_ = cmds->header.stamp;
        if(hold_pose_)
        {
          hold_pose_ = false;
          stop_holding = true;
        }
      }
      motor_velocity_reference_pub_.publish(cmds);
      ROS_INFO_STREAM_COND_NAMED(stop_holding, "safety_controller.motor_velocity_cb", "[SafetyController] detected commands, disabling pose holding");
    }
    

  protected:

    ros::Subscriber motor_velocity_reference_in_;
    
    ros::Time time_of_last_cmds_;
    ros::Duration max_time_between_cmds_;
    
    bool hold_pose_;
    
    boost::mutex status_mutex_;
  };
}

#endif // NAV_QUADROTOR_ROTORS_CONTROL_EXTENDABLE_LEE_POSITION_CONTROLLER_NODE_H


int main(int argc, char** argv) {
  ros::init(argc, argv, "safety_controller_node");
  
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  rotors_control::SafetyControllerNode lee_position_controller_node(nh, private_nh);
  
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
  
  return 0;
}
