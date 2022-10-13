#ifndef NAV_QUADROTOR_BASIC_TRAJECTORY_WRAPPER_H
#define NAV_QUADROTOR_BASIC_TRAJECTORY_WRAPPER_H

#include<nav_quadrotor/interfaces.h>
//#include<trajectory_based_nav/default_implementations.h>
//#include<nav_quadrotor/trajectory_functions.h>
#include <nav_quadrotor/ni_config_utility.h>
#include <nav_quadrotor/ros_interface_se2_z.h>

namespace nav_quadrotor
{
  
  //geometry_msgs::PoseArray TrajectoryToPoses(const msg_type& trajectory);
  
  //std::vector<geometry_msgs::Twist> TrajectoryToTwists(const msg_type& trajectory);
  
  //msg_type PosesToTrajectory(const geometry_msgs::PoseArray& poses, const std::vector<geometry_msgs::Twist>& twists, const std::vector<ros::Duration>& durations);
  
  
  class BasicTrajectoryWrapper : public TrajectoryWrapper
  {
  protected:
  public:
    se2_z_trajectory::traj_type_ptr trajectory_;
    msg_type::Ptr traj_msg_;
    geometry_msgs::PoseArray pose_array_;
    std::vector<ros::Duration> durations_;
    std::vector<geometry_msgs::Twist> twists_;
    
    std::string source_;
    
  public:
    using Ptr=std::shared_ptr<BasicTrajectoryWrapper>;
    
  protected:
    BasicTrajectoryWrapper() {}
    
  public:
    
    BasicTrajectoryWrapper(se2_z_trajectory::traj_type_ptr traj);
    
    BasicTrajectoryWrapper(geometry_msgs::PoseArray pose_array, std::vector<ros::Duration> durations, std::vector<geometry_msgs::Twist> twists=std::vector<geometry_msgs::Twist>());
    
    virtual ~BasicTrajectoryWrapper() = default;
    
    virtual BasicTrajectoryWrapper makeTrimmed(unsigned int size);
    
    virtual bool trim(unsigned int size);
    
    virtual msg_type::Ptr getTrajectoryMsg() const
    {
      if(!traj_msg_)
      {
        ROS_ERROR_STREAM("No traj_msg_! Either you shouldn't be trying to get this, or it should have been created");
      }
      return traj_msg_;
    }
    
    virtual std::string getSource() const override
    {
      return source_;
    }
        
    virtual geometry_msgs::PoseArray getPoseArray() const
    {
      return pose_array_;
    }
    
    virtual std::vector<ros::Duration> getDurations() const
    {
      return durations_;
    }
    
    virtual std::vector<geometry_msgs::Twist> getTwists() const
    {
      return twists_;
    }
    
    virtual geometry_msgs::PoseArray getPoseArray()
    {
      return pose_array_;
    }
    
    virtual std::vector<ros::Duration> getDurations()
    {
      return durations_;
    }
    
    virtual std::vector<geometry_msgs::Twist> getTwists()
    {
      return twists_;
    }
    
  };

  
  
  
  
} //end namespace nav_quadrotor



#endif // NAV_QUADROTOR_BASIC_TRAJECTORY_WRAPPER_H
