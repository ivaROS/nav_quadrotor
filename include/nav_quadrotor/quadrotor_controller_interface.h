#ifndef NAV_QUADROTOR_QUADROTOR_CONTROLLER_INTERFACE_H
#define NAV_QUADROTOR_QUADROTOR_CONTROLLER_INTERFACE_H

#include <nav_quadrotor/rotors_control/extendable_lee_position_controller_node.h>
#include <nav_quadrotor/interfaces.h>

namespace nav_quadrotor
{
  
  class QuadrotorControllerInterface : public rotors_control::ExtendableLeePositionControllerNode, public TrajectoryController
  {
    ros::NodeHandle nh_, pnh_;
    tf2_utils::TransformManager tfm_;
    nav_msgs::Odometry::ConstPtr curr_odom_;
    std::string frame_id_;
    
    ros::Publisher remaining_traj_pub_, commanded_traj_pub_, commanded_pose_pub_, current_traj_pub_;
    
  public:
    //using msg_type = trajectory_msgs::MultiDOFJointTrajectory
    using Ptr=std::shared_ptr<QuadrotorControllerInterface>;
    
  public:
    QuadrotorControllerInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm=tf2_utils::TransformManager(true));
    
    virtual void PreOdometryCallback(const nav_msgs::Odometry::ConstPtr& odometry_msg);
    
    virtual void PostTimedCommandCallback(const ros::TimerEvent& e);
    
    virtual geometry_msgs::PoseArray GetRemainingPoses(ros::Time time=ros::Time::now(), std::vector<ros::Duration>* times=nullptr, std::vector<geometry_msgs::Twist>* twists=nullptr);
    
    virtual TrajectoryWrapper::Ptr getCurrentTrajectory();
    
    virtual void setTrajectory(const msg_type::ConstPtr& trajectory_msg);
    
    virtual nav_msgs::Odometry::ConstPtr getCurrentOdom();
    
    virtual void stop(bool force_stop=false);
  
  };
} //end namespace nav_quadrotor

#endif //NAV_QUADROTOR_QUADROTOR_CONTROLLER_INTERFACE_H
