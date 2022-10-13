

#include <nav_quadrotor/path_contour/path_contour.h>
#include <ootp/config.hpp>

#include <tf/transform_datatypes.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Transform.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

namespace nav_quadrotor
{

  
  geometry_msgs::Quaternion getQuaternionFromYaw(double yaw)
  {
    double roll=0, pitch=0;
    return tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
  }
  
  
  double getYaw(const geometry_msgs::Quaternion& quaternion)
  {
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(quaternion, quat);
    
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    return yaw;
  }
  
  
  trajectory_msgs::MultiDOFJointTrajectoryPoint getTrajectoryPoint(const contour::State& state, double t)
  {
    trajectory_msgs::MultiDOFJointTrajectoryPoint point;
    
    geometry_msgs::Transform transform;
    auto& pos = transform.translation;
    pos.x = state.position[0];
    pos.y = state.position[1];
    pos.z = state.position[2];
    transform.rotation = getQuaternionFromYaw(state.yaw);
    point.transforms.push_back(transform);
    
    geometry_msgs::Twist velocity;
    auto& lin = velocity.linear;
    lin.x = state.velocity[0];
    lin.y = state.velocity[1];
    lin.z = state.velocity[2];
    velocity.angular.z = state.omega;
    point.velocities.push_back(velocity);
    
    geometry_msgs::Twist acceleration;
    auto& acc = acceleration.linear;
    acc.x = state.acceleration[0];
    acc.y = state.acceleration[1];
    acc.z = state.acceleration[2];
    //acceleration.angular.z = state.omega;
    point.velocities.push_back(acceleration);
    
    point.time_from_start = ros::Duration(t);
    
    return point;
  }
  

class WaypointTrajectoryGenerator
{
  static const uint8_t k_dimension = 3;
  // kNStates = number of states
  typedef enum {kIdle =0, kAngleFollowing, kWalkFollowing, kLost, kNStates}eStates;  
  typedef enum {kPos =0, kVel, kAcc, kJerk, kNDynbamics}eDynbamics;
  
  using Config = ootp::Config;
  
  contour::PathDynamics traj_handler_;

  
  public:
    WaypointTrajectoryGenerator() :
      traj_handler_(k_dimension, Config::arc_velocity() )
    {
      traj_handler_.SetGains( Config::path_gains(), Config::yaw_gains(), Config::path_error() );
      
    }

  
  
  
  
  
  
  trajectory_msgs::MultiDOFJointTrajectory GetTrajectory(const nav_msgs::Odometry& odom, geometry_msgs::PoseStamped goal)
  {
    SetCurrentState(odom);
    AddWaypoint(goal);
    
    trajectory_msgs::MultiDOFJointTrajectory traj;
    
    contour::State state = traj_handler_.GetState();
    Eigen::Vector3d ref_pos = traj_handler_.getLastWaypoint();
    double ref_yaw = traj_handler_.getLastYawWaypoint();
    double t = 0;
    
    bool keep_going = true;
    while(keep_going)
    {
      traj.points.emplace_back(getTrajectoryPoint(state,t));
      
      double path_error_val = ( - state.position).norm();
      double yaw_error_val = ref_yaw - state.yaw;
      
      if(path_error_val < Config::path_error() && fabs(yaw_error_val) < Config::yaw_error())
      {
        keep_going = false;
      }
      else
      {
        state = UpdateStates(t);
      }
    }
    
    return traj;
  }
  
  
  
private:
  void SetCurrentState(const nav_msgs::Odometry& odom)
  {
    contour::State initial_state;
    const auto& pos = odom.pose.pose.position;
    const auto& vel = odom.twist.twist.linear;
    
    Eigen::Vector3d eig_pos(pos.x,pos.y,pos.z);
    Eigen::Vector3d eig_vel(vel.x,vel.y,vel.z);
    
    initial_state.position = eig_pos;
    initial_state.velocity = eig_vel;
    initial_state.yaw = getYaw(odom.pose.pose.orientation);
    initial_state.omega = odom.twist.twist.angular.z;
    
    traj_handler_.SetInitialState(initial_state, Config::path_error(), Config::yaw_error() );
  }  
  
  void AddWaypoint(const geometry_msgs::PoseStamped pose)
  {
    const auto& pos = pose.pose.position;
    double yaw = getYaw(pose.pose.orientation);
    
    Eigen::Vector3d waypoint(pos.x,pos.y,pos.z);
    traj_handler_.PushWaypoint(waypoint, Config::path_error(), yaw, Config::yaw_error());
  }
    
  
  contour::State UpdateStates(double& t)
  {
    contour::State st;
    for(int step = 0 ; step < 5 ; step++)
    {
      st = traj_handler_.UpdateDynamics(0.005); // fix step time
    }
    
    t+=5*0.005;
    
    contour::State stt;
    stt.position = st.position;
    stt.velocity = st.velocity*5;
    stt.acceleration = st.acceleration*5*5;
    stt.jerk = st.jerk*5*5*5;
    stt.yaw = st.yaw;
    stt.omega = st.omega*5;
    
    return stt;
  }
  
  
};

}
