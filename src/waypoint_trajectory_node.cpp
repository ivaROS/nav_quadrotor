

#include <nav_quadrotor/path_contour/path_contour.h>
#include <ootp/config.hpp>

#include <tf/transform_datatypes.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Transform.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>


#include <message_filters/cache.h>
#include <message_filters/subscriber.h>

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
  
  contour::State convertToState(const nav_msgs::Odometry& odom)
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
    return initial_state;
  }
  
  geometry_msgs::Pose convertToPose(const trajectory_msgs::MultiDOFJointTrajectoryPoint& point)
  {
    geometry_msgs::Pose pose;
    const auto& trans = point.transforms[0].translation;
    pose.position.x = trans.x;
    pose.position.y = trans.y;
    pose.position.z = trans.z;
    pose.orientation = point.transforms[0].rotation;
    return pose;
  }
  
  geometry_msgs::PoseArray convertToPoses(const trajectory_msgs::MultiDOFJointTrajectory& points)
  {
    geometry_msgs::PoseArray pose_array;
    pose_array.header = points.header;
    for(const auto& point : points.points)
    {
      pose_array.poses.emplace_back(convertToPose(point));
    }
    return pose_array;
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
    point.accelerations.push_back(acceleration);
    
    point.time_from_start = ros::Duration(t);
    
    return point;
  }
  
  void printState(contour::State state, double t)
  {
    ROS_DEBUG_STREAM("[" << t << "] Position: " << state.position.transpose() << ", Yaw: " << state.yaw << ", Velocity: " << state.velocity.transpose() << ", Omega: " << state.omega);
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
    
    
    
    
    
    
    
    trajectory_msgs::MultiDOFJointTrajectory GetTrajectory(const nav_msgs::Odometry& odom, const geometry_msgs::PoseStamped& goal)
    {
      SetCurrentState(odom);
      AddWaypoints(odom, goal);
      AddWaypoint(goal);
      
      trajectory_msgs::MultiDOFJointTrajectory traj;
      traj.header = goal.header;
      
      contour::State state = traj_handler_.GetState();
      Eigen::Vector3d ref_pos = traj_handler_.getLastWaypoint();
      double ref_yaw = traj_handler_.getLastYawWaypoint();
      double t = 0;
      
      int counter = 0;
      bool keep_going = true;
      while(keep_going && counter < 100)
      {
        traj.points.emplace_back(getTrajectoryPoint(state,t));
        
        printState(state,t);
        
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
        counter++;
      }
      
      ROS_INFO_STREAM("Desired state: " << ref_pos << ", yaw=" << ref_yaw);
      
      return traj;
    }
    
    
    
  private:
    void SetCurrentState(const nav_msgs::Odometry& odom)
    {
      contour::State initial_state = convertToState(odom);
      
      
      traj_handler_.SetInitialState(initial_state, Config::path_error(), Config::yaw_error() );
    }  
    
    void AddWaypoint(const geometry_msgs::PoseStamped& pose)
    {
      const auto& pos = pose.pose.position;
      double yaw = getYaw(pose.pose.orientation);
      
      Eigen::Vector3d waypoint(pos.x,pos.y,pos.z+1.5);
      traj_handler_.PushWaypoint(waypoint, Config::path_error(), yaw, Config::yaw_error());
    }
    
    void AddWaypoints(const nav_msgs::Odometry& odom, const geometry_msgs::PoseStamped& pose)
    {
      const auto& pos = pose.pose.position;
      double yaw = getYaw(pose.pose.orientation);
      
      Eigen::Vector3d endpoint(pos.x,pos.y,pos.z+1.5);
      
      contour::State initial_state = convertToState(odom);
      
      auto delta_v = endpoint - initial_state.position;
      double angle = std::atan2(delta_v[1], delta_v[0]);
      
      double offset = 1;
      
      Eigen::Vector3d waypoint = initial_state.position + offset * delta_v/delta_v.norm();
      
      traj_handler_.PushWaypoint(waypoint, Config::path_error(), angle, Config::yaw_error());
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
  
  

  class WaypointTrajectoryGeneratorInterface
  {
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Cache<nav_msgs::Odometry> odom_cache_;
    ros::Publisher trajectory_publisher_, path_pub_;
    ros::Subscriber goal_sub_;
    
  public:
    WaypointTrajectoryGeneratorInterface(ros::NodeHandle nh):
      odom_sub_(),
      odom_cache_(odom_sub_, 1)
    {
      trajectory_publisher_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/desired_trajectory", 10);
      path_pub_ = nh.advertise<geometry_msgs::PoseArray>("/desired_poses", 10);
      odom_sub_.subscribe(nh, "/hummingbird/odometry_sensor1/odometry", 1);
      goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, &WaypointTrajectoryGeneratorInterface::GoalCB, this);
    }
    
    void GoalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
    {
      ros::Time anytime;
      nav_msgs::Odometry::ConstPtr odom = odom_cache_.getElemAfterTime(anytime);
      if(!odom)
      {
        ROS_WARN("No odometry!");
        return;
      }
      WaypointTrajectoryGenerator generator;
      
      trajectory_msgs::MultiDOFJointTrajectory trajectory = generator.GetTrajectory(*odom, *goal);
      
      trajectory_publisher_.publish(trajectory);
      
      path_pub_.publish(convertToPoses(trajectory));
    }
    
  };
  
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_trajectory_node");
  ros::NodeHandle nh;
  std::string name = ros::this_node::getName();
  nav_quadrotor::WaypointTrajectoryGeneratorInterface generator(nh);
  ROS_INFO_STREAM("Started node [" << name << "]");
  ros::spin();
  
  ROS_INFO_STREAM("Program exiting");
  return 0;
  
}
