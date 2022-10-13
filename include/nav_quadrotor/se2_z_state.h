
#ifndef QUADROTOR_TRAJECTORY_GENERATOR_SE2_Z_STATE_H
#define QUADROTOR_TRAJECTORY_GENERATOR_SE2_Z_STATE_H

#include <traj_generator.h>
#include <Eigen/Eigen>

//It may be cleaner to do the conversion elsewhere, but this should be more straightforward
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace nav_quadrotor
{
  namespace se2_z_trajectory 
  {
    inline
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
    
    
    
    template<typename T>
    class Point
    {
    public:
      T x,y,z;
      
      Point(T &x, T &y, T &z) :
        x(x),
        y(y),
        z(z)
      {}
      
      template<typename S>
      void from(const S& point)
      {
        x = point.x;
        y = point.y;
        z = point.z;
      }
      
//       void from(const geometry_msgs::Point& point)
//       {
//         x = point.x;
//         y = point.y;
//         z = point.z;
//       }
//       
//       void from(const geometry_msgs::Vector3& point)
//       {
//         x = point.x;
//         y = point.y;
//         z = point.z;
//       }

//       void from(const geometry_msgs::Pose& pose)
//       {
//         from(pose.position);
//       }
      
      
      //Functions converting states to other types
      template<typename S>
      void to(S& point) const
      {
        point.x = x;
        point.y = y;
        point.z = z;
      }
      
      void to(geometry_msgs::Vector3& point) const
      {
        point.x = x;
        point.y = y;
        point.z = z;
      }
      
      
      
//       void to(geometry_msgs::Pose& pose) const
//       {
//         to(pose.position);
//         to(pose.orientation);
//       }
      
    };
    
    template<typename T>
    class Pose : public Point<T>
    {
    public:
      T theta;
      
      Pose(T &x, T &y, T &z, T &theta) :
        Point<T>(x,y,z),
        theta(theta)
      {}
      
      void from(const geometry_msgs::Quaternion& quaternion)
      {
        theta = getYaw(quaternion);
      }
      
      void from(const geometry_msgs::Pose& pose)
      {
        from(pose.position);
        from(pose.orientation);
      }
      
      template <typename S>
      void from(const S& x)
      {
        Point<T>::from(x);
      }
      
//       void from(const nav_msgs::Odometry& odom)
//       {
//         from(odom.pose.pose);
//       }
      
      void to(geometry_msgs::Quaternion& quaternion) const
      {
        double yaw = theta;
        double roll = 0;
        double pitch = 0;
        quaternion = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
      }
      
      void to(geometry_msgs::Pose& pose) const
      {
        to(pose.position);
        to(pose.orientation);
      }
      
      void to(geometry_msgs::Transform& transform) const
      {
        to(transform.translation);
        to(transform.rotation);
      }
      
      template <typename S>
      void to(S& x) const
      {
        Point<T>::to(x);
      }
      
    };
    
    //Note: this representation assumes that twist is given in the body frame, which does not seem to be the case for quadrotor odometry messages or trajectory messages
    template<typename T>
    class Twist
    {
    public:
      T x,z,theta;
      
      Twist(T &x, T &z, T &theta) :
        x(x),
        z(z),
        theta(theta)
      {}
      
      void from(const geometry_msgs::Twist& twist)
      {
        x = std::sqrt(twist.linear.x*twist.linear.x + twist.linear.y*twist.linear.y);
        z = twist.linear.z;
        theta = twist.angular.z;
      }
      
      void to(geometry_msgs::Twist& twist, double yaw=0) const
      {
        twist.linear.x = x*std::cos(yaw);
        twist.linear.y = x*std::sin(yaw);
        twist.linear.z = z;
        twist.angular.z = theta;
      }
      
    };
    
    template<typename T>
    class ActualState
    {
    public:
      Pose<T> pose;
      Twist<T> twist;
      
      ActualState(T &x, T &y, T &z, T &theta,
                        T &v, T &vz, T &w):
        pose(x,y,z,theta),
        twist(v,vz,w)
      {}
      
      //TODO: Remove?
      ActualState(const ActualState<T>& other):
        pose(other.pose),
        twist(other.twist)
      {}
      
      
      template<typename S>
      void from(const S& point)
      {
        pose.from(point);
      }
      
      void from(const geometry_msgs::Quaternion& quaternion)
      {
        pose.from(quaternion);
      }
      
      void from(const geometry_msgs::Twist& t)
      {
        twist.from(t);
      }
      
      void from(const geometry_msgs::Pose& p)
      {
        pose.from(p);
      }
      
      void from(const nav_msgs::Odometry& odom)
      {
        from(odom.pose.pose);
        from(odom.twist.twist);
      }
      
      
      //Functions converting states to other types
      template<typename S>
      void to(S& point) const
      {
        pose.to(point);
      }
      
      void to(geometry_msgs::Quaternion& quaternion) const
      {
        pose.to(quaternion);
      }
      
      void to(geometry_msgs::Twist& t) const
      {
        twist.to(t, pose.theta);
      }
      
      void to(geometry_msgs::Pose& p) const
      {
        pose.to(p);
      }
      
      void to(geometry_msgs::Transform& transform) const
      {
        pose.to(transform);
      }
      
};

inline
void transformTwist(const geometry_msgs::Twist& msg_in, geometry_msgs::Twist& msg_out, const geometry_msgs::Transform& transform_msg)
{
  tf2::Vector3 twist_rot, twist_vel;
  tf2::convert(msg_in.angular, twist_rot);
  tf2::convert(msg_in.linear, twist_vel);
  
  tf2::Transform transform;
  tf2::convert(transform_msg, transform);
  //transform = transform.inverse();
  
  tf2::Vector3 out_rot = transform.getBasis() * twist_rot;
  tf2::Vector3 out_vel = transform.getBasis()* twist_vel;
  
  tf2::convert(out_rot, msg_out.angular);
  tf2::convert(out_vel, msg_out.linear);
  ROS_INFO_STREAM("transform: [" << transform.getOrigin().x() << "," << transform.getOrigin().y() << "," << transform.getOrigin().z() << "] (" << getYaw(transform_msg.rotation) << "), twist_vel: [" << twist_vel.x() << "," << twist_vel.y() << "," << twist_vel.z() << "], out_vel: [" << out_vel.x() << "," << out_vel.y() << "," << out_vel.z() << "], twist_rot: [" << twist_rot.x() << "," << twist_rot.y() << "," << twist_rot.z() << "], out_rot: [" << out_rot.x() << "," << out_rot.y() << "," << out_rot.z() << "]");
}  
  
class se2_z_state : public trajectory_generator::TrajectoryState<se2_z_state,11>
{
public:
  //TODO: Discourage external code from referring to elements by index. Temporarily giving access to ease porting of near identity code
  enum STATE_INDICES { X_IND=0, Y_IND=1, Z_IND=2, THETA_IND=3, V_IND=4, VZ_IND=5, W_IND=6, XD_IND=7, YD_IND=8, ZD_IND=9, LAMBDA_IND=10};
  
public:  
  typedef trajectory_msgs::MultiDOFJointTrajectory trajectory_msg_t;
  using VectorReference = trajectory_generator::VectorReference<se2_z_state::array>;
  
  //decltype(data)
  trajectory_generator::VectorReference<array> x,y,z,theta,v,vz,w,xd,yd,zd,lambda;
  
  using Actual = ActualState<VectorReference>;
  using Desired = Point<VectorReference>;
  
  Actual actual;
  Desired desired;
  
  se2_z_state() :
    x(data,se2_z_state::X_IND),
    y(data,se2_z_state::Y_IND),
    z(data,se2_z_state::Z_IND),
    theta(data,se2_z_state::THETA_IND),
    v(data,se2_z_state::V_IND),
    vz(data,se2_z_state::VZ_IND),
    w(data,se2_z_state::W_IND),
    xd(data,se2_z_state::XD_IND),
    yd(data,se2_z_state::YD_IND),
    zd(data,se2_z_state::ZD_IND),
    lambda(data,se2_z_state::LAMBDA_IND),
    actual(x,y,z,theta,v,vz,w),
    desired(xd,yd,zd)
  {
    lambda = 0.1;
  }
  
    
  se2_z_state(const se2_z_state& state) : 
    TrajectoryState(state),
    x(data,se2_z_state::X_IND),
    y(data,se2_z_state::Y_IND),
    z(data,se2_z_state::Z_IND),
    theta(data,se2_z_state::THETA_IND),
    v(data,se2_z_state::V_IND),
    vz(data,se2_z_state::VZ_IND),
    w(data,se2_z_state::W_IND),
    xd(data,se2_z_state::XD_IND),
    yd(data,se2_z_state::YD_IND),
    zd(data,se2_z_state::ZD_IND),
    lambda(data,se2_z_state::LAMBDA_IND),
    actual(x,y,z,theta,v,vz,w),
    desired(xd,yd,zd)
  {
  }
  
  template <typename T>
  se2_z_state(const T& state) : 
    TrajectoryState(),
    x(data,se2_z_state::X_IND),
    y(data,se2_z_state::Y_IND),
    z(data,se2_z_state::Z_IND),
    theta(data,se2_z_state::THETA_IND),
    v(data,se2_z_state::V_IND),
    vz(data,se2_z_state::VZ_IND),
    w(data,se2_z_state::W_IND),
    xd(data,se2_z_state::XD_IND),
    yd(data,se2_z_state::YD_IND),
    zd(data,se2_z_state::ZD_IND),
    lambda(data,se2_z_state::LAMBDA_IND),
    actual(x,y,z,theta,v,vz,w),
    desired(xd,yd,zd)
  {
    lambda = 0.1;
    from(state);
  }
  
  void operator=(const se2_z_state& state)
  {
    data = state.data;
  }
  
  bool isValid() const
  {
    return lambda > 0;
  }
  
  template<typename T>
  void from(const T& p)
  {
    actual.from(p);
  }
  
  template<typename T>
  void to(T& p) const
  {
    actual.to(p);
  }
  
  template<typename T>
  void from(const trajectory_generator::Desired<T>& p)
  {
    const T& tp = p;
    desired.from(tp);
  }
  
  template<typename T>
  void to(trajectory_generator::Desired<T>& p) const
  {
    T& tp = p;
    desired.to(tp);
  }
    
  trajectory_msgs::MultiDOFJointTrajectoryPoint toMsg() const
  {
    trajectory_msgs::MultiDOFJointTrajectoryPoint point;
    
    point.transforms.emplace_back();
    actual.to(point.transforms.back());
    
    point.velocities.emplace_back();
    actual.to(point.velocities.back());
    
    //transformTwist(point.velocities.back(), point.velocities.back(), point.transforms.back());
    
    return point;
  }
  
  
};
  
typedef se2_z_state state_type;



  } //namespace se2_z_trajectory

} //namespace nav_quadrotor

#endif  /* QUADROTOR_TRAJECTORY_GENERATOR_se2_z_STATE_H */

