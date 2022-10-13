#ifndef SE2_Z_BENT_STRAIGHT_LINE_TRAJECTORY_FUNCTION_H
#define SE2_Z_BENT_STRAIGHT_LINE_TRAJECTORY_FUNCTION_H

#include <nav_quadrotor/ros_interface_se2_z.h>

namespace nav_quadrotor
{

  namespace se2_z_trajectory
  {
    
    class bent_straight_traj_func : public desired_traj_func
    {
      double vf, vz, theta, switchtime;
      
    public:
      bent_straight_traj_func( double vf, double vz, double theta, double switchtime) : vf(vf), vz(vz), theta(theta), switchtime(switchtime) { }
      
      void dState ( const se2_z_state &x , se2_z_state::Desired &d_desx_dt , const double  t  )
      {
        d_desx_dt.x = vf*cos(theta);
        d_desx_dt.y = vf*sin(theta);
        d_desx_dt.z = (t < switchtime) ? vz : 0;
      }
      
      using Ptr=std::shared_ptr<bent_straight_traj_func>;
    };
    
    inline
    se2_z_trajectory::bent_straight_traj_func bentStraightLineRelativeGoal(double fw_vel, double dist, double angle, double dz, double target_z_factor, double& max_tf)
    {      
      double tf = dist/fw_vel;
      
      max_tf = std::min(tf, max_tf);
      
      double vz = dz/(target_z_factor*tf);
      
      double switchtime = tf*target_z_factor;
      
      auto dtraj = bent_straight_traj_func(fw_vel, vz, angle, switchtime);
      return dtraj;
    }
    
    inline
    void getPoseDirection(const geometry_msgs::Point& pos, const geometry_msgs::Point& dpos, double& dist, double& angle, double& dz)
    {    
      double dx = dpos.x - pos.x;
      double dy = dpos.y - pos.y;
      dz = dpos.z - pos.z;
      
      dist = std::sqrt(dx*dx+dy*dy);//TODO: Maybe include dz?
      
      //double yaw = trajectory_based_nav::getYaw(odom.pose.pose.orientation);
      
      angle = std::atan2(dy,dx);
      
      ROS_DEBUG_STREAM_NAMED("getRelativePoseDirection2", "pos= " << pos << ", dpos= " << dpos << ", dist=" << dist << ", angle=" << angle << ", dz=" << dz);
    }
    
    inline
    se2_z_trajectory::bent_straight_traj_func bentStraightLineGlobalGoal(const se2_z_trajectory::state_type& x0, const geometry_msgs::Point& dpos, double fw_vel, double target_z_factor, double& max_tf)
    {
      geometry_msgs::Point pos;
      x0.actual.to(pos);
      
      double dist, angle, dz;
      getPoseDirection(pos, dpos, dist, angle, dz);
          
      //double relative_angle = angles::normalize_angle(angle - x0.theta);
      
      return bentStraightLineRelativeGoal(fw_vel, dist, angle, dz, target_z_factor, max_tf);
    }
    
    inline
    se2_z_trajectory::SpecializedTrajGenObject<near_identity,bent_straight_traj_func> getBentStraightLineTrajObject(const se2_z_trajectory::state_type& x0, const geometry_msgs::Point& target_point, double fw_vel, double max_tf, double target_z_factor, const se2_z_trajectory::Params& ni_params)
    {
      auto dtraj = bentStraightLineGlobalGoal(x0, target_point, fw_vel, target_z_factor, max_tf);
      
      auto trajparams = traj_params();
      trajparams.tf = max_tf;
      auto trajobj = getTrajGenObject(dtraj, ni_params, x0, trajparams);
      return trajobj;
    }
  } //end namespace se2_z_trajectory

} //end namespace nav_quadrotor

#endif //SE2_Z_BENT_STRAIGHT_LINE_TRAJECTORY_FUNCTION_H
