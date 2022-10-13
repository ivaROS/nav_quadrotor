#ifndef NAV_QUADROTOR_TRAJECTORY_FUNCTIONS_H
#define NAV_QUADROTOR_TRAJECTORY_FUNCTIONS_H

#include <nav_quadrotor/ros_interface_se2_z.h>

namespace nav_quadrotor
{

  namespace se2_z_trajectory
  {
    
    class arc_traj_func : public desired_traj_func
    {
      double vf, vz, w;
      
    public:
      arc_traj_func( double vf, double vz, double w) : vf(vf), vz(vz), w(w) { }
      
      void dState ( const se2_z_state &x , se2_z_state::Desired &d_desx_dt , const double  t  )
      {
        d_desx_dt.x = vf*sin( - (w) * t );
        d_desx_dt.y = vf*cos( - (w) * t );
        d_desx_dt.z = vz;
      }
      
      using Ptr=std::shared_ptr<arc_traj_func>;
    };
    
    class straight_traj_func : public desired_traj_func
    {
      double vf, vz, theta;
      
    public:
      straight_traj_func( double vf, double vz, double theta) : vf(vf), vz(vz), theta(theta) { }
      
      void dState ( const se2_z_state &x , se2_z_state::Desired &d_desx_dt , const double  t  )
      {
        d_desx_dt.x = vf*cos(theta);
        d_desx_dt.y = vf*sin(theta);
        d_desx_dt.z = vz;
      }
      
      using Ptr=std::shared_ptr<straight_traj_func>;
    };
  } //end namespace se2_z_trajectory

} //end namespace nav_quadrotor

#endif //NAV_QUADROTOR_TRAJECTORY_FUNCTIONS_H
