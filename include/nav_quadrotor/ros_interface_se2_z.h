#ifndef NAV_QUADROTOR_TRAJECTORY_GENERATOR_ROS_INTERFACE_SE2_Z_H
#define NAV_QUADROTOR_TRAJECTORY_GENERATOR_ROS_INTERFACE_SE2_Z_H

#include <nav_quadrotor/se2_z_trajectory.h>
#include <trajectory_generator_ros_interface.h>

namespace nav_quadrotor
{
  namespace se2_z_trajectory 
  {
    
     typedef trajectory_generator::trajectory_states<state_type, traj_func_type > traj_states_type;
     typedef std::shared_ptr<traj_states_type > trajectory_ptr;
    
     typedef traj_states_type traj_type;
    typedef std::shared_ptr<traj_type> traj_type_ptr;
    
    inline
    traj_type_ptr getTrajectoryStates(const traj_func_type& traj_obj)
    {
      traj_type_ptr traj = std::make_shared<traj_type>(traj_obj.result.x_vec, traj_obj.result.times);
      traj->x0_ = traj_obj.x0;
      return traj;
    }
    
    using trajectory_msg_t = traj_type::trajectory_msg_t;
    
  }
  
}

namespace trajectory_generator
{

  template <>
  struct trajectory_msg_traits<nav_quadrotor::se2_z_trajectory::trajectory_msg_t>
  {
    static void setTrajectoryPointTime(trajectory_msgs::MultiDOFJointTrajectoryPoint& msg, ros::Duration time)
    {
      msg.time_from_start = time;
    }
  };
}


#endif //NAV_QUADROTOR_TRAJECTORY_GENERATOR_ROS_INTERFACE_SE2_Z_H

