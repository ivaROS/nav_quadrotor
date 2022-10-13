#ifndef NAV_QUADROTOR_INTERFACES_H
#define NAV_QUADROTOR_INTERFACES_H

#include <nav_quadrotor/core.h>
#include <trajectory_based_nav/interfaces.h>

namespace nav_quadrotor
{

  using TrajectoryController = trajectory_based_nav::TypedTrajectoryController<msg_type>;
  
  using TrajectorySource = trajectory_based_nav::TrajectorySource<msg_type>;
  
  using TrajectoryWrapper = trajectory_based_nav::TypedTrajectoryWrapper<msg_type>;

} //end namespace nav_quadrotor

#endif  //NAV_QUADROTOR_INTERFACES_H
