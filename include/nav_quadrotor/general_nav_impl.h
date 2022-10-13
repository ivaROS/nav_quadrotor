#ifndef NAV_QUADROTOR_GENERAL_NAV_IMPL_H
#define NAV_QUADROTOR_GENERAL_NAV_IMPL_H

#include <nav_quadrotor/core.h>
#include <trajectory_based_nav/general_nav_impl.h>

namespace nav_quadrotor
{

  using NavSystem = trajectory_based_nav::GeneralNavImpl<msg_type>;
  
}

namespace trajectory_based_nav
{
    
  extern template class trajectory_based_nav::GeneralNavImpl<nav_quadrotor::msg_type>;
  
}

#endif //NAV_QUADROTOR_QUADROTOR_NAV_IMPL_H
