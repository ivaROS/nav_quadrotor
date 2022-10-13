#include <nav_quadrotor/global_impl.h>
#include <nav_quadrotor/general_nav_impl.h>
#include <trajectory_based_nav/general_nav_impl_behavior.h>
#include <trajectory_based_nav/behavior_manager.h>
#include <trajectory_based_nav/global_goal_manager.h>

int main(int argc, char** argv) 
{
  using trajectory_based_nav::BehaviorRequest;
  using trajectory_based_nav::TestBehavior;
  
  ros::init(argc, argv, "quadrotor_global_node");
  
  //ros::master::setRetryTimeout(ros::WallDuration(2));
  
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  trajectory_based_nav::BehaviorManager m("manager");
  
  
  auto idle_behavior = std::make_shared<TestBehavior>("IDLE");
  
  auto planner = std::make_shared<nav_quadrotor::GlobalNavPlanner>();
  
  auto navigating_behavior = trajectory_based_nav::MakeGeneralNavImplBehavior("NAVIGATING", nh, pnh, planner);
  
  auto ggs_ptr = planner->global_goal_state_;
  auto controller_ptr = planner->getTrajectoryController();
  
  trajectory_based_nav::GlobalGoalManager ggm(ggs_ptr, controller_ptr);
  ggm.init(nh, pnh);
  
  ggm.addSuccessCallback([&m, &idle_behavior, &navigating_behavior]()
  { 
    m.addRequest(BehaviorRequest(navigating_behavior, false));
    m.addRequest(BehaviorRequest(idle_behavior, true));
  });
  
  ggs_ptr->getReadyGoalSource().registerCallback(
    [&m, &idle_behavior, &navigating_behavior](const geometry_msgs::PoseStamped::ConstPtr& goal) 
    { 
      m.addRequest(BehaviorRequest(idle_behavior, false));
      m.addRequest(BehaviorRequest(navigating_behavior, true));
    });
  
  ros::MultiThreadedSpinner spinner(8);
  spinner.spin();
  
  return 0;
}
/*

{
  using trajectory_based_nav::BehaviorRequest;
  using trajectory_based_nav::TestBehavior;
  using trajectory_based_nav::BehaviorTransition;
  using trajectory_based_nav::NavigationBehaviorLogic;
  
  trajectory_based_nav::BehaviorManager m("manager");
  
  
  auto idle_behavior = std::make_shared<TestBehavior>("IDLE");
  
  auto planner = std::make_shared<nav_quadrotor::GlobalNavPlanner>();
  
  auto navigating_behavior = trajectory_based_nav::MakeGeneralNavImplBehavior("NAVIGATING", nh, pnh, planner);

  trajectory_based_nav::GlobalGoalManager ggm(navigating->planner_->global_goal_state_);
  ggm.init(nh, pnh);

  ggm.addSuccessCallback([&m, &idle_behavior]()
    { 
      m.addRequest(BehaviorRequest(navigating_behavior, false));
      m.addRequest(BehaviorRequest(idle_behavior, true));
    });
  
  navigating->planner_->global_goal_state_->getReadyGoalSource().registerCallback(
    [&m, &navigating_behavior](const geometry_msgs::PoseStamped::ConstPtr& goal) 
    { 
      m.addRequest(BehaviorRequest(idle_behavior, false));
      m.addRequest(BehaviorRequest(navigating_behavior, true));
      
    });

}*/
