#ifndef NAV_QUADROTOR_GLOBAL_IMPL_H
#define NAV_QUADROTOR_GLOBAL_IMPL_H

#include <nav_quadrotor/quadrotor_nav_impl.h>
#include <pips_egocylindrical/egocylindrical_image_cc_wrapper.h>
#include <trajectory_based_nav/global_goal_nav.h>
#include <nav_quadrotor/global_goal_scoring.h>
#include <nav_quadrotor/quadrotor_controller_interface.h>
#include <nav_quadrotor/interfaces.h>
//#include <nav_quadrotor/general_nav_impl.h>
//#include <nav_quadrotor/goal_based_trajectories.h>
//#include <nav_quadrotor/new_trajectory_sources.h>
#include <nav_quadrotor/multi_level_trajectory_verifier.h>
#include <pips_egocylindrical/egocan_cc_wrapper.h>
#include <trajectory_based_nav/global_potentials_planner_interface.h>
#include <nav_quadrotor/new_trajectory_sources.h>

#include <memory>

namespace nav_quadrotor
{
  
  class GlobalNavPlanner : public NavImpl
  {
  public:
    //AngledStraightGoalTrajectorySource::Ptr trajectory_source_;
    //AngledStraightTrajectorySource::Ptr trajectory_source_;
    //GlobalGoalTrajectorySource::Ptr trajectory_source_;
    //FixedZRadiallySampledTrajectorySource::Ptr trajectory_source_;
    TrajectorySource::Ptr trajectory_source_;
    TrajectoryObjectTrajectorySource::Ptr trajectory_source1_, trajectory_source2_, trajectory_source3_, trajectory_source4_;
    
    trajectory_based_nav::MultiLevelTrajectoryVerifier::Ptr planning_trajectory_verifier_, online_trajectory_verifier_;
    trajectory_based_nav::BasicReplanLogic::Ptr replan_logic_;
    trajectory_based_nav::GlobalGoalTerminationCriteria::Ptr termination_criteria_;
    nav_quadrotor::GlobalGoalTrajectoryScoring::Ptr trajectory_scoring_;
    QuadrotorControllerInterface::Ptr trajectory_controller_;
    //std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> cc_wrapper_; //put this back later
    std::shared_ptr<trajectory_based_nav::GlobalGoalState> global_goal_state_;
    std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> cons_cc_wrapper_, lib_cc_wrapper_; //just temporary
    std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> egocan_cc_wrapper1_, egocan_cc_wrapper2_;
    
    using GlobalPlannerImpl = trajectory_based_nav::GlobalPotentialsPlannerInterface;
    GlobalPlannerImpl::Ptr global_planner_impl_;
    //using GlobalPlanner = trajectory_based_nav::TypedGlobalPlanningInterface<GlobalPlannerImpl::planner_result_t>;
    using GlobalPlanner = trajectory_based_nav::TypedGlobalPlanningInterface<GlobalPlannerImpl>;
    GlobalPlanner::Ptr global_planner_;
    
  public:
    
    GlobalNavPlanner();
    
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm=tf2_utils::TransformManager(true));
    
    virtual bool update(const nav_msgs::Odometry& odom);
    
    virtual void enablePlanning();
    
    virtual void disablePlanning();
    
    virtual TrajectorySource::Ptr getTrajectorySource()
    {
      return trajectory_source_;
    }
    
    virtual trajectory_based_nav::ReplanLogic::Ptr getReplanLogic()
    {
      return replan_logic_;
    }
    
    virtual trajectory_based_nav::TrajectoryScoring::Ptr getTrajectoryScoring()
    {
      return trajectory_scoring_;
    }
    
    virtual trajectory_based_nav::TerminationCriteria::Ptr getTerminationCriteria()
    {
      return termination_criteria_;
    }
    
    virtual trajectory_based_nav::TrajectoryVerifier::Ptr getTrajectoryMonitor()
    {
      return online_trajectory_verifier_;
    }
    
    virtual trajectory_based_nav::TrajectoryVerifier::Ptr getTrajectoryVerifier()
    {
      return planning_trajectory_verifier_;
    }
    
    virtual TrajectoryController::Ptr getTrajectoryController()
    {
      return trajectory_controller_;
    }
  };
  
  
} //end namespace nav_quadrotor

#endif //NAV_QUADROTOR_GLOBAL_IMPL_H
