#include <nav_quadrotor/global_impl.h>
#include <pips_egocylindrical/egocylindrical_image_cc_wrapper.h>
#include <trajectory_based_nav/global_goal_nav.h>
#include <nav_quadrotor/global_goal_scoring.h>
#include <nav_quadrotor/quadrotor_controller_interface.h>
#include <nav_quadrotor/quadrotor_nav_impl.h>
#include <nav_quadrotor/goal_based_trajectories.h>
#include <nav_quadrotor/new_trajectory_sources.h>
#include <nav_quadrotor/end_point_trajectory_source.h>
#include <nav_quadrotor/multi_level_trajectory_verifier.h>
#include <pips_egocylindrical/egocan_cc_wrapper.h>
#include <memory>
#include <nav_quadrotor/committed_global_goal_scoring.h>
#include <nav_quadrotor/global_goal_replan_logic.h>
#include <nav_quadrotor/detailed_waypoint_trajectory_source.h>

namespace nav_quadrotor
{
  
    GlobalNavPlanner::GlobalNavPlanner()
    {
      global_goal_state_ = std::make_shared<trajectory_based_nav::GlobalGoalState>();
      auto trajectory_source = std::make_shared<CompoundTrajectorySource>();
      trajectory_source1_ = std::make_shared<EndPointTrajectorySource>();
      trajectory_source1_->name_ = "local_goal";

      trajectory_source_ = trajectory_source;
      
      planning_trajectory_verifier_ = std::make_shared<trajectory_based_nav::MultiLevelTrajectoryVerifier>("planning_trajectory_verifier");
      online_trajectory_verifier_ = std::make_shared<trajectory_based_nav::MultiLevelTrajectoryVerifier>( "online_trajectory_verifier");
      replan_logic_ = //std::make_shared<trajectory_based_nav::GlobalGoalReplanLogic>(online_trajectory_verifier_, global_goal_state_, "replan_logic");
      std::make_shared<nav_quadrotor::GlobalGoalReplanLogic>(online_trajectory_verifier_, global_goal_state_, "replan_logic");
      termination_criteria_ = std::make_shared<trajectory_based_nav::GlobalGoalTerminationCriteria>(global_goal_state_);
      
      global_planner_impl_ = std::make_shared<GlobalPlannerImpl>("potentials_planner"); 
      global_planner_ = std::make_shared<GlobalPlanner>(global_goal_state_, global_planner_impl_, "async_planner_interface");
      

      trajectory_source2_ = std::make_shared<DetailedWaypointTrajectorySource>(planning_trajectory_verifier_);
      trajectory_source2_->name_ = "end_point";
      
      trajectory_source->addSource(trajectory_source1_);
      trajectory_source->addSource(trajectory_source2_);

      ROS_DEBUG_STREAM_NAMED("global_nav_planner", "Finished constructing global_nav_planner!");
    }
    
    bool GlobalNavPlanner::init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
    {
      ROS_INFO_STREAM("Initializing global_nav_planner!");
      global_goal_state_->init(nh, pnh, tfm);
      cc_wrapper_ = std::make_shared<pips_egocylindrical::EgocylindricalRangeImageCCWrapper>(nh, pnh, tfm);

      cons_cc_wrapper_ = std::make_shared<pips_egocylindrical::EgocylindricalRangeImageCCWrapper>(nh, pnh, tfm, "conservative_inflated_egocylindrical_image_cc_wrapper");
      lib_cc_wrapper_ = std::make_shared<pips_egocylindrical::EgocylindricalRangeImageCCWrapper>(nh, pnh, tfm, "liberal_inflated_egocylindrical_image_cc_wrapper");
      egocan_cc_wrapper1_ = std::make_shared<pips_egocylindrical::EgoCanCCWrapper>(nh, pnh, tfm, "inflated_egocan_cc_wrapper1");
      egocan_cc_wrapper2_ = std::make_shared<pips_egocylindrical::EgoCanCCWrapper>(nh, pnh, tfm, "inflated_egocan_cc_wrapper2");
      
      planning_trajectory_verifier_->setCCWrappers(cons_cc_wrapper_, lib_cc_wrapper_, cc_wrapper_, egocan_cc_wrapper1_, egocan_cc_wrapper2_);
      online_trajectory_verifier_->setCCWrappers(cons_cc_wrapper_, lib_cc_wrapper_, cc_wrapper_, egocan_cc_wrapper1_, egocan_cc_wrapper2_);
      replan_logic_->init(nh, pnh, tfm);
      global_planner_->init(nh, pnh, tfm);
      
      auto get_free_space_checker = [](std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> wrapper)
      {
        auto cc = wrapper->getCC();
        auto fsc = std::dynamic_pointer_cast<pips_egocylindrical::FreeSpaceChecker>(cc);
        return fsc;
      };
      
      std::vector<std::shared_ptr<pips_egocylindrical::FreeSpaceChecker> > free_space_checkers;
      free_space_checkers.push_back(get_free_space_checker(egocan_cc_wrapper1_));
      free_space_checkers.push_back(get_free_space_checker(lib_cc_wrapper_));
      
      trajectory_scoring_ = std::make_shared<nav_quadrotor::CommittedGlobalGoalTrajectoryScoring>(global_planner_, free_space_checkers, global_goal_state_);

      
      bool status = true;
      status &= trajectory_scoring_->init(nh, pnh, tfm);
      status &= termination_criteria_->init(nh, pnh, tfm);
      
      status &= cc_wrapper_->init();
      status &= cons_cc_wrapper_->init();
      status &= lib_cc_wrapper_->init();
      status &= egocan_cc_wrapper1_->init();
      status &= egocan_cc_wrapper2_->init();
      
      trajectory_controller_ = std::make_shared<QuadrotorControllerInterface>(nh, pnh, tfm);
      
      status &= trajectory_source_->init(nh, pnh, tfm);
      auto pnh1 = ros::NodeHandle(pnh, "local_goal_trajectory_source");
      status &= trajectory_source1_->init(nh, pnh1, tfm);
      auto pnh2 = ros::NodeHandle(pnh, "end_point_trajectory_source");
      status &= trajectory_source2_->init(nh, pnh2, tfm);
      
      status &= planning_trajectory_verifier_->init(nh, pnh, tfm);
      ROS_DEBUG_STREAM_NAMED("global_nav_planner", "initializing20");
      status &= online_trajectory_verifier_->init(nh, pnh, tfm);
      ROS_DEBUG_STREAM_NAMED("global_nav_planner", "initializing21");
      
      ROS_DEBUG_STREAM_NAMED("global_nav_planner", "Finished initializing global_nav_planner!");
      
      return status;
    }
    
    bool GlobalNavPlanner::update(const nav_msgs::Odometry& odom) //Is this needed?
    {
      auto updateCC = [this, odom]()
      {
        std_msgs::Header header = cc_wrapper_->getCurrentHeader();
        
        if(!cc_wrapper_->isReady(odom.header))
        {
          ROS_WARN_STREAM("cc_wrapper is not ready!");
          return false;
        }
        cc_wrapper_->update();
        return true;
      };
      
      //to allow debugging when planning not running:
      {
        planning_trajectory_verifier_->update(odom);
        //trajectory_scoring_->update(odom); //currently redundant, just updates collision checkers
      }
      
      if(!global_goal_state_->hasGoal())
      {
        ROS_WARN_STREAM("No global goal received!");
        return false;
      }
      
      return global_goal_state_->hasGoal() &&
      global_goal_state_->updateGoalTransform(odom) &&
      planning_trajectory_verifier_->update(odom) &&
      //trajectory_source_->update(odom) &&
      trajectory_scoring_->update(odom) && updateCC();
    }
    
    void GlobalNavPlanner::enablePlanning()
    {
      ROS_INFO_STREAM("Enable planning!");
      cc_wrapper_->setCallback(planning_cb_);
    }
    
    void GlobalNavPlanner::disablePlanning()
    {
      ROS_INFO_STREAM("Disable planning!");
      cc_wrapper_->setCallback(0);
      //trajectory_source_->stop();
    }
  
} //end namespace nav_quadrotor

