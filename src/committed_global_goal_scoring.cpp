#include <nav_quadrotor/committed_global_goal_scoring.h>
#include <nav_quadrotor/basic_trajectory_wrapper.h>
#include <pips/utils/param_utils.h>

namespace nav_quadrotor
{
  
  double getDistance(const geometry_msgs::Point& a, const geometry_msgs::Point& b)
  {
    auto dx = a.x - b.x;
    auto dy = a.y - b.y;
    auto dz = a.z - b.z;
    auto dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    return dist;
  }
  
  
  CommittedGlobalGoalTrajectoryScoring::CommittedGlobalGoalTrajectoryScoring(GlobalPlanner::Ptr global_planner, const std::vector<std::shared_ptr<pips_egocylindrical::FreeSpaceChecker> >& fscs, trajectory_based_nav::GlobalGoalState::Ptr ggs):
      GlobalGoalTrajectoryScoring(global_planner, fscs),
      ggs_(ggs)
    {}
    
  bool CommittedGlobalGoalTrajectoryScoring::init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
  {
    bool res = GlobalGoalTrajectoryScoring::init(nh, pnh, tfm);
    
    pips::utils::get_param(pnh, "current_traj_cost_factor", current_traj_cost_factor_, 0.8, 5);
    
    return res;
  }
  
  bool CommittedGlobalGoalTrajectoryScoring::scoreTrajectory(trajectory_based_nav::TrajectoryWrapper::Ptr traj)
  {
    auto pose_array = traj->getPoseArray();
    
    auto btraj = std::dynamic_pointer_cast<BasicTrajectoryWrapper>(traj);

    if(pose_array.poses.size() > 0)
    {
      auto costs = std::make_shared<trajectory_based_nav::TrajectoryCostsContainer>();
      
      double dist=FATAL_COST, heightcost=FATAL_COST;
      for(const auto& pose : pose_array.poses)
      {
        dist = getGoalDist(pose);
        heightcost = getHeightCost(pose);
        if(dist + heightcost == FATAL_COST)
        {
          ROS_DEBUG_STREAM_NAMED("trajectory_scoring.fatal_cost", "Cost is fatal");
          break;
        }
      }
      
      costs->addCost(dist, "dist");
      costs->addCost(heightcost, "height");
      
      if(btraj)
      {
        auto source = btraj->source_;
        ROS_DEBUG_STREAM_NAMED("trajectory_scoring.source", "Trajectory source=" << source);
        
        if(source == "current")
        {
          auto old_cost = costs->single_cost;
          costs->single_cost -= current_traj_cost_factor_;
          ROS_DEBUG_STREAM_NAMED("trajectory_scoring.current_trajectory", "Reducing current trajectory cost from [" << old_cost << "] to [" << costs->single_cost << "]");
        }
        
        if(source == "local_goal" || source == "current")
        {
          auto goal = ggs_->getGoal();
          {
            auto dist = getDistance(goal.position, pose_array.poses.back().position);
            ROS_DEBUG_STREAM_NAMED("trajectory_scoring.global_goal", "Endpose [" << pose_array.poses.back() << "] has distance [" << dist << "] from goal");
            if(dist <= 1)
            {
              costs->addCost(-1, "global_goal");
            }
          }
        }
        
      }
      else
      {
        ROS_ERROR("Couldn't cast trajectory to BasicTrajectoryWrapper!");
      }
        
      ROS_DEBUG_STREAM_NAMED("trajectory_scoring", "cost=" << costs->cost());
      
      ROS_DEBUG_STREAM_NAMED("trajectory_scoring.detailed_costs", "traj" << *costs);
              
      traj->costs = costs;
      
      return true;
    }
    else
    {
      ROS_WARN("Pose Array contains 0 poses!");
      return false;
    }
  }
} //end namespace nav_quadrotor
