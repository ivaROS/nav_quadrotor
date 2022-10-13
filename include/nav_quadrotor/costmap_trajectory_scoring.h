#ifndef NAV_QUADROTOR_COSTMAP_SCORING_H
#define NAV_QUADROTOR_COSTMAP_SCORING_H

#include <nav_quadrotor/global_goal_scoring.h>
#include <nav_quadrotor/committed_global_goal_scoring.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <trajectory_based_nav/default_implementations.h>  //Only need BasicTrajectoryCosts
#include <string> 


namespace nav_quadrotor
{
  
  class CostmapTrajectoryScoring : public GlobalGoalTrajectoryScoring
  {
  protected:
    trajectory_based_nav::GlobalGoalState::Ptr ggs_;
    std::shared_ptr<costmap_2d::Costmap2DROS> costmap_;
    
  public:
    CostmapTrajectoryScoring(GlobalPlanner::Ptr global_planner, const std::vector<std::shared_ptr<pips_egocylindrical::FreeSpaceChecker> >& fscs, trajectory_based_nav::GlobalGoalState::Ptr ggs);

    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm) override;
    
    virtual bool scoreTrajectory(trajectory_based_nav::TrajectoryWrapper::Ptr traj) override;
  
    virtual double getObstacleCost(const geometry_msgs::Pose& pose) override;
    
    virtual double getMinDistance(const geometry_msgs::Pose& pose) override;

  };

} //end namespace nav_quadrotor

#endif //NAV_QUADROTOR_COSTMAP_SCORING_H

