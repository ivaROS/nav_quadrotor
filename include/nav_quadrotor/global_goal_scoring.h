#ifndef NAV_QUADROTOR_GLOBAL_GOAL_SCORING_H
#define NAV_QUADROTOR_GLOBAL_GOAL_SCORING_H

#include <pips_egocylindrical/free_space_checker.h>
#include <nav_quadrotor/core.h>
#include <global_planner_plus/global_potentials_interface.h>
//#include <boost/thread/mutex.hpp>
#include <pips_egocylindrical/FreeSpaceCheckerService.h>
#include <trajectory_based_nav/global_potentials_planner_interface.h>


namespace nav_quadrotor
{

class GlobalGoalTrajectoryScoring : public trajectory_based_nav::TrajectoryScoring
{
protected:
  std::shared_ptr<global_planner_plus::GlobalPotentialsInterface> gpi_;
  std::vector<std::shared_ptr<pips_egocylindrical::FreeSpaceChecker> > fscs_;
  double min_height_, max_height_, des_height_, height_weight_, max_height_cost_;
  double desired_freespace_, obstacle_scale_;
  ros::ServiceServer free_space_checking_service_;
  
  using GlobalPlannerImpl = trajectory_based_nav::GlobalPotentialsPlannerInterface;
  using GlobalPlanner = trajectory_based_nav::TypedGlobalPlanningInterface<GlobalPlannerImpl>;
  GlobalPlanner::Ptr global_planner_;
  const double FATAL_COST;
  
public:
  
  GlobalGoalTrajectoryScoring(GlobalPlanner::Ptr global_planner, const std::vector<std::shared_ptr<pips_egocylindrical::FreeSpaceChecker> >& fscs);
  
  //TODO: pass in freespace checkers through constructor or some other function
  virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm) override;
  
  virtual bool update(const nav_msgs::Odometry& odom) override;
  
protected:
  
  bool freeSpaceCheckingSrv(pips_egocylindrical::FreeSpaceCheckerService::Request &req, pips_egocylindrical::FreeSpaceCheckerService::Response &res);
  
  virtual double getGoalDist(const geometry_msgs::Pose& pose);
  
  virtual double getHeightCost(const geometry_msgs::Pose& pose);
  
  virtual double getMinDistance(const geometry_msgs::Pose& pose);
  
  virtual double getObstacleCost(const geometry_msgs::Pose& pose);
  
public:
  
  virtual bool scoreTrajectory(trajectory_based_nav::TrajectoryWrapper::Ptr traj);
  
  using Ptr=std::shared_ptr<GlobalGoalTrajectoryScoring>;
  
};


} //end namespace nav_quadrotor


#endif //NAV_QUADROTOR_GLOBAL_GOAL_SCORING_H
  
