#include <nav_quadrotor/global_goal_scoring.h>
#include <pips_egocylindrical/free_space_checker.h>
#include <nav_quadrotor/core.h>
#include <global_planner_plus/global_potentials_interface.h>
//#include <boost/thread/mutex.hpp>
#include <pips_egocylindrical/FreeSpaceCheckerService.h>
#include <trajectory_based_nav/global_potentials_planner_interface.h>
#include <limits>

namespace nav_quadrotor
{
  
  GlobalGoalTrajectoryScoring::GlobalGoalTrajectoryScoring(GlobalPlanner::Ptr global_planner, const std::vector<std::shared_ptr<pips_egocylindrical::FreeSpaceChecker> >& fscs):
    fscs_(fscs),
    global_planner_(global_planner),
    FATAL_COST(std::numeric_limits<double>::max())
  {

  }
  
  
  bool GlobalGoalTrajectoryScoring::init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
  {
    TrajectoryScoring::init(nh, pnh, tfm);
    
    std::string potentials_topic_ = "/raw_potential";
    
    gpi_ = std::make_shared<global_planner_plus::GlobalPotentialsInterface>(nh);
    gpi_->init();
    
    auto my_pnh = ros::NodeHandle(pnh, "global_goal_trajectory_scoring");
    
    auto height_pnh = ros::NodeHandle(my_pnh, "height_cost");
    
    min_height_ = 0.5;
    max_height_ = 2.5;
    des_height_ = 1.25;
    height_weight_ = 1;
    max_height_cost_ = 1;
    
    height_pnh.getParam("min_height", min_height_);
    height_pnh.setParam("min_height", min_height_);
    
    height_pnh.getParam("max_height", max_height_);
    height_pnh.setParam("max_height", max_height_);
    
    height_pnh.getParam("des_height", des_height_);
    height_pnh.setParam("des_height", des_height_);
    
    height_pnh.getParam("weight", height_weight_);
    height_pnh.setParam("weight", height_weight_);
    
    height_pnh.getParam("max_cost", max_height_cost_);
    height_pnh.setParam("max_cost", max_height_cost_);
    
    auto obstacle_pnh = ros::NodeHandle(my_pnh, "obstacle_cost");
    
    //search_radius_ = 0.5;
    desired_freespace_ = 0.5;
    obstacle_scale_ = 2;
    
    obstacle_pnh.getParam("desired_freespace", desired_freespace_);
    obstacle_pnh.setParam("desired_freespace", desired_freespace_);
    
    obstacle_pnh.getParam("weight", obstacle_scale_);
    obstacle_pnh.setParam("weight", obstacle_scale_);
    
    free_space_checking_service_ = obstacle_pnh.advertiseService("free_space_checker", &GlobalGoalTrajectoryScoring::freeSpaceCheckingSrv, this);
    
    return true;
  }
  

  bool GlobalGoalTrajectoryScoring::update(const nav_msgs::Odometry& odom)
  {
    bool res = true;
    
    const auto plan = global_planner_->getCurrentPlan();
    
    if(plan==nullptr)
    {
      ROS_WARN("No global plan available!");
      global_planner_->triggerPlanning();
      res = false;
    }
    else if(!plan->is_valid)
    {
      ROS_WARN("Global plan is not valid!");
      global_planner_->triggerPlanning();
      res = false;
    }
    else
    {
      gpi_->setPotentials(plan->potentials);
    }
    
    
    //TODO: get actual transform between odom frame and frame of potentials, rather than assuming it is always identity
    tf::StampedTransform transform;
    transform.setIdentity();
    if(gpi_->updateTransform(transform))
    {
      ROS_INFO_ONCE("Potentials ready!");
    }
    else
    {
      res = false;
    }
    return res;
  }
  

  bool GlobalGoalTrajectoryScoring::freeSpaceCheckingSrv(pips_egocylindrical::FreeSpaceCheckerService::Request &req, pips_egocylindrical::FreeSpaceCheckerService::Response &res)
  {
    res.distance = getMinDistance(req.pose.pose);
    return true;
  }
  

  double GlobalGoalTrajectoryScoring::getGoalDist(const geometry_msgs::Pose& pose)
  {
    
    float pot = gpi_->getPotential(pose);
    ROS_DEBUG_STREAM_NAMED("trajectory_scoring.cost_components.goal_dist", "Potential at [" << pose.position.x << "," << pose.position.y << "]: " << pot);
    if(pot<0 || pot >= 1e9) //NOTE: Previously tested == POT_HIGH, uncertain if this change is necessary
    {
      ROS_DEBUG_STREAM_NAMED("trajectory_scoring.cost_components.goal_dist", "Encountered invalid potential at [" << pose.position.x << "," << pose.position.y << "]: " << pot);
      return FATAL_COST;
    }
    return pot;
  }

  
  double GlobalGoalTrajectoryScoring::getHeightCost(const geometry_msgs::Pose& pose)
  {
    if(height_weight_==0)
    {
      return 0;
    }
    
    double height = pose.position.z;
    double height_cost;
    if(height <= min_height_)
    {
      height_cost = FATAL_COST;
      ROS_DEBUG_STREAM_NAMED("trajectory_scoring.cost_components.height", "Too low!");
    }
    else if(height >= max_height_)
    {
      height_cost = FATAL_COST;
      ROS_DEBUG_STREAM_NAMED("trajectory_scoring.cost_components.height", "Too high!");
    }
    else
    {
      height_cost = std::abs(height - des_height_);
      height_cost = height_weight_ * height_cost;
      if(max_height_cost_>0)
      {
        height_cost = std::min(max_height_cost_, height_cost);
      }
    }
    return height_cost;
  }

  
  double GlobalGoalTrajectoryScoring::getMinDistance(const geometry_msgs::Pose& pose)
  {
    float min_depth = std::numeric_limits<float>::max();
    for(const auto fsc : fscs_)
    {
      float free_space = fsc->getFreeSpaceRadius(pose);
      min_depth = std::min(min_depth, free_space);
    }
    return min_depth;
  }
  

  double GlobalGoalTrajectoryScoring::getObstacleCost(const geometry_msgs::Pose& pose)
  {
    if(obstacle_scale_ > 0)
    {
      float min_depth = getMinDistance(pose);
      
      double cost=0;
      if(min_depth < desired_freespace_)
      {
        cost = desired_freespace_ - min_depth;
      }
      
      cost *= obstacle_scale_;
      
      ROS_DEBUG_STREAM_NAMED("trajectory_scoring.cost_components.obstacle_cost", "Min_depth=" << min_depth << ", desired_freespace=" << desired_freespace_ << ", cost=" << cost);
      
      return cost;
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED("trajectory_scoring.cost_components.obstacle_cost", "obstacle_scale=" << obstacle_scale_);
      return 0;
    }
    
  }
  

  bool GlobalGoalTrajectoryScoring::scoreTrajectory(trajectory_based_nav::TrajectoryWrapper::Ptr traj)
  {
    auto pose_array = traj->getPoseArray();
    
    if(pose_array.poses.size() > 0)
    {
      float best_cost = FATAL_COST;
      unsigned int best_ind=0;
      float max_obstacle_cost = 0;
      
      for(unsigned int ind=0; ind < pose_array.poses.size(); ++ind)
      {
        const auto& pose = pose_array.poses[ind];
        float dist = getGoalDist(pose);
        float heightcost = getHeightCost(pose);
        float obstcost = getObstacleCost(pose);
        
        max_obstacle_cost = std::max(obstcost, max_obstacle_cost);
        
        float cost = dist + heightcost + max_obstacle_cost;
        
        if(cost >= FATAL_COST)
        {
          ROS_DEBUG_STREAM_NAMED("trajectory_scoring.fatal_cost", "Cost is too high, stop analyzing trajectory here (at index " << ind << ")");
          
          break;
        }
        
        if(cost < best_cost)
        {
          best_cost = cost;
          best_ind = ind;
        }
      }
      
      ROS_DEBUG_STREAM_NAMED("trajectory_scoring.scoring", "best_ind=" << best_ind);
      
      auto costs = std::make_shared<trajectory_based_nav::BasicTrajectoryCosts>();
      costs->single_cost = best_cost;
      
      traj->costs = costs;
      traj->trim(best_ind+1);
      
      return true;
    }
    else
    {
      ROS_WARN("Pose Array contains 0 poses!");
      return false;
    }
  }

} //end namespace trajectory_based_nav

  
