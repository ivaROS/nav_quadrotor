#ifndef NAV_QUADROTOR_GOAL_BASED_TRAJECTORIES_H
#define NAV_QUADROTOR_GOAL_BASED_TRAJECTORIES_H

#include <nav_quadrotor/basic_trajectory_source.h>
#include <trajectory_based_nav/global_goal_nav.h>
#include <angles/angles.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace nav_quadrotor
{
  bool getRelativePoseDirection(const geometry_msgs::Point& pos, const geometry_msgs::Point& dpos, double& dist, double& angle, double& dz);
  
  class GoalBasedTrajectorySource : public BasicGenerationTrajectorySource
  {
  protected:
    std::shared_ptr<trajectory_based_nav::GlobalGoalState> ggs_;
    
  public:
    
    GoalBasedTrajectorySource(std::shared_ptr<trajectory_based_nav::GlobalGoalState> ggs);
    
    bool getLocalGoal(const nav_msgs::Odometry& odom, double& dist, double& angle, double& dz);
    
    using Ptr=std::shared_ptr<GoalBasedTrajectorySource>;
  };
  
  

  
  
  class AngledStraightGoalTrajectorySource : public GoalBasedTrajectorySource
  {
    double max_traj_duration;
  public:
    
    AngledStraightGoalTrajectorySource(std::shared_ptr<trajectory_based_nav::GlobalGoalState> ggs);
      
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm);
    
    virtual bool update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory);
    
    using Ptr=std::shared_ptr<AngledStraightGoalTrajectorySource>;
  };
  
  
  class AngledStraightGoalsTrajectorySource : public GoalBasedTrajectorySource
  {
    message_filters::Subscriber<geometry_msgs::PoseArray> point_sub_;
    std::string fixed_frame_id_;
    double max_traj_duration_;
    double max_angle_;
    
    ros::Publisher transformed_poses_pub_;
    tf2_utils::TransformManager tfm_;
    
    using tf_filter = tf2_ros::MessageFilter<geometry_msgs::PoseArray>;
    std::shared_ptr<tf_filter> point_filter_;
    std::shared_ptr<message_filters::Cache<geometry_msgs::PoseArray> > point_cache_;
    
  public:
    
    AngledStraightGoalsTrajectorySource(std::shared_ptr<trajectory_based_nav::GlobalGoalState> ggs);
    
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm);
    
    virtual bool update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory);
    
    using Ptr=std::shared_ptr<AngledStraightGoalsTrajectorySource>;
  };
  
  
  
  class AngledStraightGoalsFutureTrajectorySource : public AngledStraightGoalsTrajectorySource
  {
    ros::Duration generation_delay_, bias_delay_;
    
  public:
    
    AngledStraightGoalsFutureTrajectorySource(std::shared_ptr<trajectory_based_nav::GlobalGoalState> ggs);
    
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm);
    
    virtual bool update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory);
    
    using Ptr=std::shared_ptr<AngledStraightGoalsTrajectorySource>;
  };
  

} //end namespace nav_quadrotor

#endif //NAV_QUADROTOR_GOAL_BASED_TRAJECTORIES_H
  
