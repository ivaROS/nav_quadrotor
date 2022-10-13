#ifndef NAV_QUADROTOR_BASIC_TRAJECTORY_SOURCE_H
#define NAV_QUADROTOR_BASIC_TRAJECTORY_SOURCE_H

#include <nav_quadrotor/interfaces.h>
#include <nav_quadrotor/basic_trajectory_wrapper.h>
#include <nav_quadrotor/ni_config_utility.h>
#include <nav_quadrotor/trajectory_functions.h>


namespace nav_quadrotor
{

  class BasicGenerationTrajectorySource : public TrajectorySource
  {
  protected:
    int traj_ind_;
    std::vector<BasicTrajectoryWrapper::Ptr> trajectories_;
    se2_z_trajectory::NIConfigUtility ni_config_;
    
  public:
    
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm);
    
    virtual se2_z_trajectory::Params getNIParams();
    
    virtual bool update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory)=0;
    
    virtual bool hasMore() const;
    
    virtual TrajectoryWrapper::Ptr getNext();
    
    using Ptr=std::shared_ptr<BasicGenerationTrajectorySource>;
  };
  
  

  class ArcTrajectorySource : public BasicGenerationTrajectorySource
  {
  public:
    virtual std::vector<se2_z_trajectory::arc_traj_func::Ptr> getDesTrajs() const;
    
    virtual bool update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory);
    
    using Ptr=std::shared_ptr<ArcTrajectorySource>;
  };
  
  
  
  class AngledStraightTrajectorySource : public BasicGenerationTrajectorySource
  {
    
  public:
    virtual std::vector<se2_z_trajectory::arc_traj_func::Ptr> getDesTrajs() const;
    
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm);
    
    virtual bool update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory);
    
    using Ptr=std::shared_ptr<AngledStraightTrajectorySource>;
  };
  
} //end namespace nav_quadrotor



#endif // NAV_QUADROTOR_BASIC_TRAJECTORY_SOURCE_H
