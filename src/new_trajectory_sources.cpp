#include <ros/assert.h>
#include <nav_quadrotor/interfaces.h>
#include <nav_quadrotor/basic_trajectory_wrapper.h>
#include <nav_quadrotor/ni_config_utility.h>
#include <nav_quadrotor/trajectory_functions.h>
#include <nav_quadrotor/new_trajectory_sources.h>
#include <trajectory_based_nav/global_goal_nav.h>
#include <limits>
#include <angles/angles.h>
#include <pips/utils/param_utils.h>


namespace nav_quadrotor
{    
    
    bool PreGenerationTrajectorySource::init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm) {return true;}
        
    bool PreGenerationTrajectorySource::update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory)
    {
      trajectories_.clear();
      traj_ind_ = 0;
      
      return true;
    }
    
    bool PreGenerationTrajectorySource::hasMore() const
    {
      return traj_ind_ < (int)trajectories_.size();
    }
    
    TrajectoryWrapper::Ptr PreGenerationTrajectorySource::getNext()
    {
      auto ret_val = trajectories_[traj_ind_];
      ++traj_ind_;
      return ret_val;
    }
    
    void PreGenerationTrajectorySource::yieldTrajectory(TrajectoryWrapper::Ptr traj)
    {
      trajectories_.push_back(traj);
    }
    

  

    bool SE2TrajectorySource::init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
    {
      if(!PreGenerationTrajectorySource::init(nh, pnh, tfm))
      {
        return false;
      }
      
      if(!future_dater_.init(nh, ros::NodeHandle(pnh, "future_dater"), tfm))
      {
        return false;
      }
      
      ni_config_.init(ros::NodeHandle(pnh, "ni_params"));
      
      double default_duration = 10;
      int num_attempts = 10;
      pips::utils::get_param(pnh, "max_traj_duration", max_traj_duration_, default_duration, num_attempts);
      
      return true;
    }
    
    bool SE2TrajectorySource::update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory)
    {
      return PreGenerationTrajectorySource::update(odom, current_trajectory) && future_dater_.update(odom, current_trajectory);
    }
    
    se2_z_trajectory::Params SE2TrajectorySource::getNIParams() const
    {
      return ni_config_.getParams();
    }
    
    se2_z_trajectory::state_type SE2TrajectorySource::getInitState(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory) const
    {
      const auto& future_odom = future_dater_.getFutureOdom();
      
      using namespace se2_z_trajectory;
      
      state_type x0;
      x0.actual.from(future_odom); //Could also have used `x0(future_odom)`, but this is more explicit
      x0.desired.from(future_odom.pose.pose.position);
      
      return x0;
    }
    
    void SE2TrajectorySource::yieldTrajectory(BasicTrajectoryWrapper::Ptr traj)
    {
      auto prepended_traj = future_dater_.getPrependedTrajectory(traj);
      prepended_traj->source_ = traj->source_;
      PreGenerationTrajectorySource::yieldTrajectory(prepended_traj);
    }

    
  

    bool TrajectoryObjectTrajectorySource::init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
    {
      if(!SE2TrajectorySource::init(nh, pnh, tfm))
      {
        return false;
      }
      
      return true;
    }
    
    bool TrajectoryObjectTrajectorySource::update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory)
    {
      return SE2TrajectorySource::update(odom, current_trajectory);
    }
    
    
    
  
    se2_z_trajectory::straight_traj_func straightLineRelativeGoal(double fw_vel, double dist, double relative_angle, double dz, double& max_tf)
    {
      using namespace se2_z_trajectory;
      
      double tf = dist/fw_vel;
      
      max_tf = std::min(tf, max_tf);
      
      double vz = dz/tf;
      
      auto dtraj = straight_traj_func(fw_vel,vz,relative_angle);
      return dtraj;
    }
    
    void getRelativePoseDirection2(const geometry_msgs::Point& pos, const geometry_msgs::Point& dpos, double& dist, double& angle, double& dz)
    {    
      double dx = dpos.x - pos.x;
      double dy = dpos.y - pos.y;
      dz = dpos.z - pos.z;
      
      dist = std::sqrt(dx*dx+dy*dy);//TODO: Maybe include dz?
      
      //double yaw = trajectory_based_nav::getYaw(odom.pose.pose.orientation);
      
      angle = std::atan2(dy,dx);
      
      ROS_DEBUG_STREAM_NAMED("getRelativePoseDirection2", "pos= " << pos << ", dpos= " << dpos << ", dist=" << dist << ", angle=" << angle << ", dz=" << dz);
    }
    
    se2_z_trajectory::straight_traj_func straightLineGlobalGoal(double fw_vel, const se2_z_trajectory::state_type& x0, const geometry_msgs::Point& dpos, double& max_tf)
    {
      geometry_msgs::Point pos;
      x0.actual.to(pos);
      
      double dist, angle, dz;
      getRelativePoseDirection2(pos, dpos, dist, angle, dz);
          
      //double relative_angle = angles::normalize_angle(angle - x0.theta);
      
      return straightLineRelativeGoal(fw_vel, dist, angle, dz, max_tf);
    }
    
    
    

  
    GlobalGoalTrajectorySource::GlobalGoalTrajectorySource(std::shared_ptr<trajectory_based_nav::GlobalGoalState> ggs):
      ggs_(ggs)
    {}
        
    bool GlobalGoalTrajectorySource::init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
    {
      if(!TrajectoryObjectTrajectorySource::init(nh, pnh, tfm))
      {
        return false;
      }
      
      //TODO: populate max_traj_duration_ and fw_vel_ from parameter server
      return true;
    }
    
    bool GlobalGoalTrajectorySource::update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory)
    {
      if(!TrajectoryObjectTrajectorySource::update(odom, current_trajectory))
      {
        return false;
      }
      
      using namespace se2_z_trajectory;
      
      state_type x0 = getInitState(odom, current_trajectory);
      
      
      ROS_ASSERT(ggs_->hasGoal());
      auto dpos = ggs_->getGoal().position;
      
      double max_tf = max_traj_duration_;
      auto dtraj = straightLineGlobalGoal(fw_vel_, x0, dpos, max_tf);
      
      auto ni_params = getNIParams();
      auto trajparams = traj_params();
      trajparams.tf = max_tf;
      auto trajobj = getTrajGenObject(dtraj, ni_params, x0, trajparams);
      makeTrajectory(trajobj, odom.header);
      return true;
    }
    
    
  
    
    bool FixedZRadiallySampledTrajectorySource::init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
    {
      if(!TrajectoryObjectTrajectorySource::init(nh, pnh, tfm))
      {
        return false;
      }
      
      //TODO: populate max_traj_duration_, fw_vel_, num_trajectories_, target_z_, and angle_range_ from parameter server
      return true;
    }
    
    bool FixedZRadiallySampledTrajectorySource::update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory)
    {
      if(!TrajectoryObjectTrajectorySource::update(odom, current_trajectory))
      {
        return false;
      }
      
      using namespace se2_z_trajectory;
      
      state_type x0 = getInitState(odom, current_trajectory);
      x0.desired.z = target_z_;
      
      auto ni_params = getNIParams();
      auto trajparams = traj_params();
      
      double lower_bound;
      double upper_bound;
      double step;
      
      if(num_trajectories_>1)
      {
        lower_bound = x0.theta-angle_range_/2;
        upper_bound = x0.theta+angle_range_/2;
        step = angle_range_/(num_trajectories_-1);
      }
      else
      {
        lower_bound = upper_bound = x0.theta;
        step = 1;
      }
      
      for(double angle=lower_bound; angle<=upper_bound; angle+=step)
      {
        geometry_msgs::Point dpos;
        x0.actual.to(dpos);
        dpos.x += traj_length_*std::cos(angle);
        dpos.y += traj_length_*std::sin(angle);
        dpos.z = target_z_;
        
        double max_tf = max_traj_duration_; //std::numeric_limits<double>::max();
        auto dtraj = straightLineGlobalGoal(fw_vel_, x0, dpos, max_tf);
        trajparams.tf = max_tf;
        auto trajobj = getTrajGenObject(dtraj, ni_params, x0, trajparams);
        makeTrajectory(trajobj, odom.header);
      }
      
      
      return true;
    }

    
    

    GlobalGoalZRadiallySampledTrajectorySource::GlobalGoalZRadiallySampledTrajectorySource(std::shared_ptr<trajectory_based_nav::GlobalGoalState> ggs):
      ggs_(ggs)
    {}
    
    bool GlobalGoalZRadiallySampledTrajectorySource::init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
    {
      if(!FixedZRadiallySampledTrajectorySource::init(nh, pnh, tfm))
      {
        return false;
      }
      
      //Add any needed parameters here
      return true;
    }
    
    bool GlobalGoalZRadiallySampledTrajectorySource::update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory)
    {
      ROS_ASSERT(ggs_->hasGoal());
      auto dpos = ggs_->getGoal().position;
      double target_z = dpos.z;
      
      target_z_ = target_z;
      
      return FixedZRadiallySampledTrajectorySource::update(odom, current_trajectory);
    }

  
} //end namespace nav_quadrotor
