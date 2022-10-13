#ifndef NAV_QUADROTOR_GLOBAL_GOAL_NAV_H
#define NAV_QUADROTOR_GLOBAL_GOAL_NAV_H

#include <trajectory_based_nav/global_goal_nav.h>
#include <trajectory_based_nav/planning_data.h>

namespace nav_quadrotor
{
  
  class GlobalGoalReplanLogic : public trajectory_based_nav::GlobalGoalReplanLogic
  {
  public:
    GlobalGoalReplanLogic(trajectory_based_nav::TrajectoryVerifier::Ptr verifier, std::shared_ptr<trajectory_based_nav::GlobalGoalState> ggs, std::string name="global_goal_replan_logic"):
      trajectory_based_nav::GlobalGoalReplanLogic(verifier, ggs, name),
      to_gap_(false),
      gap_min_tte_(0.1)
    {
      
    }
    
    virtual void setPlanningData(const trajectory_based_nav::PlanningData& data) 
    {
      using trajectory_based_nav::PlanningData;
      
      last_data_ = data;
      if (last_data_.trajectory_type==PlanningData::TrajectoryType::NONE)
      {
        to_gap_ = false;
      }
      else
      {
        if (last_data_.trajectory_type==PlanningData::TrajectoryType::PREVIOUS)
        {
          //Don't change to_gap_ status
        }
        else if (last_data_.trajectory_type==PlanningData::TrajectoryType::NEW)
        {
          auto source = last_data_.trajectory->getSource();
          to_gap_ = false; //(source == "end_point");
        }
      }
    }

    
    virtual bool shouldReplan(const nav_msgs::Odometry& odom, trajectory_based_nav::TrajectoryWrapper::Ptr traj) override
    {
      bool replan = false;

      if(ggs_->hasNewGoal())
      {
        ROS_INFO_STREAM_NAMED("replanning", "New goal received, replan!");
        replan = true;
      }
      else
      {
        auto last_planning_time = last_planning_time_;
      
  //       if(!current_trajectory_)
  //       {
  //         ROS_INFO_STREAM("No current trajectory, definitely replan!");
  //         return true;
  //       }
        
        auto durations = traj->getDurations();
        
        
        if(durations.size()==0 )
        {
          ROS_INFO_STREAM_NAMED("replanning", "[Replanning] No existing trajectory, time to replan!");
          replan = true;
        }
        else
        {
          if(durations.size()==1 && durations.back()==ros::Duration())
          {
            ROS_WARN_STREAM_NAMED("replanning", "[Replanning] Reached end of current trajectory, time to replan! (This represents an unexpected case!)");
            replan = true;
          }
          if(!(traj->to_goal || to_gap_) && durations.size()>0 && durations.back() < min_tte_)
          {
            ROS_INFO_STREAM_NAMED("replanning", "[Replanning] Time to end of trajectory=" << durations.back() << ", less than permitted minimum for trajectory not going to the goal or to a gap");
            replan = true;
          }
          else  
          if(to_gap_ && durations.size()>0 && durations.back() < gap_min_tte_)
          {
            ROS_INFO_STREAM_NAMED("replanning", "[Replanning] Time to end of gap-based trajectory=" << durations.back() << ", less than permitted minimum " << gap_min_tte_.toSec());
            replan = true;
          }
          if((!to_gap_) && (max_replan_period_ >= ros::Duration() && (odom.header.stamp - last_planning_time) > max_replan_period_))
          {
            ROS_INFO_STREAM_NAMED("replanning", "[Replanning] Time since last plan=" << (odom.header.stamp - last_planning_time) << ", time to replan! (Trajectory does not go to gap)");
            replan = true;
          }
          else
          if(!verifier_->verifyTrajectory(traj))
          {
            ROS_INFO_STREAM("Verifier is not satisfied, replan");
            replan = true;
          }
          
          if(replan)
          {
            last_planning_time_ = odom.header.stamp;
          }
          else
          {
            ROS_DEBUG_STREAM_NAMED("replanning", "[Replanning] Not replanning! Trajectory has " << durations.size() << " durations: " << durations.back().toSec());
          }
        }
      }
      return replan;
    }
    
    
  protected:
    trajectory_based_nav::PlanningData last_data_;
    bool to_gap_;
    ros::Duration gap_min_tte_;
  };
  
  
} //end namespace nav_quadrotor
  
#endif //NAV_QUADROTOR_GLOBAL_GOAL_NAV_H
