#ifndef NAV_QUADROTOR_NEW_TRAJECTORY_SOURCES_H
#define NAV_QUADROTOR_NEW_TRAJECTORY_SOURCES_H

#include <ros/assert.h>
#include <nav_quadrotor/interfaces.h>
#include <nav_quadrotor/basic_trajectory_wrapper.h>
#include <nav_quadrotor/ni_config_utility.h>
#include <nav_quadrotor/trajectory_functions.h>
#include <trajectory_based_nav/global_goal_nav.h>
#include <trajectory_based_nav/future_dated_trajectory_helper.h>

namespace trajectory_based_nav
{
  template <typename T>
  class CompoundTrajectorySource : public TrajectorySource<T>
  {
    std::vector<typename TrajectorySource<T>::Ptr > sources_;
    mutable decltype(sources_.begin()) source_it_;  //Alternative to having this as 'mutable' would be updating it in getNext instead
    
  public:
    
    virtual ~CompoundTrajectorySource() = default;
      
    bool update(const nav_msgs::Odometry& odom, const TypedTrajectoryWrapper<T>& current_trajectory)
    {
      bool res = true;
      for(auto traj_src : sources_)
      {
        res |= traj_src->update(odom, current_trajectory);
      }
      
      source_it_ = sources_.begin();
      
      return res;
    }
    
    bool hasMore() const
    {
      for(; source_it_ != sources_.end(); ++source_it_)
      {
        if((*source_it_)->hasMore())
        {
          return true;
        }
      }
      
      return false;
    }
    
    typename trajectory_based_nav::TypedTrajectoryWrapper<T>::Ptr getNext()
    {
      return (*source_it_)->getNext();
    }
    
    template <typename S>
    void addSource(std::shared_ptr<S> derived_src)
    {
      typename TrajectorySource<T>::Ptr base_src = std::dynamic_pointer_cast<TrajectorySource<T> >(derived_src);
      
      ROS_ASSERT_MSG(base_src, "Tried to add invalid object type as a trajectory source!");
      
      sources_.push_back(base_src);
    }
  };
  
}


namespace nav_quadrotor
{
  using CompoundTrajectorySource = trajectory_based_nav::CompoundTrajectorySource<msg_type>;
  
  
  //Encapsulates how trajectories are pregenerated and accessed
  //The generation/access pattern should likely be applied more as a container rather than a base class
  class PreGenerationTrajectorySource : public TrajectorySource
  {
  protected:
    int traj_ind_;
    std::vector<TrajectoryWrapper::Ptr> trajectories_;
    
  public:
    
    virtual ~PreGenerationTrajectorySource() = default;
      
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm);
        
    virtual bool update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory);
    
    virtual bool hasMore() const;
    
    virtual TrajectoryWrapper::Ptr getNext();
    
  protected:
    virtual void yieldTrajectory(TrajectoryWrapper::Ptr traj);

  public:
    using Ptr=std::shared_ptr<PreGenerationTrajectorySource>;
  };
  
  
  //TODO: Add ability to generate trajectories from expected future pose, with the trajectory to that pose included as the start of the trajectory
  //This should really be applied in some general way at a higher level
  class SE2TrajectorySource : public PreGenerationTrajectorySource
  {
  protected:
    se2_z_trajectory::NIConfigUtility ni_config_;
    double fw_vel_ = 0.25;
    double max_traj_duration_ = 20;
    
    trajectory_based_nav::FutureDatedTrajectoryHelper future_dater_;

  public:
    SE2TrajectorySource() = default;
    
    virtual ~SE2TrajectorySource() = default;

    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm);
    
    virtual bool update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory);

    se2_z_trajectory::Params getNIParams() const;
    
    virtual se2_z_trajectory::state_type getInitState(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory) const;
    
    virtual void yieldTrajectory(BasicTrajectoryWrapper::Ptr traj);
    
  public:
    using Ptr=std::shared_ptr<SE2TrajectorySource>;
    
  };
  
  
  
  
  class TrajectoryObjectTrajectorySource : public SE2TrajectorySource
  {
  public:
    TrajectoryObjectTrajectorySource() = default;

    virtual ~TrajectoryObjectTrajectorySource() = default;
    
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm);
    
    virtual bool update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory);
    
    std::string name_;

  private:
    struct X
    {
        void operator()(BasicTrajectoryWrapper::Ptr& traj) const {}
    };
      
  protected:
    
    template <typename G, typename F=X>
    void makeTrajectory(G& trajobj, const std_msgs::Header& header, const F& func=X())
    {
      trajobj.run();
      auto traj = se2_z_trajectory::getTrajectoryStates(trajobj);
      traj->header=header;
      auto traj_wrapper_ptr = std::make_shared<BasicTrajectoryWrapper>(traj);
      traj_wrapper_ptr->source_ = name_;
      func(traj_wrapper_ptr);
      yieldTrajectory(traj_wrapper_ptr);
    }
    
  public:
    using Ptr=std::shared_ptr<TrajectoryObjectTrajectorySource>;
  };
  
  
  
  
  se2_z_trajectory::straight_traj_func straightLineRelativeGoal(double fw_vel, double dist, double relative_angle, double dz, double& max_tf);
  
  void getRelativePoseDirection2(const geometry_msgs::Point& pos, const geometry_msgs::Point& dpos, double& dist, double& angle, double& dz);
  
  se2_z_trajectory::straight_traj_func straightLineGlobalGoal(double fw_vel, const se2_z_trajectory::state_type& x0, const geometry_msgs::Point& dpos, double& max_tf);
  
  
  
  
  class GlobalGoalTrajectorySource : public TrajectoryObjectTrajectorySource
  {
  protected:
    std::shared_ptr<trajectory_based_nav::GlobalGoalState> ggs_;
    
  public:
  
    GlobalGoalTrajectorySource(std::shared_ptr<trajectory_based_nav::GlobalGoalState> ggs);
    
    virtual ~GlobalGoalTrajectorySource() = default;
    
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm);
  
    virtual bool update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory);
    
  protected:
//     std::vector<geometry_msgs::Point> getEndPoints(const se2_z_trajectory::state_type& x0)=0;
  public:
    using Ptr=std::shared_ptr<GlobalGoalTrajectorySource>;
  };
  
  
  class FixedZRadiallySampledTrajectorySource : public TrajectoryObjectTrajectorySource
  {
  protected:
    double traj_length_=4;
    int num_trajectories_ = 15;
    double angle_range_ = 2;
    double target_z_ = 1.5;
    
  public:
    
    FixedZRadiallySampledTrajectorySource() = default;
    
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm);
    
    virtual bool update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory);
    
  public:
    using Ptr = std::shared_ptr<FixedZRadiallySampledTrajectorySource>;
  };
  
  
  class GlobalGoalZRadiallySampledTrajectorySource : public FixedZRadiallySampledTrajectorySource
  {
  protected:
    std::shared_ptr<trajectory_based_nav::GlobalGoalState> ggs_;
    
  public:
    GlobalGoalZRadiallySampledTrajectorySource(std::shared_ptr<trajectory_based_nav::GlobalGoalState> ggs);
    
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm);
    
    virtual bool update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory);
    
  public:
    using Ptr = std::shared_ptr<GlobalGoalZRadiallySampledTrajectorySource>;
  };
  
  
  
  
  
  
  
  
  
//   AngledStraightGoalTrajectorySource(std::shared_ptr<trajectory_based_nav::GlobalGoalState> ggs):
//   GoalBasedTrajectorySource(ggs),
//   max_traj_duration(10)
//   {}
//   
//   bool AngledStraightGoalTrajectorySource::init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
//   {
//     GoalBasedTrajectorySource::init(nh,pnh,tfm);
//     //TODO: Add reconfigure for velocity, number of trajectories, range of angles
//     return true;
//   }
//   
//   bool AngledStraightGoalTrajectorySource::update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory)
//   {
//     using namespace se2_z_trajectory;
//     
//     state_type x0(odom);
//     x0.actual.from(odom);
//     x0.desired.from(odom.pose.pose.position);
//     x0.desired.z = 2;
//     
//     se2_z_trajectory::Params params = getNIParams();
//     
//     double fw_vel = .25;
//     //auto trajectories = std::vector<nav_quadrotor::se2_z_trajectory::traj_type_ptr>(); //boost::make_shared<nav_quadrotor::TrajectoryArray>();
//     trajectories_.clear();
//     
//     int num_trajectories = 15;
//     double angle_range = 4;
//     
//     double lower_bound;
//     double upper_bound;
//     double step;
//     
//     if(num_trajectories>1)
//     {
//       lower_bound = x0.theta-angle_range/2;
//       upper_bound = x0.theta+angle_range/2;
//       step = angle_range/(num_trajectories-1);
//     }
//     else
//     {
//       lower_bound = upper_bound = x0.theta;
//       step = 1;
//     }
//     
//     int i=0;
//     for(double angle=lower_bound; i<num_trajectories; angle+=step, i++)
//     {
//       //for(double length = 0.5; length<=30; length*=2)
//       double length = 4;
//       {
//         double tf = length/fw_vel;
//         //double vz = 0;
//         int i=0;
//         int num_vzs = 5;
//         double min_vz = -0.1;
//         double max_vz = 0.1;
//         for(double vz=min_vz; i < num_vzs; vz+=(max_vz-min_vz)/(num_vzs-1), i++)
//         {
//           
//           auto dtraj = straight_traj_func(fw_vel,vz,angle);
//           auto trajparams = traj_params();
//           trajparams.tf = tf;
//           auto trajobj = getTrajGenObject(dtraj, params, x0, trajparams);
//           
//           /*int steps = */ trajobj.run();
//           auto traj = getTrajectoryStates(trajobj);
//           traj->header=odom.header;
//           auto traj_wrapper = std::make_shared<BasicTrajectoryWrapper>(traj);
//           trajectories_.push_back(traj_wrapper);
//         }
//       }
//     }
//     
//       else
//       {
//         ROS_WARN_STREAM("Goal (" << normed_angle << ") does not fall between [" << normed_lower_bound << ", " << normed_upper_bound << "]");
//       }
//     }
//     else
//     {
//       ROS_WARN("Failed to get local goal!");
//     }
//     
//     traj_ind_ = 0;
//     return true;
//   }
//   
//   
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  

  
//   class TrajectoryObjectTrajectorySource : public SE2TrajectorySource
//   {
//   public:
//     
//     virtual bool update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory)
//     {
//       if(!SE2TrajectorySource::update(odom, current_trajectory))
//       {
//         return false;
//       }
//       se2_z_trajectory::state_type x0 = getInitState(odom, current_trajectory);
//       
//       GeneralInputs gi(x0, getNIParams(), odom.header);
//       return generateTrajObs(gi);
//       
//     }
//     
//   protected:
//     
//     struct GeneralInputs
//     {
//       se2_z_trajectory::state_type x0;
//       se2_z_trajectory::Params ni_params;
//       std_msgs::Header header;
//       
//       GeneralInputs(se2_z_trajectory::state_type x0, se2_z_trajectory::Params ni_params, std_msgs::Header header):
//         x0(x0),
//         ni_params(ni_params),
//         header(header)
//         {}
//     }
//     
//     bool generateTrajObs(const GeneralInputs& gi) = 0;
//     
//     template <typename G>
//     void makeTrajectory(G& trajobj, const GeneralInputs& gi)
//     {
//       trajobj.run();
//       auto traj = se2_z_trajectory::getTrajectoryStates(trajobj);
//       traj->header=gi.header;
//       auto traj_wrapper = std::make_shared<BasicTrajectoryWrapper>(traj);
//       yieldTrajectory(traj_wrapper);
//     }
//     
//   public:
//     using Ptr=std::shared_ptr<TrajectoryObjectTrajectorySource>;
//   };
  

  
  
  
//   class StraightLineTrajObjSource : public TrajectoryObjectTrajectorySource
//   {
//     double v_des_;
//     
//   public:
// 
//     virtual void makeTrajectory(double distance, double angle, double dz, const std_msgs::Header& header)
//     {
//       auto traj_des = straight_traj_func(v_des_, vz, angle);
//       auto trajparams = traj_params();
//       trajparams.tf = tf;
//       
//       auto trajobj = getTrajGenObject(dtraj, params, x0, trajparams);
//       yieldTrajectory(trajobj, header);
//     }
//   };
  
//   class LocalEndpointTrajObjSource : public StraightLineTrajObjSource
//   {
//     double v_des_=0.25;
//     double max_traj_duration_=100;
//     
//   protected:
//     virtual void makeTrajectory(const GeneralInputs& gi, double distance, double angle, double dz)
//     {
//       using namespace se2_z_trajectory;
//       
//       double tf = distance/fw_vel;
//       tf = std::min(tf, max_traj_duration_);
//       
//       double vz = dz/tf;
//       
//       auto traj_des = straight_traj_func(v_des_, vz, angle);
//       auto trajparams = traj_params();
//       trajparams.tf = tf;
//       
//       auto trajobj = getTrajGenObject(traj_des, gi.ni_params, gi.x0, trajparams);
//       makeTrajectory(trajobj, gi.header);
//     }
//   };
//   
//   class GlobalEndpointsTrajObjSource : public LocalEndpointTrajObjSource
//   {
//     virtual bool generateTrajObs(const GeneralInputs& gi)
//     {
//       
//     }
//     
//     
//     virtual void makeTrajectory(
//   };
//   
//   class GlobalGoalTrajObjSource : public GlobalEndpointTrajObjSource
//   {
//     
//   };
//   
//   

  
//   std::vector<se2_z_trajectory::traj_func_type::Ptr> TrajectoryFunctionTrajectorySource::getTrajGenObjs(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory) const
//   {
//     using namespace se2_z_trajectory;
//     
//     Params params =  getNIParams();
//     
//     se2_z_trajectory::state_type x0 = getInitState(odom, current_trajectory);
//     
//     auto des_trajs = getDesTrajFuncs(odom, current_trajectory);
//     
//     for(auto dtraj : des_trajs)
//     {
//       auto trajobj = getTrajGenObject(*dtraj, params, x0);
//       
//     }
//   }
//   
//   
//   
//   std::vector<se2_z_trajectory::arc_traj_func::Ptr> ArcTrajectorySource::getDesTrajs() const
//   {
//     double fw_vel = .25;
//     
//     std::vector<se2_z_trajectory::arc_traj_func::Ptr> des_trajs;
//     for(double w_des=-1; w_des<=1; w_des+=2.0/5)
//     {
//       double vz = 0;
//       //for(double vz=-0.2; vz < 0.2; vz+=0.4/3)
//       {
//         auto dtraj = std::make_shared<se2_z_trajectory::arc_traj_func>(fw_vel,vz,w_des);
//         des_trajs.push_back(dtraj);
//       }
//     }
//     return des_trajs;
//   }
//   
//   
//   bool ArcTrajectorySource::update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory)
//   {
//     using namespace se2_z_trajectory;
//     
//     state_type x0(odom);
//     x0.actual.from(odom);
//     x0.desired.from(odom.pose.pose.position);
//     x0.desired.z = 2;
//     
//     Params params =  getNIParams();
//     
//     double fw_vel = .25;
//     trajectories_.clear();
//     
//     for(double w_des=-1; w_des<=1; w_des+=2.0/15)
//     {
//       double vz = 0;
//       //for(double vz=-0.2; vz < 0.2; vz+=0.4/3)
//       
//       auto dtraj = arc_traj_func(fw_vel,vz,w_des);
//       auto trajobj = getTrajGenObject(dtraj, params, x0);
//       
//       /*int steps =*/ trajobj.run();
//       auto traj = getTrajectoryStates(trajobj);
//       traj->header=odom.header;
//       //trajectories.push_back(traj);
//       auto traj_wrapper = std::make_shared<BasicTrajectoryWrapper>(traj);
//       trajectories_.push_back(traj_wrapper);
//       //trajectories->trajectories.push_back(traj->toMsg());
//       
//     }
//     
//     traj_ind_ = 0;
//     return true;
//   }
//   
//   
//   
//   std::vector<se2_z_trajectory::arc_traj_func::Ptr> AngledStraightTrajectorySource::getDesTrajs() const
//   {
//     double fw_vel = .25;
//     
//     std::vector<se2_z_trajectory::arc_traj_func::Ptr> des_trajs;
//     for(double w_des=-1; w_des<=1; w_des+=2.0/5)
//     {
//       double vz = 0;
//       //for(double vz=-0.2; vz < 0.2; vz+=0.4/3)
//       {
//         auto dtraj = std::make_shared<se2_z_trajectory::arc_traj_func>(fw_vel,vz,w_des);
//         des_trajs.push_back(dtraj);
//       }
//     }
//     return des_trajs;
//   }
//   
//   bool AngledStraightTrajectorySource::init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
//   {
//     BasicGenerationTrajectorySource::init(nh,pnh,tfm);
//     //TODO: Add reconfigure for velocity, number of trajectories, range of angles
//     return true;
//   }
//   
//   bool AngledStraightTrajectorySource::update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory)
//   {
//     using namespace se2_z_trajectory;
//     
//     state_type x0(odom);
//     x0.actual.from(odom);
//     x0.desired.from(odom.pose.pose.position);
//     x0.desired.z = 2;
//     
//     se2_z_trajectory::Params params = getNIParams();
//     
//     double fw_vel = .25;
//     //auto trajectories = std::vector<nav_quadrotor::se2_z_trajectory::traj_type_ptr>(); //boost::make_shared<nav_quadrotor::TrajectoryArray>();
//     trajectories_.clear();
//     
//     int num_trajectories = 15;
//     double angle_range = 2;
//     
//     double lower_bound;
//     double upper_bound;
//     double step;
//     
//     if(num_trajectories>1)
//     {
//       lower_bound = x0.theta-angle_range/2;
//       upper_bound = x0.theta+angle_range/2;
//       step = angle_range/(num_trajectories-1);
//     }
//     else
//     {
//       lower_bound = upper_bound = x0.theta;
//       step = 1;
//     }
//     
//     for(double angle=lower_bound; angle<=upper_bound; angle+=step)
//     {
//       double vz = 0;
//       //for(double vz=-0.2; vz < 0.2; vz+=0.4/3)
//       
//       auto dtraj = straight_traj_func(fw_vel,vz,angle);
//       auto trajobj = getTrajGenObject(dtraj, params, x0);
//       
//       /*int steps =*/ trajobj.run();
//       auto traj = getTrajectoryStates(trajobj);
//       traj->header=odom.header;
//       auto traj_wrapper = std::make_shared<BasicTrajectoryWrapper>(traj);
//       trajectories_.push_back(traj_wrapper);
//       
//     }
//     
//     traj_ind_ = 0;
//     return true;
//   }
//   
  
} //end namespace nav_quadrotor

#endif //NAV_QUADROTOR_NEW_TRAJECTORY_SOURCES_H
