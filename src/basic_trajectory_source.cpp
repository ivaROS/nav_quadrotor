#include <nav_quadrotor/basic_trajectory_source.h>
// #include <nav_quadrotor/trajectory_functions.h>


namespace nav_quadrotor
{

    bool BasicGenerationTrajectorySource::init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
    {
      ni_config_.init(ros::NodeHandle(pnh, "ni_params"));
      return true;
    }
    
    se2_z_trajectory::Params BasicGenerationTrajectorySource::getNIParams()
    {
      return ni_config_.getParams();
    }
        
    bool BasicGenerationTrajectorySource::hasMore() const
    {
      return traj_ind_ < (int)trajectories_.size();
    }
    
    TrajectoryWrapper::Ptr BasicGenerationTrajectorySource::getNext()
    {
      auto ret_val = trajectories_[traj_ind_];
      ++traj_ind_;
      return ret_val;
    }


    std::vector<se2_z_trajectory::arc_traj_func::Ptr> ArcTrajectorySource::getDesTrajs() const
    {
      double fw_vel = .25;
      
      std::vector<se2_z_trajectory::arc_traj_func::Ptr> des_trajs;
      for(double w_des=-1; w_des<=1; w_des+=2.0/5)
      {
        double vz = 0;
        //for(double vz=-0.2; vz < 0.2; vz+=0.4/3)
        {
          auto dtraj = std::make_shared<se2_z_trajectory::arc_traj_func>(fw_vel,vz,w_des);
          des_trajs.push_back(dtraj);
        }
      }
      return des_trajs;
    }
    
    bool ArcTrajectorySource::update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory)
    {
      using namespace se2_z_trajectory;
      
      state_type x0(odom);
      x0.actual.from(odom);
      x0.desired.from(odom.pose.pose.position);
      x0.desired.z = 2;
      
      Params params =  getNIParams();
      
      double fw_vel = .25;
      trajectories_.clear();
      
      for(double w_des=-1; w_des<=1; w_des+=2.0/15)
      {
        double vz = 0;
        //for(double vz=-0.2; vz < 0.2; vz+=0.4/3)
                
        auto dtraj = arc_traj_func(fw_vel,vz,w_des);
        auto trajobj = getTrajGenObject(dtraj, params, x0);
        
        /*int steps =*/ trajobj.run();
        auto traj = getTrajectoryStates(trajobj);
        traj->header=odom.header;
        //trajectories.push_back(traj);
        auto traj_wrapper = std::make_shared<BasicTrajectoryWrapper>(traj);
        trajectories_.push_back(traj_wrapper);
        //trajectories->trajectories.push_back(traj->toMsg());
        
      }
      
      traj_ind_ = 0;
      return true;
    }
    
    

    std::vector<se2_z_trajectory::arc_traj_func::Ptr> AngledStraightTrajectorySource::getDesTrajs() const
    {
      double fw_vel = .25;
      
      std::vector<se2_z_trajectory::arc_traj_func::Ptr> des_trajs;
      for(double w_des=-1; w_des<=1; w_des+=2.0/5)
      {
        double vz = 0;
        //for(double vz=-0.2; vz < 0.2; vz+=0.4/3)
        {
          auto dtraj = std::make_shared<se2_z_trajectory::arc_traj_func>(fw_vel,vz,w_des);
          des_trajs.push_back(dtraj);
        }
      }
      return des_trajs;
    }
    
    bool AngledStraightTrajectorySource::init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
    {
      BasicGenerationTrajectorySource::init(nh,pnh,tfm);
      //TODO: Add reconfigure for velocity, number of trajectories, range of angles
      return true;
    }
    
    bool AngledStraightTrajectorySource::update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory)
    {
      using namespace se2_z_trajectory;
      
      state_type x0(odom);
      x0.actual.from(odom);
      x0.desired.from(odom.pose.pose.position);
      x0.desired.z = 2;
      
      se2_z_trajectory::Params params = getNIParams();
      
      double fw_vel = .25;
      //auto trajectories = std::vector<nav_quadrotor::se2_z_trajectory::traj_type_ptr>(); //boost::make_shared<nav_quadrotor::TrajectoryArray>();
      trajectories_.clear();
      
      int num_trajectories = 15;
      double angle_range = 2;
      
      double lower_bound;
      double upper_bound;
      double step;
      
      if(num_trajectories>1)
      {
        lower_bound = x0.theta-angle_range/2;
        upper_bound = x0.theta+angle_range/2;
        step = angle_range/(num_trajectories-1);
      }
      else
      {
        lower_bound = upper_bound = x0.theta;
        step = 1;
      }
      
      for(double angle=lower_bound; angle<=upper_bound; angle+=step)
      {
        double vz = 0;
        //for(double vz=-0.2; vz < 0.2; vz+=0.4/3)
        
        auto dtraj = straight_traj_func(fw_vel,vz,angle);
        auto trajobj = getTrajGenObject(dtraj, params, x0);
        
        /*int steps =*/ trajobj.run();
        auto traj = getTrajectoryStates(trajobj);
        traj->header=odom.header;
        auto traj_wrapper = std::make_shared<BasicTrajectoryWrapper>(traj);
        trajectories_.push_back(traj_wrapper);
        
      }
      
      traj_ind_ = 0;
      return true;
    }
    
  
} //end namespace nav_quadrotor

