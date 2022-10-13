//TODO: Move to trajectory_based_nav package
#include<nav_quadrotor/multi_level_trajectory_verifier.h>
#include<trajectory_based_nav/interfaces.h>


namespace trajectory_based_nav
{
    
    MultiLevelTrajectoryVerifier::MultiLevelTrajectoryVerifier(std::string name) :
      name_(name)
    {
      
    }
    
    bool MultiLevelTrajectoryVerifier::init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
    {      
      auto ppnh = ros::NodeHandle(pnh, name_);
      
      double colliding_time = 10;
      ppnh.getParam("min_colliding_time", colliding_time);
      ppnh.setParam("min_colliding_time", colliding_time);
      
      min_colliding_time_ = ros::Duration(colliding_time);
      
      return true;
    }

    void MultiLevelTrajectoryVerifier::setCCWrappers(std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> cons_wrapper, std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> lib_wrapper, std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> det_wrapper, std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> egocan_wrapper1, std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> egocan_wrapper2)
    {
      conservative_wrapper_ = cons_wrapper;
      liberal_wrapper_ = lib_wrapper;
      detailed_wrapper_ = det_wrapper;
      egocan_wrapper1_ = egocan_wrapper1;
      egocan_wrapper2_ = egocan_wrapper2;
      
      cc_wrappers_.push_back(conservative_wrapper_);
      cc_wrappers_.push_back(liberal_wrapper_);
      cc_wrappers_.push_back(detailed_wrapper_);
      cc_wrappers_.push_back(egocan_wrapper1_);
      cc_wrappers_.push_back(egocan_wrapper2_);
    }
    
    bool MultiLevelTrajectoryVerifier::verifyTrajectory(TrajectoryWrapper::Ptr traj)
    {
      int collision_ind;
      /* TODO: Use min_colliding_time_ to determine max # of poses to collision check, and if no collision detected before then, 
       * make a note indicating that later poses weren't checked in case other parts of the system care and need to check further
       */
      collision_ind = collisionCheckTrajectory(traj);
      
      if(collision_ind >=0)
      {
        auto collision_time = traj->getDurations()[std::min((int)traj->getDurations().size()-1,collision_ind)];
        if(collision_time < min_colliding_time_)
        {
          ROS_INFO_STREAM_NAMED("multi_level_trajectory_verifier", "[" << name_ << "] Trajectory failed verification: collision_ind=" << collision_ind << ", collision_time=" << collision_time << " (min_colliding_time=" << min_colliding_time_ << ")");
          
          return false;
        }
        else
        {
          ROS_INFO_STREAM_NAMED("multi_level_trajectory_verifier", "[" << name_ << "] Trajectory passed verification: collision_ind=" << collision_ind << ", collision_time=" << collision_time << " (min_colliding_time=" << min_colliding_time_ << ")");
          
          return true;
        }
      }
      else
      {
        ROS_INFO_STREAM_NAMED("multi_level_trajectory_verifier", "[" << name_ << "] Trajectory passed verification: it does not collide");
        
        return true;
      }
      
      return true;  //Execution never reaches here, but for whatever reason compiler doesn't recognize that
    }


    bool MultiLevelTrajectoryVerifier::update(const nav_msgs::Odometry& odom)
    {
      for(const auto cc_wrapper : cc_wrappers_)
      {
        std_msgs::Header header = cc_wrapper->getCurrentHeader();
      
        if(!cc_wrapper->isReady(odom.header))
        {
          return false;
        }
        cc_wrapper->update();
      }
      return true;
    }
    
    void MultiLevelTrajectoryVerifier::collisionCheckPose(const geometry_msgs::Pose& pose, bool& safe, bool& collided)
    {
      auto& cons_cc = *(conservative_wrapper_->getCC());
      auto& lib_cc = *(liberal_wrapper_->getCC());
      auto& det_cc = *(detailed_wrapper_->getCC());
      
      auto& egocan1 = *(egocan_wrapper1_->getCC());
      auto& egocan2 = *(egocan_wrapper2_->getCC());
      
      collided = false;
      
      CCOptions cco(true);
      
      bool egocan_res = egocan1.testCollision(pose, cco) || egocan2.testCollision(pose, cco);
      num_egocan_checks_++;
      
      if(egocan_res)
      {
        collided = true;
      }
      else
      {
        bool cons_res = cons_cc.testCollision(pose, cco);
        num_cons_checks_++;
        if(cons_res)
        {
          bool lib_res = lib_cc.testCollision(pose, cco);
          num_lib_checks_++;
          if(lib_res)
          {
            //ROS_ERROR_STREAM("Actually had a liberal failure!" << pose);
            collided = true;
          }
          else
          {
            collided = det_cc.testCollision(pose, cco);
            num_det_checks_++;
          }
        }
      }
      ROS_WARN_STREAM_THROTTLE(1, "[" << name_ << "] Performed " << num_egocan_checks_ << " egocan, " << num_cons_checks_ << " conservative, " << num_lib_checks_ << " liberal, and " << num_det_checks_ << " detailed collision checks");
    }
    
    
    int MultiLevelTrajectoryVerifier::collisionCheckTrajectory(TrajectoryWrapper::Ptr traj)
    {
      int collision_ind; //, safe_ind
      const auto& poses = traj->getPoseArray().poses;
      int num_remaining_poses = (int)poses.size();
      
      int step = 1;
      int i;
      for(i = 0; i < num_remaining_poses; i+=step)
      {
        bool safe, collided;
        collisionCheckPose(poses[i], safe, collided);
        if(collided)
        {
          collision_ind = i;
          return collision_ind;
        }
//         else
//         {
//           safe_ind = i;
//         }
      }
      collision_ind = -1;
      return collision_ind;
    }

} //end namespace trajectory_based_nav
