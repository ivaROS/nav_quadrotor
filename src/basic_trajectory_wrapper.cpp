#include <nav_quadrotor/basic_trajectory_wrapper.h>

namespace nav_quadrotor
{

  geometry_msgs::PoseArray TrajectoryToPoses(const msg_type& trajectory)
  {
    geometry_msgs::PoseArray pose_array;
    //pose_array.header.stamp = trajectory.header.stamp;
    //pose_array.header.frame_id = global_frame_id_;
    pose_array.header = trajectory.header;
    for(const auto& point : trajectory.points)
    {
      pose_array.poses.emplace_back();
      auto& pose = pose_array.poses.back();
      pose.position.x = point.transforms[0].translation.x;
      pose.position.y = point.transforms[0].translation.y;
      pose.position.z = point.transforms[0].translation.z;
      pose.orientation = point.transforms[0].rotation;
    }
    return pose_array;
  }

  std::vector<geometry_msgs::Twist> TrajectoryToTwists(const msg_type& trajectory)
  {
    std::vector<geometry_msgs::Twist> twists;
    for(const auto& point : trajectory.points)
    {
      twists.emplace_back();
      auto& twist = twists.back();
      twist = point.velocities[0];
    }
    return twists;
  }
  
  msg_type PosesToTrajectory(const geometry_msgs::PoseArray& poses, const std::vector<geometry_msgs::Twist>& twists, const std::vector<ros::Duration>& durations)
  {
    msg_type trajectory;
    trajectory.header = poses.header;
    
    for(size_t i = 0; i < poses.poses.size(); i++)
    {
      trajectory.points.emplace_back();
      auto& traj_point = trajectory.points.back();
      
      traj_point.transforms.emplace_back();
      auto& trans = traj_point.transforms.back().translation;
      const auto& pos = poses.poses[i].position;
      trans.x = pos.x;
      trans.y = pos.y;
      trans.z = pos.z;
      
      traj_point.transforms.back().rotation = poses.poses[i].orientation;
      
      traj_point.velocities.emplace_back(twists[i]);
      
      traj_point.time_from_start = durations[i];
    }
    return trajectory;
  }
  
  BasicTrajectoryWrapper::BasicTrajectoryWrapper(se2_z_trajectory::traj_type_ptr traj)
  {
    trajectory_ = traj;
    traj_msg_ = boost::make_shared<msg_type>(trajectory_->toMsg());
    pose_array_ = TrajectoryToPoses(*traj_msg_);
    twists_ = TrajectoryToTwists(*traj_msg_);
    
    for(double d : trajectory_->times)
    {
      durations_.push_back(ros::Duration(d));
    }
  }
  
  BasicTrajectoryWrapper::BasicTrajectoryWrapper(geometry_msgs::PoseArray pose_array, std::vector<ros::Duration> durations, std::vector<geometry_msgs::Twist> twists)
  {
    pose_array_ = pose_array;
    durations_ = durations;
    twists_ = twists;
    traj_msg_ = boost::make_shared<msg_type>();
    *traj_msg_ = PosesToTrajectory(pose_array, twists, durations);
  }
  
  //     BasicTrajectoryWrapper(const mav_msgs::EigenTrajectoryPointDeque& commands, const std::deque<ros::Duration>& command_waiting_times, std::string frame_id="world")
  //     {
  //       size_t num_poses = commands.size();
  // 
  //       pose_array_.header.stamp = ros::Time::now();
  //       pose_array_.header.frame_id = frame_id;
  //       for(int i = 0; i < num_poses; ++i)
  //       {
  //         geometry_msgs::PoseStamped pose;
  //         mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(commands[i], &pose);
  //         pose_array_.poses.push_back(pose.pose);
  //       }
  //       
  //       durations_ = std::vector<ros::Duration>(command_waiting_times.begin(), command_waiting_times.end());
  //             
  //       for(int i = 1; i < durations_.size(); i++)
  //       {
  //         durations_[i] += durations_[i-1];
  //       }
  //     }
  
  BasicTrajectoryWrapper BasicTrajectoryWrapper::makeTrimmed(unsigned int size)
  {
    //TODO: Only copy over relevant elements in the first place
    BasicTrajectoryWrapper new_wrapper;
    new_wrapper.pose_array_ = pose_array_;
    new_wrapper.pose_array_.poses.resize(size);
    
    if(traj_msg_)
    {
      new_wrapper.traj_msg_ = boost::make_shared<msg_type>(*traj_msg_);
      new_wrapper.traj_msg_->points.resize(size);
    }
    
    new_wrapper.durations_ = durations_;
    new_wrapper.durations_.resize(size);
    
    new_wrapper.twists_ = twists_;
    new_wrapper.twists_.resize(size);
    
    return new_wrapper;
  }
  
  bool BasicTrajectoryWrapper::trim(unsigned int size)
  {
    if(size > durations_.size())
    {
      ROS_ERROR_STREAM("Trying to resize trajectory larger than it already is! Commanded " << size << " but is currently " << durations_.size());
      return false;
    }
    
    pose_array_.poses.resize(size);
    durations_.resize(size);
    if(traj_msg_)
    {
      traj_msg_->points.resize(size);
    }
    return true;
  }
  
}
