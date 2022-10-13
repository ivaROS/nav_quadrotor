
#include <nav_quadrotor/quadrotor_controller_interface.h>
//#include <tf2_utils/odom_to_tf.h>
#include <nav_quadrotor/basic_trajectory_wrapper.h>
#include <ros/ros.h>

namespace nav_quadrotor
{
  
    QuadrotorControllerInterface::QuadrotorControllerInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm):
      rotors_control::ExtendableLeePositionControllerNode(nh, pnh),
      nh_(nh),
      pnh_(pnh),
      tfm_(tfm)
    {
      remaining_traj_pub_ = nh_.advertise<geometry_msgs::PoseArray>("remaining_trajectory", 1, true);
      //commanded_traj_pub_ = nh_.advertise<msg_type>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1, true);
      commanded_traj_pub_ = pnh_.advertise<msg_type>("current_trajectory_msg", 1, true);
      commanded_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(mav_msgs::default_topics::COMMAND_POSE, 1, true);
      frame_id_ = "world";
    }
    
    
    void QuadrotorControllerInterface::PreOdometryCallback(const nav_msgs::Odometry::ConstPtr& odometry_msg)
    {
      curr_odom_ = odometry_msg;
      //tf2_utils::AddToBuffer(curr_odom_, tfm_.getBuffer(), "odom"); //TODO: Check if this could result in TF_REPEATED_DATA warnings or cause any other problems
    }
    
    void QuadrotorControllerInterface::PostTimedCommandCallback(const ros::TimerEvent& e)
    {
      if(remaining_traj_pub_.getNumSubscribers()>0)
      {
        geometry_msgs::PoseArray remaining_poses = GetRemainingPoses(e.current_real);
        remaining_traj_pub_.publish(remaining_poses);
      }
    }
    
    geometry_msgs::PoseArray QuadrotorControllerInterface::GetRemainingPoses(ros::Time time, std::vector<ros::Duration>* times, std::vector<geometry_msgs::Twist>* twists)
    {
      auto lock = Lock(trajectory_mutex_);
      
      geometry_msgs::PoseArray remaining_poses;
      int num_poses = (int)commands_.size();
      std::vector<geometry_msgs::PoseStamped> poses;
      
      remaining_poses.header.stamp = time;
      remaining_poses.header.frame_id = frame_id_;
      for(int i = 0; i < num_poses; ++i)
      {
        geometry_msgs::PoseStamped pose;
        mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(commands_[i], &pose);
        remaining_poses.poses.push_back(pose.pose);
        
        if(twists)
        {
          trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
          mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(commands_[i], &msg);
          twists->push_back(msg.velocities[0]);
        }
      }
      
      if(times)
      {
        ros::Duration next_cmd_delay;
        if(time < next_command_time_)
        {
          next_cmd_delay = next_command_time_ - time;
        }
        else
        {
          ROS_WARN_STREAM_COND(num_poses > 0 && time != next_command_time_, "The next command should have already been executed! It is late by " << (time - next_command_time_).toSec()*1000 << "ms");
        }
        
        times->clear();
        times->push_back(next_cmd_delay);
        
        for(const auto& wait_time : command_waiting_times_)
        {
          times->push_back(times->back() + wait_time);
        }
        //times->insert(times->end(), command_waiting_times_.begin(), command_waiting_times_.end());
      }
      
      return remaining_poses;
    }
    
    TrajectoryWrapper::Ptr QuadrotorControllerInterface::getCurrentTrajectory()
    {
      std::vector<ros::Duration> durations;
      std::vector<geometry_msgs::Twist> twists;
      geometry_msgs::PoseArray poses = GetRemainingPoses(ros::Time::now(), &durations, &twists);
      auto current_traj = std::make_shared<BasicTrajectoryWrapper>(poses, durations, twists);
      current_traj->source_ = "current";
      //Lock lock(trajectory_mutex_);
      //TrajectoryWrapper::Ptr current_traj = std::make_shared<BasicTrajectoryWrapper>(commands_, command_waiting_times_, frame_id_);
      return current_traj;
    }
    
    void QuadrotorControllerInterface::setTrajectory(const msg_type::ConstPtr& trajectory_msg)
    {
      commanded_traj_pub_.publish(trajectory_msg);
      ExtendableLeePositionControllerNode::MultiDofJointTrajectoryCallback(trajectory_msg);
    }
    
    nav_msgs::Odometry::ConstPtr QuadrotorControllerInterface::getCurrentOdom()
    {
      return curr_odom_;
    }
    
    void QuadrotorControllerInterface::stop(bool force_stop)
    {
      auto odom_ptr = getCurrentOdom();
      if(odom_ptr)  //If no odometry has been received, then we aren't moving anyway
      {
        bool have_trajectory;
        {
          auto lock = Lock(trajectory_mutex_);
          have_trajectory = commands_.size()>1;
        }
        if(have_trajectory)
        {
          ROS_INFO_STREAM("Have trajectory, stopping");
          //TODO: Maybe publish as message to make this externally visible?
          auto odom = *odom_ptr;
          auto pose = boost::make_shared<geometry_msgs::PoseStamped>();
          pose->pose = odom.pose.pose;
          pose->header = odom.header;
          CommandPoseCallback(pose);
        }
      }
    }
    
} //end namespace nav_quadrotor

