#include <nav_quadrotor/end_point_trajectory_source.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <nav_quadrotor/bent_straight_line_trajectory_function.h>

namespace nav_quadrotor
{

    EndPointTrajectorySource::EndPointTrajectorySource()
    {}
        
    bool EndPointTrajectorySource::init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
    {
      if(!TrajectoryObjectTrajectorySource::init(nh, pnh, tfm))
      {
        return false;
      }
      
      fixed_frame_id_ = "world";
      //pips::utils::searchParam(ppnh, "fixed_frame_id", fixed_frame_id_, "world", -1);

      
      tfm_ = tfm;
      
      transformed_poses_pub_ = pnh.advertise<geometry_msgs::PoseArray>("transformed_candidate_waypoints", 2, true);
      end_poses_sub_.subscribe(pnh, "candidate_waypoints", 2);
      
      //TODO: Parameterize frame_ids
      end_poses_filter_ = std::make_shared<tf_filter>(end_poses_sub_, *tfm.getBuffer(), fixed_frame_id_, 2, nh);
      end_poses_cache_ = std::make_shared<message_filters::Cache<geometry_msgs::PoseArray> >(*end_poses_filter_, 1);
      
      return true;
    }
    
    
    geometry_msgs::PoseArray::ConstPtr EndPointTrajectorySource::getTargetPoses(const std_msgs::Header& header)
    {
      ros::Time anytime;
      geometry_msgs::PoseArray::ConstPtr msg = end_poses_cache_->getElemAfterTime(anytime);

      if(!msg)
      {
        ROS_WARN_STREAM("No candidate endpoints received on topic [" << end_poses_sub_.getTopic() << "]");
        return nullptr;
      }

      geometry_msgs::PoseArray::Ptr transformed_pose_array;
      {
        geometry_msgs::TransformStamped transform;
        try
        {
          transform = tfm_.getBuffer()->lookupTransform(header.frame_id, header.stamp, msg->header.frame_id, msg->header.stamp, fixed_frame_id_);
        }
        catch (tf2::TransformException &ex) 
        {
          ROS_WARN_STREAM("Problem finding transform:\n" <<ex.what());
          return nullptr;
        }
        
        transformed_pose_array = boost::make_shared<geometry_msgs::PoseArray>();
        for(const auto& pose : msg->poses)
        {
          geometry_msgs::PoseStamped pose_stamped;
          pose_stamped.pose = pose;
          pose_stamped.header = msg->header;
          geometry_msgs::PoseStamped transformed_pose;
          tf2::doTransform(pose_stamped, transformed_pose, transform);
          transformed_pose_array->poses.push_back(transformed_pose.pose);
          transformed_pose_array->header = transformed_pose.header;
        }
      }
      
      return transformed_pose_array;      
    }
    
    
    bool EndPointTrajectorySource::update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory)
    {
      if(!TrajectoryObjectTrajectorySource::update(odom, current_trajectory))
      {
        return false;
      }
      
      auto target_pose_array = getTargetPoses(odom.header);

      if(target_pose_array)
      {
        transformed_poses_pub_.publish(target_pose_array);
        
        using namespace se2_z_trajectory;
      
        state_type x0 = getInitState(odom, current_trajectory);
        auto ni_params = getNIParams();
        
        for(const auto end_pose : target_pose_array->poses)
        {
          if(false)
          {
            double max_tf = max_traj_duration_;
            auto dtraj = straightLineGlobalGoal(fw_vel_, x0, end_pose.position, max_tf);
            
            auto trajparams = traj_params();
            trajparams.tf = max_tf;
            auto trajobj = getTrajGenObject(dtraj, ni_params, x0, trajparams);
          }
          double target_z_factor_ = 0.8;
          auto trajobj = getBentStraightLineTrajObject(x0, end_pose.position, fw_vel_, max_traj_duration_, target_z_factor_, ni_params);
          makeTrajectory(trajobj, odom.header);
        }
        
        return true;
      }
      else
      {
        return false;
      }
    }


} //end namespace nav_quadrotor
