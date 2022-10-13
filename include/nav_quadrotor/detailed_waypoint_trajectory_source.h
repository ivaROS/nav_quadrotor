#ifndef NAV_QUADROTOR_DETAILED_WAYPOINT_TRAJECTORY_SOURCE_H
#define NAV_QUADROTOR_DETAILED_WAYPOINT_TRAJECTORY_SOURCE_H

#include <nav_quadrotor/new_trajectory_sources.h>
#include <nav_quadrotor/CandidateWaypoints.h>
#include <nav_quadrotor/DetailedWaypointArray.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>
#include <nav_quadrotor/multi_level_trajectory_verifier.h>

namespace nav_quadrotor
{
  
  class DetailedWaypointTrajectorySource : public TrajectoryObjectTrajectorySource
  {
  public:
    using WP_MSG = nav_quadrotor::DetailedWaypointArray; //nav_quadrotor::CandidateWaypoints;
    
  protected:
    std::string fixed_frame_id_;    
    tf2_utils::TransformManager tfm_;
    
    message_filters::Subscriber<geometry_msgs::PoseArray> end_poses_sub_;
    ros::Publisher transformed_poses_pub_, resampled_poses_pub_;
    
    message_filters::Subscriber<WP_MSG> detailed_waypoints_sub_;
    
    using tf_filter = tf2_ros::MessageFilter<WP_MSG>;
    std::shared_ptr<tf_filter> end_poses_filter_;
    std::shared_ptr<message_filters::Cache<WP_MSG> > end_poses_cache_;
    trajectory_based_nav::MultiLevelTrajectoryVerifier::Ptr verifier_;
    
  public:
    
    DetailedWaypointTrajectorySource(trajectory_based_nav::MultiLevelTrajectoryVerifier::Ptr verifier=nullptr);
        
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm);

    virtual WP_MSG::ConstPtr getTargetPoses(const std_msgs::Header& header, geometry_msgs::TransformStamped& transform);
    
    virtual bool update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory);

    
    using Ptr=std::shared_ptr<DetailedWaypointTrajectorySource>;
  };
  
  


} //end namespace nav_quadrotor

#endif //NAV_QUADROTOR_DETAILED_WAYPOINT_TRAJECTORY_SOURCE_H
  
