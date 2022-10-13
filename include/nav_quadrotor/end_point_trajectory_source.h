#ifndef NAV_QUADROTOR_END_POINT_TRAJECTORY_SOURCE_H
#define NAV_QUADROTOR_END_POINT_TRAJECTORY_SOURCE_H

#include <nav_quadrotor/new_trajectory_sources.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>

namespace nav_quadrotor
{
  
  class EndPointTrajectorySource : public TrajectoryObjectTrajectorySource
  {
  protected:
    std::string fixed_frame_id_;    
    tf2_utils::TransformManager tfm_;
    
    message_filters::Subscriber<geometry_msgs::PoseArray> end_poses_sub_;
    ros::Publisher transformed_poses_pub_;
    
    using tf_filter = tf2_ros::MessageFilter<geometry_msgs::PoseArray>;
    std::shared_ptr<tf_filter> end_poses_filter_;
    std::shared_ptr<message_filters::Cache<geometry_msgs::PoseArray> > end_poses_cache_;
    
  public:
    
    EndPointTrajectorySource();
        
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm);

    virtual geometry_msgs::PoseArray::ConstPtr getTargetPoses(const std_msgs::Header& header);
    
    virtual bool update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory);

    
    using Ptr=std::shared_ptr<EndPointTrajectorySource>;
  };
  
  


} //end namespace nav_quadrotor

#endif //NAV_QUADROTOR_END_POINT_TRAJECTORY_SOURCE_H
  
