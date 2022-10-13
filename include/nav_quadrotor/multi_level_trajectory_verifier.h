#ifndef NAV_QUADROTOR_MULTI_LEVEL_TRAJECTORY_VERIFIER_H
#define NAV_QUADROTOR_MULTI_LEVEL_TRAJECTORY_VERIFIER_H

#include<trajectory_based_nav/interfaces.h>


namespace trajectory_based_nav
{
  class MultiLevelTrajectoryVerifier : public trajectory_based_nav::TrajectoryVerifier
  {
  protected:
    std::vector<std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> > cc_wrappers_;
    std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> conservative_wrapper_, liberal_wrapper_, detailed_wrapper_, egocan_wrapper1_, egocan_wrapper2_;
    
    ros::Duration min_safe_time_, min_colliding_time_;
    
    size_t num_cons_checks_=0, num_lib_checks_=0, num_det_checks_=0, num_egocan_checks_=0;
    
    std::string name_;
  
  public:
    
    MultiLevelTrajectoryVerifier(std::string name);
    
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm);

    virtual void setCCWrappers(std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> cons_wrapper, std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> lib_wrapper, std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> det_wrapper, std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> egocan_wrapper1, std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> egocan_wrapper2);
    
    virtual bool verifyTrajectory(TrajectoryWrapper::Ptr traj);

    virtual bool update(const nav_msgs::Odometry& odom);
    
    virtual void collisionCheckPose(const geometry_msgs::Pose& pose, bool& safe, bool& collided);
    
    virtual int collisionCheckTrajectory(TrajectoryWrapper::Ptr traj);

    using Ptr=std::shared_ptr<MultiLevelTrajectoryVerifier>;
  };

} //end namespace trajectory_based_nav
    
#endif  //NAV_QUADROTOR_MULTI_LEVEL_TRAJECTORY_VERIFIER_H
