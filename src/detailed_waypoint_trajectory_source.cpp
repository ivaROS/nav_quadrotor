#include <nav_quadrotor/detailed_waypoint_trajectory_source.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <nav_quadrotor/bent_straight_line_trajectory_function.h>
#include <unordered_map>
#include <stdexcept>
//#include <InscribedRectFinder.h>
#include <nav_quadrotor/inscribed_square_finder.h>
#include <sstream>

namespace nav_quadrotor
{    

  
    DetailedWaypointTrajectorySource::DetailedWaypointTrajectorySource(trajectory_based_nav::MultiLevelTrajectoryVerifier::Ptr verifier):
      verifier_(verifier)
    {}
        
    bool DetailedWaypointTrajectorySource::init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
    {
      if(!TrajectoryObjectTrajectorySource::init(nh, pnh, tfm))
      {
        return false;
      }
      
      fixed_frame_id_ = "world";
      //pips::utils::searchParam(ppnh, "fixed_frame_id", fixed_frame_id_, "world", -1);

      
      tfm_ = tfm;
      
      transformed_poses_pub_ = pnh.advertise<geometry_msgs::PoseArray>("transformed_candidate_waypoints", 2, true);
      resampled_poses_pub_ = pnh.advertise<geometry_msgs::PoseArray>("resampled_candidate_waypoints", 2, true);
      //end_poses_sub_.subscribe(pnh, "candidate_waypoints", 2);
      detailed_waypoints_sub_.subscribe(pnh, "detailed_candidate_waypoints", 2);
      
      
      //TODO: Parameterize frame_ids
      end_poses_filter_ = std::make_shared<tf_filter>(detailed_waypoints_sub_, *tfm.getBuffer(), fixed_frame_id_, 2, nh);
      end_poses_cache_ = std::make_shared<message_filters::Cache<WP_MSG> >(*end_poses_filter_, 1);
      
      return true;
    }
    
    
    DetailedWaypointTrajectorySource::WP_MSG::ConstPtr DetailedWaypointTrajectorySource::getTargetPoses(const std_msgs::Header& header, geometry_msgs::TransformStamped& transform)
    {
      ros::Time anytime;
      WP_MSG::ConstPtr msg = end_poses_cache_->getElemAfterTime(anytime);

      if(!msg)
      {
        ROS_WARN_STREAM("No candidate endpoints received on topic [" << detailed_waypoints_sub_.getTopic() << "]");
        return nullptr;
      }

      WP_MSG::Ptr transformed_pose_array;
      {
        //geometry_msgs::TransformStamped transform;
        try
        {
          transform = tfm_.getBuffer()->lookupTransform(header.frame_id, header.stamp, msg->header.frame_id, msg->header.stamp, fixed_frame_id_);
        }
        catch (tf2::TransformException &ex) 
        {
          ROS_WARN_STREAM("Problem finding transform:\n" <<ex.what());
          return nullptr;
        }
        
        ROS_DEBUG_STREAM_NAMED("waypoint_transform", "Transform:\n" << transform);
        
        transformed_pose_array = boost::make_shared<WP_MSG>();
        for(const auto& waypoint : msg->poses)
        {
          geometry_msgs::PoseStamped pose_stamped;
          pose_stamped.pose = waypoint;
          geometry_msgs::PoseStamped transformed_pose;
          tf2::doTransform(pose_stamped, transformed_pose, transform);
          ROS_DEBUG_STREAM_NAMED("waypoint_transform", "waypoint:\n" << waypoint << "\n\npose_stamped:\n" << pose_stamped << "\n\ntransformed_pose\n" << transformed_pose);
          
//           auto wp = waypoint;
//           wp.pose = transformed_pose;
          transformed_pose_array->poses.push_back(transformed_pose.pose);
        }
        
        {
          geometry_msgs::Point p;
          p.z=2;
          geometry_msgs::PoseStamped pose_stamped;
          pose_stamped.pose.position = p;
          pose_stamped.pose.orientation.w = 1;
          geometry_msgs::PoseStamped transformed_pose;
          tf2::doTransform(pose_stamped, transformed_pose, transform);
          transformed_pose_array->poses.push_back(transformed_pose.pose);
        }
        
        
        transformed_pose_array->properties = msg->properties;
        transformed_pose_array->header = header;
      }
      
      return transformed_pose_array;      
    }
    
    
    bool DetailedWaypointTrajectorySource::update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory)
    {
      if(!TrajectoryObjectTrajectorySource::update(odom, current_trajectory))
      {
        return false;
      }
      
      geometry_msgs::TransformStamped transform;
      auto target_waypoints = getTargetPoses(odom.header, transform);

      if(target_waypoints)
      {
        ROS_INFO_STREAM("target_pose_array:\n" << *target_waypoints);
        
        {
          geometry_msgs::PoseArray::Ptr pose_array_msg = boost::make_shared<geometry_msgs::PoseArray>();
          pose_array_msg->header = target_waypoints->header;
          pose_array_msg->poses = target_waypoints->poses;
//           for(const auto& wp : target_pose_array->poses)
//           {
//             pose_array_msg->poses.push_back(wp.pose.pose);
//           }
          
          transformed_poses_pub_.publish(pose_array_msg);
        }
        
        
        std::unordered_map<std::string, std::vector<float> > val_mapping;
        for(int i = 0; i < target_waypoints->properties.size(); ++i)
        {
          auto prop = target_waypoints->properties[i];
          val_mapping[prop.name] = prop.values;
        }
        
        const auto& op = odom.pose.pose.position;
        cv::Point3d p0(op.x, op.y, op.z);
        
        /*
        //Get basis vectors:
        geometry_msgs::Point cam_xvec, cam_yvec, world_xvec, world_yvec;
        cam_xvec.x=1;
        cam_yvec.y=1;
        
        tf2::doTransform(cam_xvec, world_xvec, transform);
        tf2::doTransform(cam_yvec, world_yvec, transform);

        cv::Point3d basis_x(world_xvec.x, world_xvec.y, world_xvec.z);
        cv::Point3d basis_y(world_yvec.x, world_yvec.y, world_yvec.z);*/
        
        double explore_rad = 0.2;
        int num_points = 4;
        double kf = explore_rad*2/num_points;
        
        //InscribedRectFinder rect_finder;
        //rect_finder.setMinArea(1);
        //rect_finder.setMaxArea((num_points+1)*(num_points+1));

        std::vector<geometry_msgs::Pose> resampled_poses, centered_poses;
        
        for(int i = 0; i < target_waypoints->poses.size(); ++i)
        {
          const auto& pose = target_waypoints->poses[i];
          const auto& pos = pose.position;
          cv::Point3d wp(pos.x, pos.y, pos.z);
          
          cv::Point3d v1 = wp - p0;
          v1/=cv::norm(v1);
          cv::Point3d v2(-v1.y/v1.x, 1, 0);
          v2/=cv::norm(v2);
          cv::Point3d v3= v1.cross(v2);
          v3/=cv::norm(v3);
          
          ///TODO: use freespace_dist to select the sample area
//           float freespace_dist=-1;
//           try 
//           {
//             freespace_dist = val_mapping["freespace_dist"][i];
//           }
//           catch(const std::out_of_range& e)
//           {
//             
//           }
          auto get_pnt = [wp, kf, v2, v3](double j, double k)
          {
            return wp + kf*(j*v2 + k*v3);
          };
          
          auto cv2pose = [](cv::Point3d dp)
          {
            geometry_msgs::Pose pm;
            pm.position.x = dp.x;
            pm.position.y = dp.y;
            pm.position.z = dp.z;
            pm.orientation.w = 1;
            return pm;
          };
          
          
          cv::Mat full_cc_res = cv::Mat::zeros(num_points+3,num_points+3,CV_8U);
          cv::Mat cc_res = full_cc_res(cv::Rect(1,1,num_points+1,num_points+1));
          //cv::Mat cc_res = cv::Mat::zeros(num_points+1,num_points+1,CV_8U);
          auto mark_safe = [&cc_res, num_points](int j, int k)
          {
            int nj = j+num_points/2;
            int nk = k+num_points/2;
            //ROS_INFO_STREAM("Marking " << nj << "," << nk);
            cc_res.at<uint8_t>(nj,nk) = 1;
          };
          
          for(int j=-num_points/2; j<=num_points/2; ++j)
          {
            for(int k=-num_points/2; k<=num_points/2; ++k)
            {
              cv::Point3d dp = get_pnt(j,k); //wp + kf*(j*v2 + k*v3);
              geometry_msgs::Pose pm = cv2pose(dp);
              
              bool safe, collided;
              verifier_->collisionCheckPose(pm, safe, collided);
              if(!collided)
              {
                mark_safe(j,k);
              }
              else
              {
                pm.orientation.x = 0.707;
                pm.orientation.w = 0.707;
              }
              resampled_poses.push_back(pm);
            }
          }
          //std::stringstream cc_res_str;
          //cc_res_str << cc_res;
          //ROS_INFO_STREAM("cc_res:\n" << cc_res_str.str());

          cv::Rect inscribed_rect; // = rect_finder.findRectangle(cc_res);
          bool status = getInscribedSquare(full_cc_res, inscribed_rect);
          
          if(status)
          {
            //ROS_INFO_STREAM("Inscribed Rect: " << inscribed_rect << "");
            
            if(inscribed_rect.width>0 && inscribed_rect.height>0)
            {
              double nj = inscribed_rect.y + inscribed_rect.height/2.0 - num_points/2;
              double nk = inscribed_rect.x + inscribed_rect.width/2.0 - num_points/2;
              
              auto center_pose = cv2pose(get_pnt(nj,nk));
              centered_poses.push_back(center_pose);
              //resampled_poses.push_back(center_pose);
            }
          }
          else
          {
            centered_poses.push_back(pose);
          }
        }
        
        
        
        {
          geometry_msgs::PoseArray::Ptr pose_array_msg = boost::make_shared<geometry_msgs::PoseArray>();
          pose_array_msg->header = target_waypoints->header;
          pose_array_msg->poses = resampled_poses;
//           for(const auto& wp : target_pose_array->poses)
//           {
//             pose_array_msg->poses.push_back(wp.pose.pose);
//           }
          
          resampled_poses_pub_.publish(pose_array_msg);
        }
        
        
        
        using namespace se2_z_trajectory;
      
        state_type x0 = getInitState(odom, current_trajectory);
        auto ni_params = getNIParams();
        
        
        
        int i = 0;
        //for(const auto waypoint : target_waypoints->poses)
        for(const auto end_pose : centered_poses) //centered_poses
        {
          double target_z_factor_ = 0.8;
          auto trajobj = getBentStraightLineTrajObject(x0, end_pose.position, fw_vel_, max_traj_duration_, target_z_factor_, ni_params);
          
          /*
          float freespace_dist=-1;
          try 
          {
            freespace_dist = val_mapping["freespace_dist"][i];
          }
          catch(const std::out_of_range& e)
          {
            
          }
          
          
          if(freespace_dist > 30)
          {
            auto f = [](BasicTrajectoryWrapper::Ptr& traj){traj->source_ += "_big_gap";};
            makeTrajectory(trajobj, odom.header, f);
          }
          else if(freespace_dist == 1)
          {
            //ignore 'gap'
          }
          else
            */
          {
            makeTrajectory(trajobj, odom.header);
          }
          
          ++i;
        }
        
        return true;
      }
      else
      {
        return false;
      }
    }


} //end namespace nav_quadrotor
