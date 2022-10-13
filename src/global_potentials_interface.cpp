#include <nav_quadrotor/global_potentials_interface.h>
#include <global_planner_plus/planner_core.h>
#include <costmap_2d/costmap_2d_ros.h>

namespace nav_quadrotor
{
  
  geometry_msgs::Pose transformPose(const tf::StampedTransform& transform, const geometry_msgs::Pose& pose_msg)
  {
    geometry_msgs::Pose transformed_pose_msg;
    
    tf::Pose pose_tf;
    tf::poseMsgToTF(pose_msg, pose_tf);
    tf::Pose transformed_pose_tf = transform * pose_tf;
    
    tf::poseTFToMsg(transformed_pose_tf, transformed_pose_msg);
    return transformed_pose_msg;
  }
  
//   geometry_msgs::Pose transformPose(const tf::StampedTransform& transform, const PoseSE2& pose)
//   {
//     geometry_msgs::Pose pose_msg;
//     pose.toPoseMsg(pose_msg);
//     
//     return transformPose(transform, pose_msg);
//   }
  
  void transformGradient(const tf::StampedTransform& transform, const Eigen::Vector2d& grad, Eigen::Vector2d& t)
  {
    tf::Vector3 v(grad.x(), grad.y(), 0);
    tf::Vector3 transformed = transform.getBasis().transpose() * v;
    
    t.x() = transformed.x();
    t.y() = transformed.y();
  }
  

  GlobalPotentialsInterface::GlobalPotentialsInterface(ros::NodeHandle nh):
    nh_(nh),
    potentials_sub_(),
    potentials_cache_(),
    costmap_(),
    inited_(false)
  {
  }
  
  
  
  void GlobalPotentialsInterface::init(std::string potentials_topic)
  {
    potentials_sub_.subscribe(nh_, potentials_topic, 1);
    potentials_sub_.registerCallback(&GlobalPotentialsInterface::potentialsCB, this);
    
    potentials_cache_.setCacheSize(1);
    potentials_cache_.connectInput(potentials_sub_);
    
    //planner_.initialize("potentials_planner", &lcr_);
    inited_=true;
  }
  
  bool GlobalPotentialsInterface::isInited() const
  {
    return inited_;
  }
  
  
//   void GlobalPotentialsInterface::initStandAlone(std::string planning_frame_id)
//   {
//     
//   }
  
  // Note: In order for this transform to be 'correct', the potentials must be received before the planning request comes through.
  bool GlobalPotentialsInterface::updateTransform(const tf::StampedTransform& transform)
  {
    map_to_global_transform_ = transform;
    tf::Transform inverted = transform.inverse();
    global_to_map_transform_ = tf::StampedTransform(inverted, transform.stamp_, transform.child_frame_id_, transform.frame_id_);
    
    {
      global_planner_plus::PotentialGrid::ConstPtr msg = potentials_cache_.getElemAfterTime(ros::Time());
      
      if(msg)
      {
        costmap_.update(msg);
        ROS_INFO_STREAM_NAMED(name_ + ".update","Updating costmap potentials");
        return true;
      }
      else
      {
        ROS_WARN_STREAM_NAMED(name_ + ".update","Warning, no potentials have been received!");
        return false;
      }
    }
  }
  
//   bool GlobalPotentialsInterface::updatePotentials(const PoseSE2& start, const PoseSE2& goal)
//   {
//     std::vector<geometry_msgs::Point> target_points;
//     
//     //TODO: make sure to use correct time to ensure consistency; really, perform this much earlier in teb process (if it isn't already) and make sure it is available everywhere that needs it
//     //tf_.lookupTransform(lcr_.getGlobalFrameID(), planning_frame_id_, ros::Time(0), global_to_map_transform_);
//     
//     geometry_msgs::Pose start_pose = transformPose(global_to_map_transform_, start);
//     geometry_msgs::Pose goal_pose = transformPose(global_to_map_transform_, goal);
//     
//     double tolerance=0; //As near as I can tell, it doesn't do anything anyway
//     
//     if(planner_.updatePotentials(start_pose, goal_pose, tolerance, target_points))
//     {
//       
//       //global_to_map_transform_;
//     
//       float pot = getPotential(goal);
//     
//       return pot > 0;
//     }
//     else
//     {
//       return false;
//     }
//   }

  void GlobalPotentialsInterface::potentialsCB(const global_planner_plus::PotentialGrid::ConstPtr& potentials_msg)
  {
    ROS_INFO_STREAM_NAMED(name_ + ".potentials_cb","Received new potentials [" << potentials_msg->header.stamp << "]");
  }
  
  float GlobalPotentialsInterface::getPotential(const geometry_msgs::Pose& pose) const
  {
    unsigned int neutral_cost = 50;
    geometry_msgs::Pose transformed_pose_msg = transformPose(global_to_map_transform_, pose);
    float potential = costmap_.getPointPotential(transformed_pose_msg.position);
    
    float scaled_potential;
    if(potential < 0 || potential == POT_HIGH)
    {
      scaled_potential = potential;
    }
    else
    {
      scaled_potential = potential * costmap_.getResolution() / neutral_cost;
    }
    ROS_DEBUG_STREAM_NAMED(name_ + ".get_potential","transformed pose: [" << transformed_pose_msg.position.x << "," << transformed_pose_msg.position.y << "] Potential = " << potential << ", Scaled Potential = " << scaled_potential);
    return scaled_potential;
  }
  
  bool GlobalPotentialsInterface::getGradient(const geometry_msgs::Pose& pose, Eigen::Vector2d& grad) const
  {
    geometry_msgs::Pose transformed_pose_msg = transformPose(global_to_map_transform_, pose);
    Eigen::Vector2d raw_grad;
    bool res = costmap_.getGradient(transformed_pose_msg.position, raw_grad);
    transformGradient(global_to_map_transform_, raw_grad, grad);
    ROS_DEBUG_STREAM_NAMED(name_ + ".get_gradient","transformed pose: [" << transformed_pose_msg.position.x << "," << transformed_pose_msg.position.y << "]" );
    ROS_DEBUG_STREAM_NAMED(name_ + ".get_gradient","Raw Gradient = [" << raw_grad.x() << "," << raw_grad.y() << "], Transformed Gradient=[" << grad.x() << "," << grad.y() << "]");
    return res;
  }
  
  bool GlobalPotentialsInterface::worldToGrid(const Eigen::Vector2d& world, Eigen::Vector2d& grid) const
  {
    geometry_msgs::Pose world_p;
    world_p.position.x = world.x();
    world_p.position.y = world.y();
    
    geometry_msgs::Pose map = transformPose(global_to_map_transform_, world_p);
    
    return costmap_.mapToGrid(map.position.x, map.position.y, grid.x(), grid.y());
  }
  
//   Eigen::Vector2d GlobalPotentialsInterface::gridToMap(const Eigen::Vector2d& grid_pos) const
//   {
//     
//   }
  
  
//   class GradientCalculator
//   {
//     GradientCalculator(const float* potentials, unsigned int xs, unsigned int ys, float* gradx, float* grady) :
//       potential_(potentials),
//       xs_(xs),
//       ys_(ys),
//       gradx_(gradx),
//       grady_(grady)
//     {
//       
//     }
//     
//     bool updateGradients()
//     {
//       unsigned int num_cells = xs_*xy_;
//       
//       for(unsigned int i = 0; i < num_cells; i++)
//       {
//         gradCell(i);
//       }
//     }
//     
//     bool getGradient(double px, double py)
//     {
//       float dx = px - (int)px;
//       float dy = py - (int)py;
//       
//       int stcnx = n + xs_;
//       int stcpx = n - xs_;
//         bool Costmap2DPotentials::mapToGrid(double wx, double wy, double& mx, double& my) const

//       // get interpolated gradient
//       float x1 = (1.0 - dx) * gradx_[stc] + dx * gradx_[stc + 1];
//       float x2 = (1.0 - dx) * gradx_[stcnx] + dx * gradx_[stcnx + 1];
//       float x = (1.0 - dy) * x1 + dy * x2; // interpolated x
//       float y1 = (1.0 - dx) * grady_[stc] + dx * grady_[stc + 1];
//       float y2 = (1.0 - dx) * grady_[stcnx] + dx * grady_[stcnx + 1];
//       float y = (1.0 - dy) * y1 + dy * y2; // interpolated y
//     }
//     
//     float gradCell(int n) {
//       if (gradx_[n] + grady_[n] > 0.0)    // check this cell
//         return 1.0;
//       
//       if (n < xs_ || n > xs_ * ys_ - xs_)    // would be out of bounds
//         return 0.0;
//       float cv = potential_[n];
//       float dx = 0.0;
//       float dy = 0.0;
//       
//       // check for in an obstacle
//       if (cv >= POT_HIGH) {
//         if (potential_[n - 1] < POT_HIGH)
//           dx = -lethal_cost_;
//         else if (potential_[n + 1] < POT_HIGH)
//           dx = lethal_cost_;
//         
//         if (potential_[n - xs_] < POT_HIGH)
//           dy = -lethal_cost_;
//         else if (potential_[n + xs_] < POT_HIGH)
//           dy = lethal_cost_;
//       }
//       
//       else                // not in an obstacle
//       {
//         // dx calc, average to sides
//         if (potential_[n - 1] < POT_HIGH)
//           dx += potential_[n - 1] - cv;
//         if (potential_[n + 1] < POT_HIGH)
//           dx += cv - potential_[n + 1];
//         
//         // dy calc, average to sides
//         if (potential_[n - xs_] < POT_HIGH)
//           dy += potential_[n - xs_] - cv;
//         if (potential_[n + xs_] < POT_HIGH)
//           dy += cv - potential_[n + xs_];
//       }
//       
//       // normalize
//       float norm = hypot(dx, dy);
//       if (norm > 0) {
//         norm = 1.0 / norm;
//         gradx_[n] = norm * dx;
//         grady_[n] = norm * dy;
//       }
//       return norm;
//     }
//     
//     unsigned int lethal_cost_=254;
//     
//     unsigned int xs_, ys_;
//     float* gradx_, grady_;
//     const float* potential_;
//   };
//   
  
  
  bool Costmap2DPotentials::update(const global_planner_plus::PotentialGrid::ConstPtr& potentials)
  {
    potentials_ = potentials;
    size_x_ = potentials_->info.width;
    size_y_ = potentials_->info.height;
    resolution_ = potentials_->info.resolution;
    origin_x_ = potentials_->info.origin.position.x;
    origin_y_ = potentials_->info.origin.position.y;
    convert_offset_ = 0.5;  //always 0.5 with 'new' planner, 0 if using 'old' navfn version
    
//     GradientCalculator gc(potentials_->data, size_x_, size_y_, gradx_.data(), grady_.data());
//     gc.updateGradients();
    
    updateGradients();
    
    return true;
  }
  
  // Copied from global_planner planner_core.cpp
  bool Costmap2DPotentials::mapToGrid(double wx, double wy, double& mx, double& my) const
  {
    double origin_x = getOriginX(), origin_y = getOriginY();
    double resolution = getResolution();
    
    if (wx < origin_x || wy < origin_y)
      return false;
    
    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;
    
    if (mx < getSizeInCellsX() && my < getSizeInCellsY())
      return true;
    
    return false;
  }
  
  Eigen::Vector2d Costmap2DPotentials::gridToMap(const Eigen::Vector2d& grid_pos) const
  {
    //wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
    //wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
    
    Eigen::Vector2d map_pos = Eigen::Array2d(getOriginX(), getOriginY()) + (grid_pos.array() + convert_offset_)*getResolution();
    return map_pos;
  }
    
  // TODO: Make sure handle bounds properly
  bool Costmap2DPotentials::interpolateValue(const std::vector<float>& values, const geometry_msgs::Point& world_point, float& result) const
  {
    double mx, my;
    if(!mapToGrid(world_point.x, world_point.y, mx, my))
      return false;
    
    unsigned int index = getIndex(mx, my);
    
    double dx = mx - (int)mx;
    double dy = my - (int)my;
        
    // get interpolated value
    float v1 = (1.0 - dx) * values[index] + dx * values[index + 1];
    float v2 = (1.0 - dx) * values[index + size_x_] + dx * values[index + size_x_ + 1];
    float v = (1.0 - dy) * v1 + dy * v2; // interpolated value
    
    result = v;
    
    return true;
  }
  
  float Costmap2DPotentials::getPointPotential(const geometry_msgs::Point& world_point) const
  {
    if(!potentials_)
    {
      ROS_ERROR_NAMED(name_ + ".get_point_potential","No potentials have been received yet!");
      return -1.0;
    }
    
    float val;
    if(interpolateValue(potentials_->data, world_point, val))
    {
      return val;
    }
    else
    {
      return -1;
    }
  }
  
  bool Costmap2DPotentials::getGradient(const geometry_msgs::Point& world_point, Eigen::Vector2d& grad) const
  {
    if(!potentials_)
    {
      ROS_ERROR_NAMED(name_ + ".get_gradient","No potentials have been received yet!");
      return false;
    }
    
    if(gradx_.size()==0 || grady_.size()==0)
    {
      ROS_ERROR_NAMED(name_ + ".get_gradient","Gradients do not exist! Make sure to call 'updateGradients' first!");
    }
    
    float x,y;
    if(!interpolateValue(gradx_, world_point, x))
    {
      return false;
    }
    if(!interpolateValue(grady_, world_point, y))
    {
      return false;
    }
    
    grad.x() = x;
    grad.y() = y;
    return true;
  }
  

  
  bool Costmap2DPotentials::updateGradients()
  {
    unsigned int num_cells = size_x_*size_y_;
    lethal_cost_ = 254; //TODO: Verify this!
    
    gradx_.clear();
    grady_.clear();
    gradx_.resize(num_cells);
    grady_.resize(num_cells);
    
    for(unsigned int i = 0; i < num_cells; i++)
    {
      gradCell(i);
    }
    
    return true;
  }
  

  float Costmap2DPotentials::gradCell(unsigned int n) 
  {
    if (gradx_[n] + grady_[n] > 0.0)    // check this cell
      return 1.0;
    
    if (n < size_x_ || n > size_x_ * size_y_ - size_x_)    // would be out of bounds
      return 0.0;
    float cv = potentials_->data[n];
    float dx = 0.0;
    float dy = 0.0;
    
    // check for in an obstacle
    if (cv >= POT_HIGH) {
      if (potentials_->data[n - 1] < POT_HIGH)
        dx = -lethal_cost_;
      else if (potentials_->data[n + 1] < POT_HIGH)
        dx = lethal_cost_;
      
      if (potentials_->data[n - size_x_] < POT_HIGH)
        dy = -lethal_cost_;
      else if (potentials_->data[n + size_x_] < POT_HIGH)
        dy = lethal_cost_;
    }
    
    else                // not in an obstacle
    {
      // dx calc, average to sides
      if (potentials_->data[n - 1] < POT_HIGH)
        dx += potentials_->data[n - 1] - cv;
      if (potentials_->data[n + 1] < POT_HIGH)
        dx += cv - potentials_->data[n + 1];
      
      // dy calc, average to sides
      if (potentials_->data[n - size_x_] < POT_HIGH)
        dy += potentials_->data[n - size_x_] - cv;
      if (potentials_->data[n + size_x_] < POT_HIGH)
        dy += cv - potentials_->data[n + size_x_];
    }
    
    // normalize
    float norm = hypot(dx, dy);
    if (norm > 0) {
      norm = 1.0 / norm;
      gradx_[n] = norm * dx;
      grady_[n] = norm * dy;
    }
    return norm;
  }

  
  
} //end namespace teb_local_planner

