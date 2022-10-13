#include <pips_trajectory_testing/pips_cc_wrapper.h>
#include <pips_egocylindrical/egocylindrical_image_collision_checker.h>
#include <pips/collision_testing/image_geometry_models/image_geometry_converter.h>
#include <nav_quadrotor/fastmarch_nearest_depth.h>

#include <sensor_msgs/Image.h>
//#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <tf2_utils/transform_manager.h>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/highgui.hpp>
#include <unordered_map>
#include <fastmarch/dt_helper.h>


class ImageConvolutionHelper
{
public:
  cv::Rect total_roi;
  cv::Mat raw_kernel_image;
  
};


class CCAccessor : public pips::collision_testing::EgocylindricalImageCollisionChecker
{
  //std::shared_ptr<PipsCollisionChecker> cc_;
  
public:
  
  CCAccessor(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME, const tf2_utils::TransformManager& tfm=tf2_utils::TransformManager(true)):
    pips::collision_testing::EgocylindricalImageCollisionChecker(nh, pnh, name, tfm)
    {
      PipsCollisionChecker::transpose_ = false;
    }
  
  
  virtual void initImpl()
  {
    pips::collision_testing::EgocylindricalImageCollisionChecker::initImpl();
    
    //any other setup? like getting ranges to check at
  }
  
  cv::Mat generateImage(geometry_msgs::Pose pose, cv::Rect& relevant_roi)
  {
    cv::Mat viz;
    if(std::numeric_limits<float>::has_quiet_NaN && false)
    {
      double dNaN = std::numeric_limits<float>::quiet_NaN();
      viz = cv::Mat(image_ref_.rows, image_ref_.cols, image_ref_.type(), cv::Scalar(dNaN));
    }
    else
    {
      viz = cv::Mat::zeros(image_ref_.rows, image_ref_.cols, image_ref_.type());
    }
    
    int img_width = input_bridge_ref_->image.cols;
    int img_height = input_bridge_ref_->image.rows;
    
    ROS_DEBUG_STREAM_NAMED(name_+".image_generation", "Parent image dimensions: [" << viz.cols << "x" << viz.rows << "], image_ref dimensions: [" << img_width << "x" << img_height << "]");
    
    const std_msgs::Header header = getCurrentHeader();
    
    
    ROS_DEBUG_STREAM_NAMED(name_+".image_generation","Pose: " << toString(pose));
    
    auto models = robot_model_.getModel<pips::collision_testing::image_geometry_models::ImageGeometryConverter>(pose);
    
    int m_id = 0;
    for(const auto& model : models)
    {
      std::vector<COLUMN_TYPE> cols = model->getColumns(cam_model_, img_width, img_height);
      
      for(unsigned int i = 0; i < cols.size(); ++i)
      {
        cv::Rect roi = getColumnRect(cols[i].rect);
        relevant_roi |= roi;
        cv::Mat col = cv::Mat(viz, roi); 
        float depth = cols[i].depth * scale_;
        //col.setTo(depth);
        col = cv::max(col,depth);
        ROS_DEBUG_STREAM_NAMED("freespace_accessor", "Model " << m_id << ", col " << i << ": " << cv::countNonZero(col>0) << "(" << cv::countNonZero(viz>0) << ")");
      }
      m_id++;
    }
    
    if(transpose_)
    {
      cv::Mat viz_t = viz.t();
      cv::Rect temp = relevant_roi;
      relevant_roi.x = temp.y;
      relevant_roi.y = temp.x;
      relevant_roi.height = temp.width;
      relevant_roi.width = temp.height;
      
      return viz_t;
    }
    
    return viz;
  }

  cv::Point2d getProjectionOrigin(const geometry_msgs::Pose& pose)
  {
    geometry_msgs::Pose transformed_pose;
    tf2::doTransform(pose, transformed_pose, base_optical_transform_);
    cv::Point3d point;
    convertPose(transformed_pose, point);
    cv::Point2d pixel = cam_model_->project3dToPixel(point);
    return pixel;
  }
  
  
  std::shared_ptr<pips::utils::AbstractCameraModel> getMyCameraModel()
  {
    return cam_model_;
  }
  
};


template <typename T>
T get_param(ros::NodeHandle& nh, std::string name, const T& default_val)
{
  T obj;
  if(!nh.getParam(name, obj))
  {
    obj = default_val;
    nh.setParam(name, obj);  
  }
  return obj;
}


//From https://stackoverflow.com/a/17820615
std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

void describeMat(std::string name, cv::Mat mat)
{
  ROS_INFO_STREAM(name << " has type: " << type2str(mat.type()));
}

class ImshowHelper
{
  bool show_im_, publish_im_;
  std_msgs::Header header_;
  ros::NodeHandle pnh_;
  
  class ImshowHelperInstance
  {
    ros::Publisher pub_;
    const std_msgs::Header& header_;
    std::string encoding_;

  public:
    ImshowHelperInstance(ros::Publisher pub, const std_msgs::Header& header, std::string encoding):
      pub_(pub),
      header_(header),
      encoding_(encoding)
    {
      
    }
    
    void publish(cv::Mat img)
    {
      cv_bridge::CvImage out_msg;
      out_msg.header = header_;
      out_msg.encoding = encoding_;
      out_msg.image = img;
      pub_.publish(out_msg.toImageMsg());
    }
  };
  
  std::unordered_map<std::string, ImshowHelperInstance> helpers_;
  
public:
  
  bool init(ros::NodeHandle nh, ros::NodeHandle pnh)
  {
    pnh_ = pnh;
    show_im_ = get_param(pnh, "display_visuals", false);
    publish_im_ = get_param(pnh, "publish_visuals", true);
    return true;
  }
  
  bool isActive()
  {
    return show_im_ || publish_im_;
  }
  
  void imshow(const std::string& name, cv::Mat mat, const std::string& encoding)
  {
    describeMat(name, mat);

    if(show_im_)
    {
      cv::imshow(name, mat);
    }
    
    if(publish_im_)
    {
      auto* helper = getHelper(name, encoding);
      if(helper)
      {
        helper->publish(mat);
      }
    }
  }
  
  ImshowHelperInstance* getHelper(const std::string& name, const std::string& encoding)
  {    
    ImshowHelperInstance* helper=nullptr;
    {
      auto it = helpers_.find(name);
      if( it != helpers_.end() )
      {
        helper = &(it->second);
      }
      else
      {
        std::string topic = name;
        std::replace( topic.begin(), topic.end(), '.', '_');
        auto pub = pnh_.advertise<sensor_msgs::Image>(topic, 2, true);
        auto res = helpers_.emplace(std::piecewise_construct,
              std::forward_as_tuple(name),
              std::forward_as_tuple(pub, header_, encoding));
        auto it = res.first;
        helper = &(it->second);
      }
    }
    return helper;
  }
  
  void preUpdate(const std_msgs::Header& header)
  {
    header_ = header;
  }
  
  void postUpdate()
  {
    if(show_im_)
    {
      cv::waitKey(1);
    }
  }
};

class FreespaceEstimator //: public pips_trajectory_testing::PipsCCWrapper
{
  ros::NodeHandle nh_, pnh_;
  tf2_utils::TransformManager tfm_;
  
  CCAccessor accessor_;
  nav_quadrotor::FastmarchNearestDepth fnd_;

  std::vector<float> analysis_ranges_;
  std::string name_;
  
  ImshowHelper imshow_;
  bool display_visuals_;
  
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, egocylindrical::EgoCylinderPoints> exact_image_sync_policy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, egocylindrical::EgoCylinderPoints> approx_image_sync_policy;
  typedef message_filters::Synchronizer<approx_image_sync_policy> time_synchronizer;
  
  typedef tf2_ros::MessageFilter<egocylindrical::EgoCylinderPoints> tf_filter;
  //typedef pips_trajectory_testing::PipsCCWrapper Super;
  
  boost::shared_ptr<tf_filter> info_tf_filter_;
  boost::shared_ptr<time_synchronizer> image_synchronizer_;
  
  image_transport::SubscriberFilter ec_sub_;
  message_filters::Subscriber<egocylindrical::EgoCylinderPoints> ec_info_sub_;
  
  ros::Publisher conv_im_pub_, nd_im_pub_;
  
  sensor_msgs::Image::ConstPtr current_image;
  egocylindrical::EgoCylinderPoints::ConstPtr current_camInfo;
  
  
public:
  
  FreespaceEstimator(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name, const tf2_utils::TransformManager& tfm):
    nh_(nh),
    pnh_(pnh, name),
    tfm_(tfm),
    accessor_(nh_, pnh_, "egocylindrical_image_collision_checker", tfm_),
    imshow_()
  {
  }
  
  bool init()
  {    
    
    std::string temp;
    if(pnh_.getParam("egocylindrical_image_collision_checker/robot_model/param_name", temp))
    {
      ROS_INFO_STREAM("Found parameter name: " << temp);
    }
    else
    {
      temp = "/simplified_robot_description";
      pnh_.setParam("egocylindrical_image_collision_checker/robot_model/param_name", temp);
    }
    
    //display_visuals_ = get_param(pnh_, "display_visuals", true);
//     display_visuals_ = true;
    
    accessor_.init();
    geometry_msgs::TransformStamped transform;
    transform.transform.rotation.w=1;
    accessor_.setTransform(transform);
    
    std::string depth_image_topic="egocylinder/image", depth_info_topic= "/egocylinder/egocylinder_info";
    
    pnh_.getParam("egocylindrical_image_topic", depth_image_topic );
    pnh_.getParam("egocylindrical_info_topic", depth_info_topic );
    
    // The idea here is to set the parameter on the parameter server to the default value to make it easier to see what it is.
    pnh_.setParam("egocylindrical_image_topic", depth_image_topic);
    pnh_.setParam("egocylindrical_info_topic", depth_info_topic );
    
    
    analysis_ranges_ = {1,2,3};
    
    if(pnh_.getParam("analysis_ranges", analysis_ranges_))
    {
      ROS_INFO_STREAM("Loaded analysis ranges from parameter server");
    }
    
    // TODO: use parameters for base_frame_id and odom_frame_id
    
    ROS_DEBUG_STREAM_NAMED ( name_,  "Setting up publishers and subscribers" );
    
    image_transport::ImageTransport it ( nh_ );
    
    //pub_trajectoryProjection = it.advertise ( "trajectoryProjection", 1 );
    
    conv_im_pub_ = pnh_.advertise<sensor_msgs::Image>("convolved_projection", 1);
    nd_im_pub_ = pnh_.advertise<sensor_msgs::Image>("nearest_depth_im", 1);
    
    imshow_.init(nh_, pnh_);
    display_visuals_ = imshow_.isActive();

    auto transport_hints = ros::TransportHints().tcpNoDelay(true);
    ec_sub_.subscribe(it, depth_image_topic, 2, image_transport::TransportHints("raw", transport_hints));
    ec_info_sub_.subscribe(nh_, depth_info_topic, 2, transport_hints);
    
    ec_sub_.registerCallback([this](const sensor_msgs::Image::ConstPtr& image_msg){ROS_DEBUG_STREAM_NAMED("msg_timestamps.detailed", "[" << name_ << "] Received image msg [" << image_msg->header.stamp << "] at [" << ros::WallTime::now() << "]");});
    ec_info_sub_.registerCallback([this](const egocylindrical::EgoCylinderPoints::ConstPtr& info_msg){ROS_DEBUG_STREAM_NAMED("msg_timestamps.detailed", "[" << name_ << "] Received info msg [" << info_msg->header.stamp << "] at [" << ros::WallTime::now() << "]");});
    
    // Ensure that CameraInfo is transformable
    info_tf_filter_ = boost::make_shared<tf_filter>(ec_info_sub_, *tfm_.getBuffer(), "world", 2,nh_);
    
    // Synchronize Image and CameraInfo callbacks
    image_synchronizer_ = boost::make_shared<time_synchronizer>(time_synchronizer(10),ec_sub_, *info_tf_filter_);
    image_synchronizer_->registerCallback(boost::bind(&FreespaceEstimator::ecImageCb, this, _1, _2));
    
    return true;
    
  }
  
//   virtual std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> getCC()
//   {
//     return accessor_;
//   }

  
  void ecImageCb (const sensor_msgs::Image::ConstPtr& image_msg, const egocylindrical::EgoCylinderPoints::ConstPtr& info_msg)
  {
    ROS_DEBUG_STREAM_NAMED("msg_timestamps.detailed", "[" << name_ << "] Received syncronized messages [" << image_msg->header.stamp << "] at [" << ros::WallTime::now() << "]");
    ROS_DEBUG_STREAM_NAMED(name_ + ".image_callback", "Received synchronized messages with image stamp " << image_msg->header.stamp << " and info stamp " << info_msg->header.stamp);
    if ( image_msg->header.stamp == ros::Time ( 0 ) || info_msg->header.stamp == ros::Time ( 0 ) ) { // Gazebo occasionally publishes Image and CameraInfo messages with time=0
      ROS_WARN_STREAM_NAMED ( name_,"Bad timestamp" );
      return;
    }
    
    //TODO: Add mutex here
    current_image = image_msg;
    current_camInfo = info_msg;
    
    update();
  }
  
  bool update()
  {
    accessor_.setImage ( current_image, current_camInfo );
    
    ros::Duration timeout ( 0 ); //Could be used for transform lookup?
    
    auto start_time = ros::WallTime::now();
    
    auto target_header = current_image->header;
    std_msgs::Header source_header;
    source_header.frame_id = "hummingbird/base_link";
    source_header.stamp = target_header.stamp;
    
    imshow_.preUpdate(current_image->header);
    
    ROS_DEBUG_STREAM_ONCE_NAMED ( name_, "Not ready, check for transform..." );
    try {
      //Get the transform that takes a point in the base frame and transforms it to the depth optical
      geometry_msgs::TransformStamped sensor_base_transform = tfm_.getBuffer()->lookupTransform ( target_header.frame_id, target_header.stamp, source_header.frame_id, source_header.stamp, "world", timeout);
      accessor_.setTransform ( sensor_base_transform );
      
      accessor_.init();
      
      ROS_DEBUG_STREAM_NAMED(name_+".transform", "transform: from [" << target_header.frame_id << "] at " << target_header.stamp << " to [" << source_header.frame_id << "] at " << source_header.stamp << ": " << toString(sensor_base_transform));
      
      
      ROS_DEBUG_STREAM_NAMED ( name_,  "Transform found! Passing transform to collision checker" );
      
    } catch ( tf2::TransformException &ex ) {
      ROS_WARN_STREAM_NAMED (name_, "Problem finding transform:\n" <<ex.what() );
      return false;
    }
    
    auto image_ref = cv_bridge::toCvShare(current_image);
    cv::Mat raw_source_img = image_ref->image;
    auto img_encoding = current_image->encoding;
    
    std::string label;

    if(display_visuals_)
    {
      label = "raw_source_img";
      //cv::imshow(label, raw_source_img);
      //describeMat(label, raw_source_img);
      imshow_.imshow(label, raw_source_img, img_encoding);
    }

    
    //
    std::vector<cv::Mat> thresh_imgs, bin_thresh_imgs, raw_gen_imgs, roi_gen_imgs, roi_gen_thresh_imgs, raw_conv_imgs, accum_conv_imgs;
    for(float range : analysis_ranges_)
    {
      geometry_msgs::Pose pose;
      pose.orientation.w = 1;
      pose.position.x = range;
      //pose.position.z = 0.5;
      
      cv::Rect roi;
      cv::Mat gen_img = accessor_.generateImage(pose, roi);
      cv::patchNaNs(gen_img, 0);
      raw_conv_imgs.push_back(gen_img);
      
      cv::Point origin_pt = accessor_.getProjectionOrigin(pose);
      
      if(display_visuals_)
      {
        label = "generated" + std::to_string(range);
        //cv::imshow(label, gen_img);
        //describeMat(label, gen_img);
        imshow_.imshow(label, gen_img, img_encoding);
      }

      
      cv::Mat roi_im(gen_img, roi);
      roi_gen_imgs.push_back(roi_im);
      if(roi_im.cols>0 && roi_im.rows>0)
      {
        cv::Point anchort = origin_pt - roi.tl();
        
        double front_depth;
        cv::minMaxIdx(roi_im, nullptr, &front_depth);
        ROS_INFO_STREAM("Range=" << range << ", Front_depth=" << front_depth);
        
        if(display_visuals_)
        {
          label = "generated_roi" + std::to_string(range);
          //cv::imshow(label, roi_im);
          //describeMat(label, roi_im);
          imshow_.imshow(label, roi_im, img_encoding);
        }
        //cv::Mat roi_thresholded = roi_im==roi_im; //This SHOULD give us 1's wherever the original image is not NaN, but currently broken 
        cv::Mat roi_thresholded = roi_im>0;
        
        int count = cv::countNonZero(roi_thresholded);
        ROS_INFO_STREAM("Count=" << count);
        roi_gen_thresh_imgs.push_back(roi_thresholded);
        if(display_visuals_)
        {
          label = "thresholded_roi" + std::to_string(range);
          cv::Mat viz_roi_thresh = roi_thresholded.clone();
          cv::cvtColor(viz_roi_thresh, viz_roi_thresh, cv::COLOR_GRAY2BGR);
          cv::circle( viz_roi_thresh,
              anchort,
              3,
              cv::Scalar(0,0,255),
              cv::FILLED,
              cv::LINE_8 );
          //cv::imshow(label, viz_roi_thresh);
          //describeMat(label, roi_thresholded);
          imshow_.imshow(label, viz_roi_thresh, "bgr8");
        }
        
        cv::Mat thresholded;
        cv::threshold(raw_source_img, thresholded, front_depth, front_depth, cv::THRESH_TOZERO_INV);
        thresh_imgs.push_back(thresholded);
        if(display_visuals_)
        {
          label = "thresholded" + std::to_string(range);
          cv::Mat thresholded_viz;
          cv::normalize(thresholded, thresholded_viz, 1, 0, cv::NORM_MINMAX);
          //cv::imshow(label, thresholded_viz);
          //escribeMat(label, thresholded);
          imshow_.imshow(label, thresholded, img_encoding);
        }
        
        //get binary thresholded image
        cv::Mat bin_thresholded = thresholded > 0;
        bin_thresh_imgs.push_back(bin_thresholded);
        if(display_visuals_)
        {
          label = "bin_thresholded" + std::to_string(range);
          //cv::imshow(label, bin_thresholded);
          //describeMat(label, bin_thresholded);
          imshow_.imshow(label, bin_thresholded, "mono8");
        }
        
        //cv::Mat convolved = getConvolvedImage(roi_im, thresholded);
        int ddepth = -1;
        cv::Mat convolved;
        cv::Point anchor = anchort;  //can use this to establish offset so that value at a pixel corresponds to result for robot origin at that point
        cv::Mat kernel = roi_thresholded;
        double delta = 0;
        int border_type = cv::BORDER_ISOLATED;
        ros::WallTime start = ros::WallTime::now();
        cv::filter2D(bin_thresholded, convolved, ddepth, kernel, anchor, delta, border_type);
        ROS_INFO_STREAM("Convolution time for depth " << range << " is " << (ros::WallTime::now() - start).toSec()*1e3 << "ms");
        raw_conv_imgs.push_back(convolved);
        if(display_visuals_)
        {
          label = "convolved" + std::to_string(range);
          //cv::imshow(label, convolved);
          //describeMat(label, convolved);
          imshow_.imshow(label, convolved, "mono8");
        }
        
        cv::Mat accum_conv;
        if(accum_conv_imgs.size()>0)
        {
          accum_conv = convolved | accum_conv_imgs.back();
        }
        else
        {
          accum_conv = convolved;
        }
        
        accum_conv_imgs.push_back(accum_conv);
        if(display_visuals_)
        {
          label = "accum_conv" + std::to_string(range);
          //cv::imshow(label, accum_conv);
          //describeMat(label, accum_conv);
          imshow_.imshow(label, accum_conv, "mono8");
        }
      }
      //
    }
    
    if(accum_conv_imgs.size()>0)
    {
      cv_bridge::CvImage out_msg;
      out_msg.header = current_image->header; // Same timestamp and tf frame as input image
      out_msg.encoding = "mono8";
      out_msg.image = accum_conv_imgs.back();
      conv_im_pub_.publish(out_msg.toImageMsg());
      
      
      ros::WallTime start = ros::WallTime::now();
      fastmarching::DTHelper dth;
      cv::Mat result = dth.run(accum_conv_imgs.back());
      ROS_INFO_STREAM("Fastmarch time: " << (ros::WallTime::now() - start).toSec()*1000 << "ms");

    }
    
    
    if(thresh_imgs.size()>0)
    {
      label= "nearest_depths";
      ros::WallTime nd_start = ros::WallTime::now();
      cv::Mat last_thresh_img = thresh_imgs.back();
      cv::Mat nd_img = fnd_.processImage(last_thresh_img);
      
      cv_bridge::CvImage out_msg;
      out_msg.header = current_image->header; // Same timestamp and tf frame as input image
      out_msg.encoding = "32FC1";
      out_msg.image = nd_img;
      nd_im_pub_.publish(out_msg.toImageMsg());
      
      imshow_.imshow(label, nd_img, "32FC1");

      
      ROS_INFO_STREAM("Nearest Depths processing time: " << (ros::WallTime::now() - nd_start).toSec()*1000 << "ms");
    }
    
    
    
    auto end_time = ros::WallTime::now();
    ROS_INFO_STREAM("Total processing time: " << (end_time - start_time).toSec()*1000 << "ms");
    
    imshow_.postUpdate();
    
//     if(display_visuals_)
//     {
//       cv::waitKey(1);
//     }
    return true;
  }
  
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "freespace_estimator_node");
    
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tf2_utils::TransformManager tfm(true);
  FreespaceEstimator s(nh, pnh, "freespace_estimator", tfm);
  s.init();

  ros::spin();
}
