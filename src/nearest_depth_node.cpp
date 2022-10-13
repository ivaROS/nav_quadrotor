#include <nav_quadrotor/fastmarch_nearest_depth.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

namespace nav_quadrotor
{
    
  class NearestDepthImageProcessing
  {
    FastmarchNearestDepth fnd_;
    ros::Publisher im_pub_;
    ros::Subscriber im_sub_;
    
    
  public:
    NearestDepthImageProcessing():
      fnd_()
    {
      
    }
    
    bool init(ros::NodeHandle nh, ros::NodeHandle pnh)
    {
      im_pub_ = pnh.advertise<sensor_msgs::Image>("image_out", 1, true);
      im_sub_ = pnh.subscribe("image_in", 1, &NearestDepthImageProcessing::imageCB, this);
      
      return true;
    }
    
    void imageCB(const sensor_msgs::Image::ConstPtr& image_msg)
    {
      ROS_DEBUG_STREAM_NAMED("nearest_depth_node","Received new image!");
      
      auto image_ref = cv_bridge::toCvShare(image_msg);
      cv::Mat img = image_ref->image.clone();
      
      img.setTo(std::numeric_limits<float>::quiet_NaN(), img>2);
      cv::patchNaNs(img, 0);
      cv::imshow("thresholded", img);
      
      cv::Mat processed_im = fnd_.processImage(img);
      cv::imshow("processed_im", processed_im);
      
      cv_bridge::CvImage out_cv_im;
      out_cv_im.header = image_msg->header; // Same timestamp and tf frame as input image
      out_cv_im.encoding = image_msg->encoding;
      out_cv_im.image = processed_im;
      auto out_msg = out_cv_im.toImageMsg();
      im_pub_.publish(out_msg);
      cv::waitKey(1);
    }
    
    
  };

}

int main(int argc, char **argv)
{
    std::string name= "nearest_depth_node";
    ros::init(argc, argv, name);
    ros::start();
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    nav_quadrotor::NearestDepthImageProcessing processor;
    processor.init(nh, pnh);

    ros::spin();

	return 0;
}
