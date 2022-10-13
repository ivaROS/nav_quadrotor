#include <nav_quadrotor/fastmarch_nearest_depth.h>

namespace nav_quadrotor
{

  FastmarchNearestDepth::FastmarchNearestDepth():
    fmh_()
  {
    
  }
  
  cv::Mat FastmarchNearestDepth::processImage(cv::Mat image)
  {
    /* create empty labels image of same size full of 0's.
    * create array reference to original image and labels
    * iterate through image, incrementing counter w/ each pixel (1 indexed)
    *  if pixel value is not 0 and not NaN: set labels value to counter
    * 
    * After expanding labels:
    * create new depths image w/ all NaNs
    * iterate through labels image
    *  for each label:
    *    if value is not 0: 
    *      get depth from original image[label-1]
    * 
    * 
    * 
    * 
    * For u16 type: could just use the raw depth values as labels
    * 
    */
    if(!image.isContinuous())
    {
      image = image.clone();
    }
    
    float* im_arr = (float*)image.data;
    
    int rows=image.rows;
    int cols=image.cols;
    int num_pts = rows*cols;
    
    //Fill in labels matrix
    cv::Mat labels = cv::Mat::zeros(rows, cols, CV_32S);
    {
      auto* labels_arr = (int32_t*)labels.data;
      
      for(int i = 0; i < num_pts; i++)
      {
        auto im_val = im_arr[i];
        auto& label = labels_arr[i];
        if(im_val!=0)
        {
          label = i+1;
        }
      }
    }
    
    //Fastmarch labels
    cv::Mat result_labels = fmh_.labelMarch(labels);
    fmh_.showLabels();
    
    
    //Fill in depths based on fastmarched labels
    cv::Mat result_depths(rows, cols, CV_32F, std::numeric_limits<float>::quiet_NaN());
    {
      auto* labels_arr = (int32_t*)result_labels.data;
      auto* result_depths_arr = (float*)result_depths.data;
      
      for(int i = 0; i < num_pts; i++)
      {
        auto im_val = im_arr[i];
        //if(im_val == 0)
        {
          auto label_val = labels_arr[i];
          auto& result_depth = result_depths_arr[i];
          if(label_val>0)
          {
            result_depth = im_arr[label_val-1];
          }
        }
      }
    }
    
    return result_depths;
  }

} //end namespace nav_quadrotor
