#include <fastmarch/fastmarch_helper.h>
#include <cstdint>
#include <limits>

namespace nav_quadrotor
{

  class FastmarchNearestDepth
  {
    fastmarching::FastMarchHelper fmh_;
    
  public:
    FastmarchNearestDepth();
    
    cv::Mat processImage(cv::Mat image);

  };

}
