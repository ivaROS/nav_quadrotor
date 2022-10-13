#include <nav_quadrotor/inscribed_square_finder.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>  //For 'findContours', 'pointPolygonTest', 
#include <opencv2/core.hpp>  //For 'minMaxLoc()'

namespace nav_quadrotor
{
  
//using namespace cv;
//using namespace std;

//using Rect = cv::Rect;
//using Mat = cv::Mat;
//using Point = cv::Point;
//Based on https://blog.krybot.com/a?ID=01700-85f47bff-aec9-4dda-a7a8-b01456946566
bool getInscribedSquare(const cv::Mat &map_one_label, cv::Rect& biggest_rect)
{
    std::vector<cv::Rect> results;
    int biggest_radius = 0;
    cv::Point center(map_one_label.cols / 2, map_one_label.rows / 2);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(map_one_label, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (int i = 0; i < contours.size(); i++)
    {
        cv::Mat  dist = cv::Mat::zeros(map_one_label.size(), CV_32F);		//Define a Mat object to store the distance from each point in the original image to the contour, as floating point data 
        //Traverse each point and calculate the distance from the point to the contour 
        for (int row = 0; row < map_one_label.rows; row++)
        {
            for (int col = 0; col < map_one_label.cols; col++)
            {
                //Calculate the point-to-contour distance through point polygon detection, and Store in
                dist.at<float>(row, col) = cv::pointPolygonTest(contours[i], cv::Point(col, row), true);
            }
        }

        //Calculate the maximum and minimum values ​​in dist, and its position coordinates
        double minVal, maxVal;
        cv::Point maxloc, minloc;
        cv::minMaxLoc(dist, &minVal, &maxVal, &minloc, &maxloc);
        int radius = abs(maxVal);			//Find the absolute value of the maximum value, which is the inscribed circle radius
        if(radius > biggest_radius)
        {
          cv::Point center = maxloc;	//a point of maximum distance from the contour, then the point is a center of the circle inscribed
          cv::Point point_tl(center.x-radius , center.y-radius);
          cv::Point point_dr(center.x+radius , center.y+radius);
          biggest_rect = cv::Rect(point_dr , point_tl );
        }  
    }
    //imshow("src", map_one_label);
    //cv::waitKey(0);
    return biggest_radius>0;
}


} //end namespace nav_quadrotor
