#ifndef NAV_QUADROTOR_INSCRIBED_SQUARE_FINDER_H
#define NAV_QUADROTOR_INSCRIBED_SQUARE_FINDER_H

#include <opencv2/core/mat.hpp>

namespace nav_quadrotor
{

//Based on https://blog.krybot.com/a?ID=01700-85f47bff-aec9-4dda-a7a8-b01456946566
bool getInscribedSquare( const cv::Mat &map_one_label, cv::Rect& biggest_rect);

} //end namespace nav_quadrotor

#endif //NAV_QUADROTOR_INSCRIBED_SQUARE_FINDER_H
