#ifndef CONVEYOR_SCANNING__BACKGROUND_FILTER_H
#define CONVEYOR_SCANNING__BACKGROUND_FILTER_H

//#include <opencv2/core/core.h>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include "../include/pcl_types.h"

namespace Conveyor_scanning {

class Background_filter
{
    public:
        Background_filter ( const int );
        ~Background_filter ();

        void image_update_foreground_mask (const cv::Mat& );
        void update_foreground_mask ( const Cloud& );
        void get_foreground_indices ( Indices_ptr ) const;
        void write_images ( const Cloud& ) const;
        void image_write_images ( const cv::Mat& ) const;

    private:
        cv::BackgroundSubtractor* _bgs;

        cv::Size _size;
        cv::Mat _bgr_threshold;
        cv::Mat _morph_element;
        cv::Mat _foreground_mask;
};//class Background_filter

}//namespace Conveyor_object_tracking

#endif//CONVEYOR_SCANNING__BACKGROUND_FILTER_H
