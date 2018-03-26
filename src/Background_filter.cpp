#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include "../include/Background_filter.h"

#define MIN_DIST (0.9)
#define MAX_DIST (1.35)

namespace Conveyor_scanning {

Background_filter::Background_filter ( const int s )
    : _size( cv::Size( s,s ) )
{
    _bgs = new cv::BackgroundSubtractorMOG2();
    _bgr_threshold;
    _morph_element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( s+2, s+2) );
}//Background_filter::Background_filter

Background_filter::~Background_filter ()
{
    delete _bgs;
}//Background_filter::~Background_filter


void Background_filter::update_foreground_mask ( const Cloud& input_cloud ) {
    if( !input_cloud.isOrganized() || input_cloud.size() == 0 ) return;
    cv::Mat bgr_image( input_cloud.height, input_cloud.width, CV_8UC3, cv::Scalar::all( 128 ) );
    if (_bgr_threshold.empty() ) {
    	_bgr_threshold = bgr_image;
    }

    for( size_t x = 8; x < input_cloud.width - 8; x++ ){
        for( size_t y = 32; y < input_cloud.height - 48; y++ ){
            Point p = input_cloud.at( x,y );
            if( !std::isnan( p.z ) && (p.z >= MIN_DIST) && (p.z <= MAX_DIST) ) {
            	bgr_image.at<cv::Vec3b>( y,x ) = cv::Vec3b( p.b, p.g, p.r );
            } else if (std::isnan( p.z ) && (_bgr_threshold.at<cv::Vec3b>( y,x ) != cv::Vec3b( 128, 128, 128 ) ) ) {
            	bgr_image.at<cv::Vec3b>( y,x ) = _bgr_threshold.at<cv::Vec3b>( y,x );
            }
        }
    }
    _bgr_threshold = bgr_image.clone();

    cv::GaussianBlur( bgr_image, bgr_image, _size, 0, 0 );

    (*_bgs)( bgr_image, _foreground_mask );
}//Background_filter::update_foreground_mask

void Background_filter::image_update_foreground_mask ( const cv::Mat& input_cv )
{
	(*_bgs)( input_cv, _foreground_mask );
}

void Background_filter::get_foreground_indices ( Indices_ptr output_indices ) const
{
    cv::Mat filtered_mask;

    cv::morphologyEx( _foreground_mask, filtered_mask, cv::MORPH_OPEN, _morph_element );
    cv::morphologyEx( filtered_mask, filtered_mask, cv::MORPH_CLOSE, _morph_element );

    Indices foreground_points;

    for( size_t x = 0; x < filtered_mask.cols; x++ ){
        for( size_t y = 0; y < filtered_mask.rows; y++ ){
     //     if( filtered_mask.at<uint8_t>( y,x ) > 128 ){
            if( filtered_mask.at<uint8_t>( y,x ) > 20 ) {
                foreground_points.push_back( x + y*filtered_mask.cols );
            }
        }
    }

    *output_indices = foreground_points;
}//Background_filter::get_foreground_indices

void Background_filter::write_images ( const Cloud& input_cloud ) const
{
    cv::Mat bgr_image( input_cloud.height, input_cloud.width, CV_8UC3, cv::Scalar::all( 128 ) );

    for( size_t x = 0; x < input_cloud.width; x++ ){
        for( size_t y = 0; y < input_cloud.height; y++ ){
            Point p = input_cloud.at( x,y );
            if( !std::isnan( p.z ) ){
                bgr_image.at<cv::Vec3b>( y,x ) = cv::Vec3b( p.b, p.g, p.r );
            } else {
            	bgr_image.at<cv::Vec3b>( y,x ) = _bgr_threshold.at<cv::Vec3b>( y,x );
            }
        }
    }
    
    cv::Mat filtered_mask;

    cv::morphologyEx( _foreground_mask, filtered_mask, cv::MORPH_OPEN, _morph_element );
    cv::morphologyEx( filtered_mask, filtered_mask, cv::MORPH_CLOSE, _morph_element );

    cv::Mat foreground_image = bgr_image.clone();
    
    for( size_t x = 0; x < filtered_mask.cols; x++ ){
        for( size_t y = 0; y < filtered_mask.rows; y++ ){
            if( filtered_mask.at<uint8_t>( y,x ) > 128 ){
		foreground_image.at<cv::Vec3b>( y,x ) = cv::Vec3b( 255, 0, 0 );
            }
        }
    }

    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    cv::imwrite( "conveyor_scene.png", bgr_image, compression_params );
    cv::imwrite( "conveyor_fgmask.png", filtered_mask, compression_params );
    cv::imwrite( "conveyor_fgscene.png", foreground_image, compression_params );

    cv::imwrite( "conveyor_threshold.png", _bgr_threshold, compression_params );
}//Background_filter::write_images

void Background_filter::image_write_images ( const cv::Mat& input_cv ) const
{
    cv::Mat filtered_mask;

    cv::morphologyEx( _foreground_mask, filtered_mask, cv::MORPH_OPEN, _morph_element );
    cv::morphologyEx( filtered_mask, filtered_mask, cv::MORPH_CLOSE, _morph_element );

    cv::Mat foreground_image = input_cv.clone();

    for( size_t x = 0; x < filtered_mask.cols; x++ ){
        for( size_t y = 0; y < filtered_mask.rows; y++ ){
            if( filtered_mask.at<uint8_t>( y,x ) > 128 ){
		foreground_image.at<cv::Vec3b>( y,x ) = cv::Vec3b( 255, 0, 0 );
            }
        }
    }

    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    cv::imwrite( "img_conveyor_scene.png", input_cv, compression_params );
    cv::imwrite( "img_conveyor_fgmask.png", filtered_mask, compression_params );
    cv::imwrite( "img_conveyor_fgscene.png", foreground_image, compression_params );
}//Background_filter::write_images

}//namespace Conveyor_object_tracking
