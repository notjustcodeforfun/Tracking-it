// ros includes
#include <ros/ros.h>
#include <opencv/highgui.h>

// custom tracker class
#include "../include/Conveyor_filter.h"

using namespace Conveyor_object_tracking;

int main ( int argc, char* argv[] )
{

	ros::init( argc, argv, "ConveyorFiltering" );
	Conveyor_filter filter;
	ros::NodeHandle ros_node;

	filter.attach_to_node( ros_node );
	
	ros::Rate loop_rate( 100 );

	for( ; ros::ok(); ) {
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros_node.shutdown();

	return 0;
}//main()
