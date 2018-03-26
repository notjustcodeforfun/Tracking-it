
// ros includes
#include <ros/ros.h>
#include <opencv/highgui.h>

// custom class
#include "../include/Conveyor_scanner.h"

using namespace Conveyor_scanning;

int main ( int argc, char* argv[] )
{

	ros::init( argc, argv, "ConveyorScanner" );
	Conveyor_scanner scanner;
	ros::NodeHandle ros_node( scanner.globalID );

	scanner.attach_to_node( ros_node );
	
	ros::Rate loop_rate( 100 );

	for( ; ros::ok(); ) {
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros_node.shutdown();

	return 0;
}//main()
