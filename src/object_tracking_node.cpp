// ros includes
#include <ros/ros.h>

// custom tracker class
#include "../include/Object_tracker.h"

// action client and action for robot communication
#include <industrial_kuka_ros/FollowObjectAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<industrial_kuka_ros::FollowObjectAction> Client;

using namespace Conveyor_object_tracking;

int main ( int argc, char* argv[] )
{
	ros::init( argc, argv, "ConveyorObjectTracker" );
	ros::NodeHandle ros_node;

	Object_tracker tracker;
	tracker.attach_to_node( ros_node );
	
	ros::spin();
	ros_node.shutdown();

	return 0;
}//main()
