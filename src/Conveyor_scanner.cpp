
#include "../include/Conveyor_scanner.h"

#define FRAMECOUNT_BGS (40) //hoch much frames backgroundSubsctraction is done
#define MAX_ITERATIONS_RANSAC (25)
#define CONVEYOR_TOLERANCE (0.05) //distance threshold for sacSegmentation


namespace Conveyor_scanning {


Conveyor_scanner::Conveyor_scanner ()
    : _frame_count( 0 ),
      _foreground_indices( new Indices ),     //it is a vector-pointer, points to a new vector.
      _plane_coefficients( new pcl::ModelCoefficients )			// a modelcoefficients-pointer, points to a new modelcoefficients
{
	_background_filter = new Background_filter( 3 );  // _background_filter is a Background_filter-pointer, points to a new Background_filter
}//Conveyor_scanner::Conveyor_scanner()

Conveyor_scanner::~Conveyor_scanner () {
	delete _background_filter;
}//Conveyor_scanner::~Conveyor_scanner()

void Conveyor_scanner::attach_to_node ( ros::NodeHandle& ros_node ) {
	// setup ros node
	nh = ros_node;
	//subscribe to raw kinect2 pointcloud data
	_points_subscriber = nh.subscribe( "/kinect2/camera1/depth_registered/points", 5, &Conveyor_scanner::points_callback, this );
	//publish normal pointer to see if the transformation is okay
	_marker_publisher = nh.advertise<visualization_msgs::Marker>( "plane_normal", 0 );
}//Conveyor_scanner::attach_to_node()

void Conveyor_scanner::points_callback ( const Cloud& input_cloud ) {
	
	++_frame_count;
	if( _frame_count < FRAMECOUNT_BGS ) {

		_background_filter->update_foreground_mask( input_cloud );
	} else if( _frame_count == FRAMECOUNT_BGS ) {

		_background_filter->get_foreground_indices( _foreground_indices );
		_background_filter->write_images( input_cloud );
		ROS_DEBUG( "Pointcloud > Background scan complete" );

		//calculate Plane with RANSAC
		Cloud_cptr foreground_cloud( new Cloud( input_cloud, *_foreground_indices ) );  // give the foreground point of input_cloud to foreground_cloud
		update_conveyor_plane( foreground_cloud ); 			//foreground_cloud is a cloud-pointer, points to a Cloud, which was intialed with foreground points.

		//calculate conveyor_plane Transformation
		calculate_conveyor_transformation( foreground_cloud );
		ROS_DEBUG( "Pointcloud > Conveyor plane estimated" );
		publish_transformation_on_rosparam();
		send_static_conveyor_transformation();
		nh.shutdown();
	}

}//Conveyor_scanner::points_callback()

void Conveyor_scanner::update_conveyor_plane ( const Cloud_cptr& input_cloud ) {   //
	pcl::SACSegmentation<Point> segmenter;

	segmenter.setModelType( pcl::SACMODEL_PLANE );
	segmenter.setMethodType( pcl::SAC_RANSAC );
	segmenter.setDistanceThreshold( CONVEYOR_TOLERANCE );
	segmenter.setOptimizeCoefficients( true );
	segmenter.setInputCloud( input_cloud );  					 //here input_cloud is a Cloud con. pointer, (foreground_cloud)
	segmenter.setMaxIterations( MAX_ITERATIONS_RANSAC );
	
	pcl::PointIndices inliers;
	segmenter.segment( inliers, *_plane_coefficients );				//inliers represents the support point. _plane_coefficients is a modellcoefficients-pointer, points to the plane coefficients
	ROS_INFO("We have %d inliers out of %d total points.", (int) inliers.indices.size(), (int) input_cloud.get()->size());

	Eigen::Vector3f normal( _plane_coefficients->values[0], _plane_coefficients->values[1], _plane_coefficients->values[2] );
	if( normal[1] < 0 ){
	    normal = -normal;
	}

}//Conveyor_scanner::update_conveyor_plane

void Conveyor_scanner::calculate_conveyor_transformation ( const Cloud_cptr& input_cloud ) {
    Eigen::Vector3f normal( _plane_coefficients->values[0], _plane_coefficients->values[1], _plane_coefficients->values[2] );  //the normal of the conveyor plane 
    if( normal[2] < 0 ){
        normal = -normal;
    }

	//calculate transformation parameters for kinect->conveyor coordinates
	Eigen::Vector3f ZAxis(0, 0, -1);			//
	_conveyorplane_rotation.setFromTwoVectors(ZAxis, normal);   //	Eigen::Quaternionf _conveyorplane_rotation;
	_conveyorplane_translation = Eigen::Vector3f(input_cloud->points[0].x, input_cloud->points[0].y, input_cloud->points[0].z);
	frame_id = "kinect2_depth_frame";
	child_frame_id = "conveyor_plane";

	ROS_DEBUG("Transformation to ConveyorFrame calculated:\n  Translation (%f, %f, %f)\n Rotation (%f, %f, %f, %f)",
			_conveyorplane_translation.x(),
			_conveyorplane_translation.y(),
			_conveyorplane_translation.z(),
			_conveyorplane_rotation.x(),
			_conveyorplane_rotation.y(),
			_conveyorplane_rotation.z(),
			_conveyorplane_rotation.w());

	visualization_msgs::Marker PlaneNormalMarkerMsg;
	PlaneNormalMarkerMsg.header.frame_id = "kinect2_depth_frame";
	PlaneNormalMarkerMsg.header.stamp = ros::Time();
	PlaneNormalMarkerMsg.ns = "my_namespace";
	PlaneNormalMarkerMsg.id = 0;
	PlaneNormalMarkerMsg.type = visualization_msgs::Marker::ARROW;
	PlaneNormalMarkerMsg.action = visualization_msgs::Marker::ADD;
	PlaneNormalMarkerMsg.pose.position.x = _conveyorplane_translation.x();
	PlaneNormalMarkerMsg.pose.position.y = _conveyorplane_translation.y();
	PlaneNormalMarkerMsg.pose.position.z = _conveyorplane_translation.z();
	PlaneNormalMarkerMsg.pose.orientation.x = _conveyorplane_rotation.x();
	PlaneNormalMarkerMsg.pose.orientation.y = _conveyorplane_rotation.y();
	PlaneNormalMarkerMsg.pose.orientation.z = _conveyorplane_rotation.z();
	PlaneNormalMarkerMsg.pose.orientation.w = _conveyorplane_rotation.w();
	PlaneNormalMarkerMsg.scale.x = 0.5;
	PlaneNormalMarkerMsg.scale.y = 0.03;
	PlaneNormalMarkerMsg.scale.z = 0.03;
	PlaneNormalMarkerMsg.color.a = 1.0; // Don't forget to set the alpha!
	PlaneNormalMarkerMsg.color.r = 0.0;
	PlaneNormalMarkerMsg.color.g = 1.0;
	PlaneNormalMarkerMsg.color.b = 0.0;
	_marker_publisher.publish(PlaneNormalMarkerMsg);
    

	ROS_DEBUG("Plane normal direction calculated as (%f, %f, %f)", normal.x(), normal.y(),normal.z());

}//Conveyor_scanner::get_conveyor_transformation

void Conveyor_scanner::publish_transformation_on_rosparam() {

		ROS_INFO("Publishing transformation from kinect2 to conveyorFrame onto parameter server");
		std::vector<double> translationVector(3);
		translationVector[0] = _conveyorplane_translation.x();
		translationVector[1] = _conveyorplane_translation.y();
		translationVector[2] = _conveyorplane_translation.z();
		std::vector<double> rotationVector(4);
		rotationVector[0] = _conveyorplane_rotation.x();
		rotationVector[1] = _conveyorplane_rotation.y();
		rotationVector[2] = _conveyorplane_rotation.z();
		rotationVector[3] = _conveyorplane_rotation.w();

		nh.setParam( "transform/translation", translationVector );
		nh.setParam( "transform/rotation", rotationVector );
		nh.setParam( "frame_id", frame_id);
		nh.setParam( "child_frame_id", child_frame_id);

		system("rosparam dump `rospack find conveyor_object_tracking`/config/kinect2conveyor_tf.yaml /kinect2conveyor");

}//Conveyor_scanner::publish_transformation_on_rosparam()

void Conveyor_scanner::send_static_conveyor_transformation() {
	kinect2_to_conveyor_TF.header.frame_id = frame_id;
	kinect2_to_conveyor_TF.child_frame_id = child_frame_id;
	kinect2_to_conveyor_TF.transform.translation.x = _conveyorplane_translation.x();
	kinect2_to_conveyor_TF.transform.translation.y = _conveyorplane_translation.y();
	kinect2_to_conveyor_TF.transform.translation.z = _conveyorplane_translation.z();
	kinect2_to_conveyor_TF.transform.rotation.w = _conveyorplane_rotation.w();
	kinect2_to_conveyor_TF.transform.rotation.x = _conveyorplane_rotation.x();
	kinect2_to_conveyor_TF.transform.rotation.y = _conveyorplane_rotation.y();
	kinect2_to_conveyor_TF.transform.rotation.z = _conveyorplane_rotation.z();

	tfb.sendTransform(kinect2_to_conveyor_TF);  //static transformation from "kinect2_depth_frame" to "conveyor_plane"，
												//Send a vector of TransformStamped messages The stamped data structure includes frame_id, and time, and parent_id already.

}//Conveyor_scanner::send_static_conveyor_transformation


}//namespace Conveyor_object_tracking
