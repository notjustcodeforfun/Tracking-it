
#include "../include/Conveyor_filter.h"

#define BOUNDING_BOX_MIN_KINECT_DEPTH (0.95)// ntd
#define BOUNDING_BOX_MAX_KINECT_DEPTH (1.58)//ntd
#define BOUNDING_BOX_CONVEYOR_OFFSET (0.045) //positive offset needed to exclude conveyor borders
#define BOUNDING_BOX_MAX_OBJECT_HEIGHT (0.35)
#define MIN_OBJECT_SIZE (800) //cloud with less than this amount of points won't be published

namespace Conveyor_object_tracking
{

Conveyor_filter::Conveyor_filter () {
  Vector vector;
  Quaternion quaternion;
  
  publish_transformation_from_rosparam("kinect2conveyor" , _conveyorplane_translation, _conveyorplane_rotation);
  publish_transformation_from_rosparam("kinect2kuka", vector, quaternion);
}//Conveyor_filter::Conveyor_filter()

Conveyor_filter::~Conveyor_filter () {
}//Conveyor_filter::~Conveyor_filter()

void Conveyor_filter::attach_to_node ( ros::NodeHandle& ros_node ) {
	nh = ros_node;
    // setup ros node
    _points_publisher = nh.advertise<Cloud&>( "conveyor_object_tracker/object_points", 5 );
    _transformed_points_publisher = nh.advertise<Cloud&>( "conveyor_object_tracker/conveyorframe_object_points", 5 );
    _points_subscriber = nh.subscribe( "/kinect2/camera1/depth_registered/points", 5, &Conveyor_filter::points_callback, this );
    _marker_publisher = nh.advertise<visualization_msgs::Marker>( "plane_normal", 0 );
}//Conveyor_filter::attach_to_node()

void Conveyor_filter::points_callback ( const Cloud& input_cloud ) {
   	ROS_INFO_ONCE("CONVEYOR_FILTER: Beginning to send point cloud data!");
   	Cloud_cptr full_cloud( new Cloud(input_cloud));
   	filter_objects( full_cloud );

}//Conveyor_filter::points_callback()

bool Conveyor_filter::publish_transformation_from_rosparam(std::string id, Vector& translation, Quaternion& rotation) {

	ROS_DEBUG("Trying to fetch previously published transformation from parameter server");
	if ( nh.hasParam("/" + id) ) {

		geometry_msgs::TransformStamped transform;
		nh.getParam("/" + id + "/frame_id", transform.header.frame_id);
		std::cout << transform.header.frame_id << std::endl;
		nh.getParam("/" + id + "/child_frame_id", transform.child_frame_id);
		std::cout << transform.child_frame_id << std::endl;
		std::vector<double> translationVector;
		std::vector<double> rotationVector;
		nh.getParam("/" + id + "/transform/translation", translationVector );
		nh.getParam("/" + id + "/transform/rotation", rotationVector );

		transform.transform.translation.x = translationVector[0];
		transform.transform.translation.y = translationVector[1];
		transform.transform.translation.z = translationVector[2];
		transform.transform.rotation.x = rotationVector[0];
		transform.transform.rotation.y = rotationVector[1];
		transform.transform.rotation.z = rotationVector[2];
		transform.transform.rotation.w = rotationVector[3];
		//Added someting

		translation = Vector( translationVector[0], translationVector[1], translationVector[2] );
		rotation.x() = rotationVector[0];
		rotation.y() = rotationVector[1];
		rotation.z() = rotationVector[2];
		rotation.w() = rotationVector[3];

		tfb.sendTransform(transform);  //static transformation from "kinect2_depth_frame" to "conveyor_plane"
		ROS_INFO("Successfully fetched previous transformation data!");
		return true;
	} else {
		ROS_ERROR("Fetching transformation data not possible!");
		return false;
	}
}//Conveyor_filter::publish_transformation_from_rosparam

void Conveyor_filter::filter_objects ( const Cloud_cptr& input_cloud ) {
	//cropping cloud part 1: cut points in kinect coordinates depth based (cut foreground and background)
	Indices cropped_indices;
	    for( size_t i = 0; i < input_cloud->size(); i++ ){
	    	float z = input_cloud->points[i].z;
	        if(z > BOUNDING_BOX_MIN_KINECT_DEPTH && z < BOUNDING_BOX_MAX_KINECT_DEPTH ){
	        	cropped_indices.push_back(i);
	        }
	    }
	Cloud cropped_cloud( *input_cloud, cropped_indices );

	//cropping cloud part 2: transform remaining cloud into conveyor coordinates and cut points lower than conveyor to extract objects
	Cloud_ptr transformed_cloud( new Cloud );
	Eigen::Quaternionf ID;
	ID.setIdentity();							//Indentity matrix
	pcl::transformPointCloud( cropped_cloud, *transformed_cloud, -_conveyorplane_translation, ID); //first translate points
	pcl::transformPointCloud( *transformed_cloud, *transformed_cloud, Eigen::Vector3f(0,0,0), _conveyorplane_rotation.inverse() ); //then rotate

	Indices object_indices;
    for( size_t i = 0; i < transformed_cloud->size(); i++ ){
    	float z = transformed_cloud->points[i].z;
        if( z > BOUNDING_BOX_CONVEYOR_OFFSET && z <  BOUNDING_BOX_MAX_OBJECT_HEIGHT ){
            object_indices.push_back(i);
        }
    }
    Cloud transformed_obj_cloud( *transformed_cloud, object_indices);
    Cloud object_cloud( cropped_cloud, object_indices );

    if (object_cloud.size() > MIN_OBJECT_SIZE) {
    	ROS_DEBUG("UPDATE OBJECTS: InputCloud size: %d, CroppedCloud size: %d, ObjectCloud size: %d.",
			(int) input_cloud.get()->size(),
			(int) cropped_cloud.size(),
			(int) object_cloud.size());
	object_cloud.header.frame_id = input_cloud->header.frame_id;
    	_points_publisher.publish( object_cloud );
	transformed_obj_cloud.header.frame_id = "conveyor_plane";
    	_transformed_points_publisher.publish( transformed_obj_cloud );
    } else {
    	ROS_DEBUG("UPDATE OBJECTS: ObjectCloud size too small (%d)", (int) object_cloud.size());
    }

}//Conveyor_filter::update_objects


}//namespace Conveyor_object_tracking
