#ifndef CONVEYOR_SCANNING__CONVEYOR_SCANNER_H
#define CONVEYOR_SCANNING__CONVEYOR_SCANNER_H

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <cmath>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf_conversions/tf_eigen.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include "../include/pcl_types.h"

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <cv_bridge/cv_bridge.h>

#include <visualization_msgs/Marker.h>

#include "../include/Background_filter.h"

namespace Conveyor_scanning {

class Conveyor_scanner {
    public:
      
	typedef Eigen::Vector3f Vector;
	typedef pcl::PointXYZRGB Point;
	typedef pcl::PointCloud<Point> Cloud;
	typedef Cloud::Ptr Cloud_ptr;
	typedef Cloud::ConstPtr Cloud_cptr;
	typedef pcl::Normal Normal;
	typedef pcl::PointCloud<Normal> Normals;
	typedef Normals::Ptr Normals_ptr;
	typedef Normals::ConstPtr Normals_cptr;
	typedef std::vector<int> Indices;
	typedef boost::shared_ptr<Indices> Indices_ptr;
	typedef boost::shared_ptr<Indices const> Indices_cptr;
	
	// constructors
	Conveyor_scanner ();

	// destructor
	~Conveyor_scanner ();

	// public methods
	void attach_to_node ( ros::NodeHandle& );
	void points_callback ( const Cloud& );

	//global identifier for parameter server
	std::string globalID = "kinect2conveyor";

    private:
	void update_foreground_mask ( const Cloud_cptr& );
	void update_conveyor_plane ( const Cloud_cptr& );
	void calculate_conveyor_transformation ( const Cloud_cptr& );
	void publish_transformation_on_rosparam();
	void send_static_conveyor_transformation();

	//Ros Node Handle & topics
	ros::NodeHandle nh;

	Background_filter* _background_filter;

	// ros topics
	ros::Subscriber _points_subscriber;
	ros::Publisher _marker_publisher;

	long _frame_count;
	Indices_ptr _foreground_indices;
	pcl::ModelCoefficients::Ptr _plane_coefficients;

	//transformation data
	Eigen::Quaternionf _conveyorplane_rotation;
	Vector _conveyorplane_translation;
	std::string frame_id;
	std::string child_frame_id;

	tf2_ros::StaticTransformBroadcaster tfb;
	geometry_msgs::TransformStamped kinect2_to_conveyor_TF;

	// disallow copy and assign
	Conveyor_scanner ( const Conveyor_scanner& );
	void operator= ( const Conveyor_scanner& );
	
};//class Conveyor_scanner

}//namespace Conveyor_object_tracking

#endif//CONVEYOR_SCANNING__CONVEYOR_SCANNER_H
