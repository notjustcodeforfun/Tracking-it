#ifndef CONVEYOR_OBJECT_TRACKING__CONVEYOR_FILTER_H
#define CONVEYOR_OBJECT_TRACKING__CONVEYOR_FILTER_H

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
#include "pcl_types.h"

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <cv_bridge/cv_bridge.h>

#include <visualization_msgs/Marker.h>

#include "Background_filter.h"




namespace Conveyor_object_tracking {

class Conveyor_filter {
    public:
	typedef Eigen::Vector3f Vector;
	typedef Eigen::Quaternionf Quaternion;

      // constructors
    	Conveyor_filter ();

      // destructor
    	~Conveyor_filter ();

      // public methods
    	void attach_to_node ( ros::NodeHandle& );
    	void points_callback ( const Cloud& );

    private:
        //tries to fetch the transformation parameters from ROS parameter server, returns whether any are available
    	bool publish_transformation_from_rosparam( std::string id, Vector& translation, Quaternion& rotation);
        void filter_objects ( const Cloud_cptr& );

        //Ros Node Handle
        ros::NodeHandle nh;

    	// private data members
    	ros::Subscriber _points_subscriber;
    	ros::Publisher _points_publisher;
    	ros::Publisher _transformed_points_publisher;
    	ros::Publisher _marker_publisher;

        //transformation data
	Vector _conveyorplane_translation;
        Quaternion _conveyorplane_rotation;

        Eigen::Matrix4f _plane_transformation;
        Eigen::Matrix4f _CirclePatternTransformation;

        tf2_ros::StaticTransformBroadcaster tfb;

    	// disallow copy and assign
	    Conveyor_filter ( const Conveyor_filter& );
    	void operator= ( const Conveyor_filter& );
};//class Conveyor_scanner

}//namespace Conveyor_object_tracking

#endif//CONVEYOR_OBJECT_TRACKING__CONVEYOR_FILTER_H
