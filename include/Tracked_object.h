#ifndef CONVEYOR_OBJECT_TRACKING__TRACKED_OBJECT_H
#define CONVEYOR_OBJECT_TRACKING__TRACKED_OBJECT_H

#include <ros/ros.h>

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "Observed_object.h"

namespace Conveyor_object_tracking {

/** 
 * Tracked_object class
 * represents a tracked object using a simplified kalman filter
 * holds 6d state, 3d position and 3d velocity
 */
class Tracked_object {
public:
	Tracked_object () {} // default constructor is required for container initialization
	Tracked_object ( const Observed_object& observation, Eigen::Vector3f objPosition ); // initializing constructor

	void update_position ( const Observed_object& observation );
	void predict_position ( const ros::Time time, Eigen::Vector3f* prediction, double* prediction_cov ) const;
	void predict_velocity ( const ros::Time time, Eigen::Vector3f* prediction, double* prediction_cov ) const;
	void update_object ( pcl::PointCloud<pcl::PointXYZRGB> newObject , Eigen::Vector3f _last_object_position, double additionalDegrees);
	pcl::PointCloud<pcl::PointXYZRGB> getObject ();
	Eigen::Vector3f getLastObjectPosition ();
	std::vector<double> get_radii ();
	std::vector<double> get_wallThickness ();
	std::vector<Eigen::Vector3f> get_handleHull ();
	double get_interval ( const ros::Time time );
	Eigen::Vector3f get_position ();
	double getDegreesReconstructed ();
	double getHeight ();

	unsigned int unobserved;
	unsigned int age;

private:
	Eigen::Vector3f _position;
	Eigen::Vector3f _velocity;

	std::vector<double> _radii;
	std::vector<double> _wallThickness;
	std::vector<Eigen::Vector3f> _handleHull;
	double _height;
	double _position_cov;
	double _velocity_cov;

	ros::Time _first_time;
	ros::Time _last_time;

	pcl::PointCloud<pcl::PointXYZRGB> constructedObject;
	Eigen::Vector3f _last_object_position;
	float degreesReconstructed;

};//class Tracked_object

}//namespace Conveyor_object_tracking

#endif//CONVEYOR_OBJECT_TRACKING__TRACKED_OBJECT_H
