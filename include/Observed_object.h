#ifndef CONVEYOR_OBJECT_TRACKING__OBSERVED_OBJECT_H
#define CONVEYOR_OBJECT_TRACKING__OBSERVED_OBJECT_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace Conveyor_object_tracking {

/** 
 * Observed_object class
 * represents an object found in a point cloud by clustering
 * holds 3d position and covariance
 */
class Observed_object {
public:
	Observed_object () {} // default constructor required for container initialization
	Observed_object ( const ros::Time t, const Eigen::Vector3f pos, const std::vector<double> radii, const std::vector<double> thicknesses, const std::vector<Eigen::Vector3f> handle, const double h, const double cov, pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj ) // initializing constructor
		: time(t), position(pos), sliceRadii(radii), wallThickness(thicknesses), handleHull(handle), height(h), covariance(cov) , observedCloud(obj) {}
	virtual ~Observed_object () {}

	ros::Time time;
	Eigen::Vector3f position;
	std::vector<double> sliceRadii;
	std::vector<double> wallThickness;
	std::vector<Eigen::Vector3f> handleHull;
	double height;
	double covariance;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr observedCloud;
};//class Observed_object

}//namespace Conveyor_object_tracking

#endif//CONVEYOR_OBJECT_TRACKING__OBSERVED_OBJECT_H
