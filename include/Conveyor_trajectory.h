#ifndef CONVEYOR_OBJECT_TRACKING__CONVEYOR_TRAJECTORY_H
#define CONVEYOR_OBJECT_TRACKING__CONVEYOR_TRAJECTORY_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Eigen>

namespace Conveyor_object_tracking {

class Conveyor_trajectory {
public:
	typedef pcl::PointXYZ Point;
	typedef Eigen::Vector3f Vector;
	typedef Eigen::Vector4f VectorRot;
	typedef typename pcl::PointCloud<Point> Point_cloud;
	typedef typename Point_cloud::Ptr Point_cloud_ptr;
	Conveyor_trajectory();
	void attachNodeHandle( ros::NodeHandle& nh );
	
	bool fetchTrajectoryParameters();

	//update conveyor object with new tracked point data
	void updateTrajectory( Vector position );
	
	/* updates the trajectory with a new object position */
	//TODO declare private after using general function call from Object_tracker
	void updateRecordedTrajectory( Vector position );
	void updateConveyorVelocity( double objectVelocity );
	void finishRecording();
	void publishTrajectoryToRosparam();

	//for visualization purposes
	visualization_msgs::MarkerArray visualizeTrajectory();
	bool trajectoryVisualizable();
	
	//for ROS_INFO
	void updateFrameCounter( int frameCounter);
	
	// trajectory generation for object following & gripping
	std::vector<Eigen::Vector4f> generateTrajectory( double objectHeight, double verticalApproachDistance, double interceptionTime, double robotTactTime, ros::Duration gripperApproachTime, ros::Duration grippingTime );

	double getVisibleTimeLeft(); //calculates and returns the time in second for which the object will remain on the visible/known conveyor path
	ros::Time getTerminationTime();
	ros::Time getParameterTime();
	double getAngleTowardsObjectDirection( Vector objectCenter, Vector handleDirection );
	
	VectorRot getFutureObjectPosition( double timeInSec); //calculates and returns the nearest point on the trajectory
	bool trajectoryCompleted();
	double getVelocity();

private:
	/* PARAMETERS */
	
	//ros node handle for storing/receiving files
	ros::NodeHandle nh;
	std::string name_id;
	
	//overall parameter
	int objectTrackingFrame;
	int recordingCounter;
	Vector error = Vector(0,0,0);
	VectorRot errorRot = VectorRot(0,0,0,0);
	
	//conveyor parameters (trajectory & velocity)
	std::vector<Vector> conveyorCoefficients; //vector with conveyor trajectory parameters (start&end point 1st linear, center of circular, start&end 2nd linear segment)
	Vector firstLineOrthogonal;
	Vector secondLineOrthogonal;
	double conveyorCircleRadius;
	std::vector<double> segmentLengths; //contains length in meter for first segment, circular segment, second segment and total length
	double conveyorVelocity; //estimation of the conveyor velocity
	
	//trajectory calculation
	uint degreeOfCompletion; //0 = no segment, 1 = 1st line segment, 2 = 1st line + circle segment, 3 whole trajectory
	pcl::SACSegmentation<Point>* sacLine;
	pcl::SACSegmentation<Point>* sacCircle;
	bool recordingActive;
	bool calculationFailed;
	std::vector<Vector> objectPositionPoints;
	
	//velocity calculation
	Vector lastObjectPosition;
	ros::Time lastObjectTime;
	
	//object position parameters, updated each frame
	ros::Time currentPositionRequestTime;
	bool currentPositionKnown;
	Eigen::Vector3f currentProjectedPosition;
	double currentDistanceToProjection; // negative values for starboard, positive ones for port
	double currentAngle;
	uint currentSegmentPart;
	double remainingSegmentPath;
	double visibleTimeLeft;
	ros::Time terminationTime;
	
	//trajectory visualization
	visualization_msgs::MarkerArray conveyorMarkers;
	ros::Duration visualizationLifetime;
	
	

	
	/* FUNCTIONS */
	
	//conveyor trajectory generation functions
	void estimateTrajectory();
	bool calculateSACLine(); //calculates the linear segments of the conveyor belt based on recorded points, returns whether calculation was successfull
	bool calculateSACCircle(); //calculates the circular segment of the conveyor belt baed on recorded points, returns whether calculation was successfull
	Vector calculateSnapPoint( Vector lineStart, Vector lineEnd); //calculates the point where line and circle parts of the conveyor are closest to each other (where the sections meet)
	
	//object detection functions
	void calculateCurrentObjectPositionParameters( Vector position ); //calculates and returns the projection of the position on the trajectory
	bool positionParametersValid();	


	//help functions to keep code clean
	double calculateDistance2D( Vector first, Vector second ); //calculates the 2D-Distance between 2 3D points (z assumed as being the same)
	bool clockwiseRotation( Vector first, Vector second ); //returns whether the angle between first and second vector in XY-plane are clockwise
	Vector projectPointToLine( Vector lineOrigin, Vector lineDirection, Vector point ); //projects a point onto a line defined by origin and direction
	Vector projectOnCircle( double angle, double radius ); //returns the point on the circular segment defined by its clockwise angle towards the segment start and the desired radius
	double getPercentOnCircular( double angle ); //for ROS_INFO outputs
	double getPercentOnLinear( double pathLength, double totalLength);//for ROS_INFO outputs
	Eigen::Vector4f convertToVectorWithRotation( Vector position, double rotationInDeg );
	std::vector<double> convertEigenPosition( Vector input );
	Vector convertDoubleToEigenPosition( std::vector<double> input );

	
};

}//namespace Conveyor_object_tracking

#endif /* CONVEYOR_OBJECT_TRACKING__CONVEYOR_TRAJECTORY_H */
