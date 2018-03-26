#include "../include/Conveyor_trajectory.h"

#define ANGLE_THRESHOLD ( 13.0 ) // value in degrees upon which two following trajectory pieces are considered to be part of a circle trajectory
#define OBJECT_CONVEYOR_OFFSET ( 0.05 ) //MUST be of same value as defined in Object_tracker.cpp
#define TRAJECTORY_RECORDING_RESOLUTION ( 0.03 ) //value in meter

namespace Conveyor_object_tracking {


Conveyor_trajectory::Conveyor_trajectory()
	:	objectPositionPoints( 0 ),
	 	conveyorCoefficients( 5 ),// stores 2 vectors for first line, 1 for circle center, 2 for second line
	 	segmentLengths( 4 ),
		degreeOfCompletion( 0 ),
		conveyorCircleRadius( 0.0 ),
		calculationFailed( false ),
		visualizationLifetime( 3.0 ),
		currentPositionKnown( false ),
		recordingCounter( 1 ),
		lastObjectPosition( error ),
		name_id( "conveyorTrajectory" )
{
		//ransac initialization for conveyor line recognition
		sacLine = new pcl::SACSegmentation<Point>;
		sacLine->setModelType( pcl::SACMODEL_LINE );
		sacLine->setMethodType( pcl::SAC_RANSAC );
		sacLine->setDistanceThreshold( 0.01 );
		sacLine->setMaxIterations( 5 );

		//ransac initialization for conveyor circle segment calculation
		sacCircle = new pcl::SACSegmentation<Point>;
		sacCircle->setModelType( pcl::SACMODEL_CIRCLE2D );
		sacCircle->setMethodType( pcl::SAC_RANSAC );
		sacCircle->setMaxIterations( 7 );
		sacCircle->setDistanceThreshold( 0.015 );
		sacCircle->setRadiusLimits( 0.08, 0.2 );
		sacCircle->setOptimizeCoefficients( true );

}//Conveyor_trajectory::Conveyor_trajectory

void Conveyor_trajectory::attachNodeHandle( ros::NodeHandle& nodeHandle ) {
  
  nh = nodeHandle;
  
}//Conveyor_trajectory::attachNodeHandle

bool Conveyor_trajectory::trajectoryVisualizable() {
	return( (degreeOfCompletion > 0) || (!objectPositionPoints.empty()) );
}//Conveyor_trajectory::trajectoryVisualizable

void Conveyor_trajectory::updateTrajectory( Vector position ) {
	
	lastObjectTime = ros::Time::now();
	lastObjectPosition = currentProjectedPosition;

	
	if ( !trajectoryCompleted() ) {
		updateRecordedTrajectory( position );
	} else {
		calculateCurrentObjectPositionParameters( position );
	}
	
	recordingCounter++;
}//Conveyor_trajectory::updateTrajectory

void Conveyor_trajectory::updateRecordedTrajectory( Vector position ) {

	recordingActive = true;

	if ( objectPositionPoints.empty() ) { //for empty vector add first position

		objectPositionPoints.push_back(position);

	} else if ( calculateDistance2D( objectPositionPoints.back(), position ) > TRAJECTORY_RECORDING_RESOLUTION ) {
		objectPositionPoints.push_back(position);
		ROS_DEBUG("Frame %d: CONVEYOR_TRAJECTORY_%d: Added new point to list! (now containing %d points)", objectTrackingFrame, degreeOfCompletion, (int) objectPositionPoints.size() );
		estimateTrajectory();

	}

}//Conveyor_trajectory::updateRecordedTrajectory

void Conveyor_trajectory::finishRecording() {

	if ( recordingActive ) {
		ROS_INFO_ONCE("Frame %d: CONVEYOR_TRAJECTORY_%d: FINISH RECORDING TRIGGERED!", objectTrackingFrame, degreeOfCompletion );
		recordingActive = false;
		estimateTrajectory();
		visualizationLifetime = ros::Duration(40.0);
		
		segmentLengths[0] = calculateDistance2D(conveyorCoefficients[0], conveyorCoefficients[1]);
		segmentLengths[1] = M_PI * conveyorCircleRadius;
		segmentLengths[2] = calculateDistance2D(conveyorCoefficients[3], conveyorCoefficients[4]);
		segmentLengths[3] = segmentLengths[0] + segmentLengths[1] + segmentLengths[2];
		Vector firstLineDirection = conveyorCoefficients[1] - conveyorCoefficients[0];
		firstLineDirection.normalize();
		firstLineOrthogonal = Vector(-1.0 * firstLineDirection[1], firstLineDirection[0], 0.0);
		Vector secondLineDirection = conveyorCoefficients[4] - conveyorCoefficients[3];
		secondLineDirection.normalize();
		secondLineOrthogonal = Vector(-1.0 * secondLineDirection[1], secondLineDirection[0], 0.0);
		
		publishTrajectoryToRosparam();
	}
}//Conveyor_trajectory::finishRecording

void Conveyor_trajectory::estimateTrajectory() {

	size_t size = objectPositionPoints.size();

	if (size > 3) {

		Vector point1 = objectPositionPoints[size - 4];
		Vector point2 = objectPositionPoints[size - 3];
		Vector point3 = objectPositionPoints[size - 2];
		Vector point4 = objectPositionPoints[size - 1];

		Vector velocity1 = point2 - point1;
		Vector velocity2 = point3 - point2;
		Vector velocity3 = point4 - point3;
		velocity1.normalize();
		velocity2.normalize();
		velocity3.normalize();
		double firstAngle = ( acos( velocity1.dot(velocity2) ) * 180 ) / M_PI;
		bool firstDirLeft = ( (velocity1[0] * velocity2[1] - velocity1[1] * velocity2[0]) > 0 );
		if (firstDirLeft) { firstAngle *= -1.0; }
		double secondAngle = ( acos( velocity2.dot(velocity3) ) * 180 ) / M_PI;
		bool secondDirLeft = ( (velocity2[0] * velocity3[1] - velocity2[1] * velocity3[0]) > 0 );
		if (secondDirLeft) { secondAngle *= -1.0; }
		double combinedAngles = firstAngle + secondAngle;

		bool straight = ( combinedAngles < ANGLE_THRESHOLD);
		if (straight) {
			ROS_DEBUG("Frame %d: CONVEYOR_TRAJECTORY_%d: Last recorded section detected as linear: %f < %f degree.", objectTrackingFrame, degreeOfCompletion, combinedAngles, ANGLE_THRESHOLD );
		} else {
			ROS_DEBUG("Frame %d: CONVEYOR_TRAJECTORY_%d: Last recorded section detected as circular: %f >= %f degree.", objectTrackingFrame, degreeOfCompletion, combinedAngles, ANGLE_THRESHOLD );
		}

		if ( (degreeOfCompletion == 0) && straight ) {
			// do nothing, wait until object enters circular segment
			ROS_INFO_ONCE("Frame %d: CONVEYOR_TRAJECTORY_0: Recording points for first linear segment, %d points so far.", objectTrackingFrame, (int) objectPositionPoints.size() );
		} else if ( degreeOfCompletion == 0 ) {
			//calculate first linear conveyor trajectory segment
			if (!calculateSACLine() ) {//circle not calculated
				ROS_ERROR("Frame %d: CONVEYOR_TRAJECTORY_0: Calculation of first linear segment failed, try again with more points!", objectTrackingFrame );
			} else {
				degreeOfCompletion++;
			}
		} else if ( (degreeOfCompletion == 1) && !straight ) {
			//do nothing, wait until object leaves circular segment
			ROS_INFO_ONCE("Frame %d: CONVEYOR_TRAJECTORY_1: Recording points for circular segment.", objectTrackingFrame );

		} else if ( degreeOfCompletion == 1 ) {
			//calculate circle conveyor trajectory segment
			if ( !calculateSACCircle() ) {//circle not calculated
				ROS_ERROR("Frame %d: CONVEYOR_TRAJECTORY_1: Calculation of circular segment failed, try again with more points!", objectTrackingFrame );
			} else {
				degreeOfCompletion++;
				conveyorCoefficients[1] = calculateSnapPoint(conveyorCoefficients[0], conveyorCoefficients[1]);
			}

		} else if ( (degreeOfCompletion == 2) && straight && recordingActive ) {
			//do nothing, wait until complete linear path is recorded
			ROS_INFO_ONCE("Frame %d: CONVEYOR_TRAJECTORY_2: Recording points for second linear segment.", objectTrackingFrame );

		} else if ( (degreeOfCompletion == 2) && straight && !recordingActive ) {
			//calculate second linear conveyor trajectory segment and store last point as end of whole trajectory

			if ( !calculateSACLine() ) {//second linear segment not calculated
				ROS_ERROR("Frame %d: CONVEYOR_TRAJECTORY_2: Calculation of second linear segment failed, try again with more points!", objectTrackingFrame );
			} else {
				conveyorCoefficients[3] = calculateSnapPoint(conveyorCoefficients[3], conveyorCoefficients[4]);
				//update circle center and radius based on both calculated lines
				conveyorCircleRadius = 0.5 * calculateDistance2D(conveyorCoefficients[1], conveyorCoefficients[3]);
				Vector circleDirection = conveyorCoefficients[2] - conveyorCoefficients[1];
				circleDirection.normalize();
				conveyorCoefficients[2] = conveyorCoefficients[1] + circleDirection * conveyorCircleRadius;
				degreeOfCompletion++;
				ROS_INFO_ONCE("Frame %d: CONVEYOR_TRAJECTORY_3: Finished conveyor trajectory calculation by adding end point!", objectTrackingFrame );
			}

		} else if ( degreeOfCompletion == 2 ){
			ROS_ERROR("Frame %d: CONVEYOR_TRAJECTORY_2: Second circular segment detected, but trajectory only has one!", objectTrackingFrame );
		} else if ( !recordingActive ) {
			ROS_ERROR("Frame %d: CONVEYOR_TRAJECTORY: Stopping before all three segments were recorded!", objectTrackingFrame );
		}

	} else if ( recordingActive ) {
		ROS_DEBUG("Frame %d: CONVEYOR_TRAJECTORY_%d: Waiting for more points to be recorded!", objectTrackingFrame, degreeOfCompletion );
	} else {
		ROS_WARN("Frame %d: CONVEYOR_TRAJECTORY_%d: Stopping before all three segments were recorded!", objectTrackingFrame, degreeOfCompletion );
	}

}//Conveyor_trajectory::estimateTrajectory

bool Conveyor_trajectory::calculateSACLine() {

	pcl::ModelCoefficients::Ptr coefficients ( new pcl::ModelCoefficients );
	pcl::PointIndices::Ptr inliers ( new pcl::PointIndices );
	Point_cloud_ptr lineCloud ( new Point_cloud );

	for (size_t i = 0; i < objectPositionPoints.size() - 1; ++i ) {
		Vector positionVector = objectPositionPoints[i];
		Point positionPoint( positionVector.x(), positionVector.y(), positionVector.z() );
		lineCloud->push_back( positionPoint );
	}

	sacLine->setInputCloud( lineCloud );
	sacLine->segment( *inliers, *coefficients );

	if ( inliers->indices.size() == 0 ) {
		return false;
	}
	Vector lastPoint = objectPositionPoints.back();
	Vector secondLastPoint = objectPositionPoints[objectPositionPoints.size() - 2];
	Vector thirdLastPoint = objectPositionPoints[objectPositionPoints.size() - 3];
	Vector lineOrigin ( coefficients->values[0], coefficients->values[1], coefficients->values[2] );
	Vector lineDirection ( coefficients->values[3], coefficients->values[4], coefficients->values[5] );
	Vector startOfLine = projectPointToLine( lineOrigin, lineDirection, objectPositionPoints.front() );
	Vector endOfLine;

	if ( degreeOfCompletion == 0) {
		conveyorCoefficients[0] = startOfLine;
		endOfLine = lineOrigin + lineDirection.dot( secondLastPoint - lineOrigin ) * lineDirection;
		conveyorCoefficients[1] = endOfLine;
		ROS_DEBUG("Frame %d: CONVEYOR_TRAJECTORY_0: First linear segment calculated!", objectTrackingFrame );
		ROS_DEBUG("Frame %d: CONVEYOR_TRAJECTORY_0: L1:(%f,%f,%f) V1:(%f,%f,%f)", objectTrackingFrame, lineOrigin[0], lineOrigin[1], lineOrigin[2],
																						lineDirection[0], lineDirection[1], lineDirection[2] );
		ROS_DEBUG("Frame %d: CONVEYOR_TRAJECTORY_0: P0:(%f,%f,%f) P1:(%f,%f,%f)", objectTrackingFrame, startOfLine[0], startOfLine[1], startOfLine[2],
																						endOfLine[0], endOfLine[1], endOfLine[2] );
		objectPositionPoints.clear();
		objectPositionPoints.push_back( endOfLine ); //instead of secondLastPoint, so that circular and linear segments share the same points
		objectPositionPoints.push_back( lastPoint );
	} else {
		conveyorCoefficients[3] = startOfLine;
		endOfLine = projectPointToLine( lineOrigin, lineDirection, lastPoint );
		conveyorCoefficients[4] = endOfLine;
		ROS_DEBUG("Frame %d: CONVEYOR_TRAJECTORY_2: Second linear segment calculated.", objectTrackingFrame );
		ROS_DEBUG("Frame %d: CONVEYOR_TRAJECTORY_2: L1:(%f,%f,%f) V1:(%f,%f,%f)", objectTrackingFrame, lineOrigin[0], lineOrigin[1], lineOrigin[2],
																						lineDirection[0], lineDirection[1], lineDirection[2] );
		ROS_DEBUG("Frame %d: CONVEYOR_TRAJECTORY_2: P3:(%f,%f,%f) P4:(%f,%f,%f)", objectTrackingFrame, startOfLine[0], startOfLine[1], startOfLine[2],
																						endOfLine[0], endOfLine[1], endOfLine[2] );
		objectPositionPoints.clear();
	}

	return true;
}//Conveyor_trajectory::calculateSACLine

bool Conveyor_trajectory::calculateSACCircle() {

	pcl::ModelCoefficients::Ptr coefficients ( new pcl::ModelCoefficients );
	pcl::PointIndices::Ptr inliers ( new pcl::PointIndices );
	Point_cloud_ptr circleCloud ( new Point_cloud );
	for (size_t i = 0; i < objectPositionPoints.size(); ++i ) {
		Vector positionVector = objectPositionPoints[i];
		Point positionPoint( positionVector.x(), positionVector.y(), positionVector.z() );
		circleCloud->push_back( positionPoint );
	}

	sacCircle->setInputCloud( circleCloud );
	sacCircle->segment( *inliers, *coefficients );

	if (inliers->indices.size() == 0) {
		return false;
	}

	Vector lastPoint = objectPositionPoints.back();
	Vector secondLastPoint = objectPositionPoints[objectPositionPoints.size() - 2];
	Vector center ( coefficients->values[0], coefficients->values[1], OBJECT_CONVEYOR_OFFSET );
	conveyorCircleRadius = coefficients->values[2];
	conveyorCoefficients[2] = center;
	Vector eocDirection = lastPoint - center;
	eocDirection.normalize();
	//Vector endOfCircle = center + eocDirection * conveyorCircleRadius;  //last point belonging to the circle
	Vector endOfCircle = projectOnCircle(M_PI, conveyorCircleRadius);

	ROS_DEBUG("Frame %d: CONVEYOR_TRAJECTORY_1: Circular segment calculated.", objectTrackingFrame );
	ROS_DEBUG("Frame %d: CONVEYOR_TRAJECTORY_1: Center:(%f,%f,%f) Radius: %f", objectTrackingFrame, center[0], center[1], center[2], conveyorCircleRadius );
	ROS_DEBUG("Frame %d: CONVEYOR_TRAJECTORY_1: EndOfCircle:(%f,%f,%f)", objectTrackingFrame, endOfCircle[0], endOfCircle[1], endOfCircle[2] );

	objectPositionPoints.clear();

	objectPositionPoints.push_back(endOfCircle);//instead of secondLastPoint, so that circular and linear segments share the same points
	objectPositionPoints.push_back(lastPoint);
	return true;
}//Conveyor_trajectory::calculateSACCircle

Eigen::Vector3f Conveyor_trajectory::calculateSnapPoint( Vector lineStart, Vector lineEnd ) {

	if (degreeOfCompletion < 2) {
		ROS_ERROR("Function Conveyor_trajectory::calculateSnapPoint called while second part of Trajectory is still unknown!");
		return Vector(-1,-1,-1);
	} else {
		Vector lineOrigin = lineStart;
		Vector lineDirection = lineEnd - lineStart;
		lineDirection.normalize();
		return projectPointToLine(lineOrigin, lineDirection, conveyorCoefficients[2] );
	}

}//Conveyor_trajectory::calculateSnapPoint

bool Conveyor_trajectory::trajectoryCompleted() {
	return ( degreeOfCompletion == 3 );
}//Conveyor_trajectory::trajectoryCompleted

void Conveyor_trajectory::calculateCurrentObjectPositionParameters( Vector positionEstimation ) {
	
	//double pathLength = timeInSec * conveyorVelocity; //length in meters
	if ( !trajectoryCompleted() ) {
		ROS_ERROR("Frame %d: OBJECT_PROJECTION: call while trajectory has not been calculated completely!"
				, objectTrackingFrame);
		currentProjectedPosition = error;
		currentDistanceToProjection = 99.9;
	} else {
		//projection of object position onto first line
		Vector firstLineOrigin = conveyorCoefficients[0];
		Vector firstLineDirection = conveyorCoefficients[1] - firstLineOrigin;
		Vector firstLineProjection = projectPointToLine(firstLineOrigin, firstLineDirection, positionEstimation);
		//deltaCalculation
		double firstProjectionDistance = calculateDistance2D(positionEstimation, firstLineProjection);
		//check if object is projected in valid segment
		double firstProjStartDelta = calculateDistance2D(firstLineProjection, conveyorCoefficients[0]);
		double firstProjEndDelta = calculateDistance2D(firstLineProjection, conveyorCoefficients[1]);
		bool firstProjectionValid =  fabs( segmentLengths[0] - (firstProjStartDelta + firstProjEndDelta) ) < (0.01 + currentDistanceToProjection);

		//second line projection
		Vector secondLineOrigin = conveyorCoefficients[3];
		Vector secondLineDirection = conveyorCoefficients[4] - secondLineOrigin;
		Vector secondLineProjection = projectPointToLine(secondLineOrigin, secondLineDirection, positionEstimation);
		//delta calculation
		double secondProjectionDistance = calculateDistance2D(positionEstimation, secondLineProjection);
		//validation check
		double secondProjStartDelta = calculateDistance2D(secondLineProjection, conveyorCoefficients[3]);
		double secondProjEndDelta = calculateDistance2D(secondLineProjection, conveyorCoefficients[4]);
		bool secondProjectionValid = fabs( segmentLengths[2] - (secondProjStartDelta + secondProjEndDelta) ) < (0.01 + currentDistanceToProjection);

		//circular projection
		Vector circleCenter = conveyorCoefficients[2];
		Vector circleObjectDirection = positionEstimation - circleCenter;
		circleObjectDirection.normalize();	
		Vector scaledDirection = circleObjectDirection * conveyorCircleRadius;
		Vector circleProjection = circleCenter + scaledDirection;
		//delta calculation
		double circleProjectionDistance = fabs( calculateDistance2D(positionEstimation, circleProjection) );
		//validation check
		bool circleProjectionValid = !firstProjectionValid && !secondProjectionValid && (circleProjectionDistance < (0.05 + currentDistanceToProjection) );

		currentPositionRequestTime = ros::Time::now();
		currentPositionKnown = true;
		if ( circleProjectionValid ) { //object is on circular part
			currentSegmentPart = 2;
			currentProjectedPosition = circleProjection;
			currentDistanceToProjection = circleProjectionDistance;
			Vector circleStartDirection = conveyorCoefficients[1] - circleCenter;
			circleStartDirection.normalize();
			currentAngle = acos( circleStartDirection.dot(circleObjectDirection) );
			remainingSegmentPath = (M_PI - currentAngle) * conveyorCircleRadius;
			(conveyorVelocity < 0.005) ? visibleTimeLeft = 3600 : visibleTimeLeft = (remainingSegmentPath + segmentLengths[2]) / conveyorVelocity;
			double percentPassed = getPercentOnCircular( currentAngle );
			ROS_DEBUG("Frame %d: OBJECT_PROJECTION: Object is at %f%% on circular segment (%f seconds left visible).",
								objectTrackingFrame, percentPassed, visibleTimeLeft );
		} else if ( firstProjectionValid && (firstProjectionDistance <= secondProjectionDistance) ) { //object is on first linear part
			currentSegmentPart = 1;
			currentProjectedPosition = firstLineProjection;
			currentDistanceToProjection = firstProjectionDistance;
			currentAngle = 0.0;
			remainingSegmentPath = firstProjEndDelta;
			(conveyorVelocity < 0.005) ? visibleTimeLeft = 3600 : visibleTimeLeft = (remainingSegmentPath + segmentLengths[1] + segmentLengths[2]) / conveyorVelocity;
			double percentPassed = getPercentOnLinear( remainingSegmentPath, segmentLengths[0] );
			ROS_DEBUG("Frame %d: OBJECT_PROJECTION: Object is at %f%% on first linear segment (%f seconds left visible).",
					objectTrackingFrame, percentPassed, visibleTimeLeft );
		} else if ( secondProjectionValid ){ //object is on second linear part
			currentSegmentPart = 3;
			currentProjectedPosition = secondLineProjection;
			currentDistanceToProjection = secondProjectionDistance;
			currentAngle = 180.0;
			remainingSegmentPath = secondProjEndDelta;
			(conveyorVelocity < 0.005) ? visibleTimeLeft = 3600 : visibleTimeLeft = remainingSegmentPath / conveyorVelocity;
			double percentPassed = getPercentOnLinear( remainingSegmentPath, segmentLengths[2] );
			ROS_DEBUG("Frame %d: OBJECT_PROJECTION: Object is at %f%% on second linear segment (%f seconds left visible).",
					objectTrackingFrame, percentPassed, visibleTimeLeft );
		} else { //object currently out of bounds!
			ROS_ERROR("Frame %d: OBJECT_PROJECTION: Object currently not detected on valid conveyor trajectory!",
					objectTrackingFrame );
			currentProjectedPosition = error;
			currentDistanceToProjection = 99.9;
		}


		ros::Duration visibleTimeInSec(visibleTimeLeft);		
		terminationTime = currentPositionRequestTime + visibleTimeInSec;		
	}
  
}//Conveyor_trajectory::calculateCurrentObjectPositionParameters

std::vector<Eigen::Vector4f> Conveyor_trajectory::generateTrajectory( double objectHeight, double verticalApproachDistance, double interceptionTime, double robotTactTime, ros::Duration gripperApproachTime, ros::Duration grippingTime ) {
	
	int lowerLiftTacts = ceil( gripperApproachTime.toSec() /robotTactTime);
	int grippingTacts = ceil( grippingTime.toSec() /robotTactTime);
	int requiredTacts = lowerLiftTacts + grippingTacts + lowerLiftTacts; //lower gripper, grip and then lift it with the gripped object
	double gripperHeight = objectHeight + verticalApproachDistance;
	ROS_WARN("Distance to ConveyorProjection is %f meter.", currentDistanceToProjection);
	
	std::vector<Eigen::Vector4f> trajectoryPoints;
	
	if ( !positionParametersValid() ) { 
		ROS_WARN("Generating trajectory not possible at this time, as the object could not be detected on valid conveyor segment!");
	} else {
	
		int trajectoryTacts = floor( ( getVisibleTimeLeft() - interceptionTime ) / robotTactTime );
		ROS_INFO("GenerateTrajectory triggered for interception in %f seconds!\n" 
				"With a tact time of %f sec. the object will be visible for %d tacts after interception", 
				interceptionTime, robotTactTime, trajectoryTacts);
		
		VectorRot interceptionPointRot = getFutureObjectPosition( interceptionTime );
		interceptionPointRot[2] = gripperHeight;
		Vector interceptionPoint = Vector(interceptionPointRot[0], interceptionPointRot[1], interceptionPointRot[2]);
		
		
		
		if (interceptionPoint == error) {
			ROS_WARN( "Object will have surpassed visible trajectory at requested interceptionTime!" );
		} else if (trajectoryTacts < requiredTacts) {
			ROS_WARN( "Object not visible for enough time to be successfully gripped!\n"
				  "The robot can only track the object on the conveyor for %d tacts (%d required for gripping).", 
				  trajectoryTacts, requiredTacts); 
		} else {
			

			VectorRot interceptionVelocityPoint = getFutureObjectPosition( interceptionTime + robotTactTime );
			VectorRot interceptionVelocityDir = interceptionVelocityPoint - interceptionPointRot;
			interceptionVelocityDir *= (1.0 / robotTactTime);			
			
			double zvelocity = -1.0 * verticalApproachDistance / gripperApproachTime.toSec();
			double verticalApproachDistanceDelta = (1.0 / lowerLiftTacts) * verticalApproachDistance;
			
			ROS_INFO_ONCE("GripperHeight is %f, distanceDelta is %f", gripperHeight, verticalApproachDistanceDelta);
			//convert into Vector4f to contain rotation in 4th dimension
			Eigen::Vector4f interceptionDir( interceptionVelocityDir[0],  interceptionVelocityDir[1], zvelocity, interceptionVelocityDir[4]); 
			trajectoryPoints.push_back(interceptionDir);
			trajectoryPoints.push_back(interceptionPointRot);

			for (int i = 1; i < requiredTacts + 1; i++) {
				double requestTime = interceptionTime + i * robotTactTime;
				VectorRot futurePos = getFutureObjectPosition( requestTime );
				
				if (i <= lowerLiftTacts ) { //lower the gripper -> decrease height
					gripperHeight -= verticalApproachDistanceDelta;
				} else if ( i > (lowerLiftTacts + grippingTacts) ) { //lift the gripper again -> increase height
					gripperHeight += verticalApproachDistanceDelta;
				}
					futurePos[2] = gripperHeight;
				
				
				trajectoryPoints.push_back( futurePos );
			}
		  
		}

			
	}
	return trajectoryPoints;
  
}//Conveyor_trajectory::generateTrajectory

bool Conveyor_trajectory::positionParametersValid() {
	bool positionIsOnConveyor = (currentProjectedPosition != error);
	double parameterAge = ros::Time::now().toSec() - currentPositionRequestTime.toSec();
	return ( positionIsOnConveyor && ( parameterAge < 2.0 ) );
}//Conveyor_trajectory::positionParametersValid

void Conveyor_trajectory::updateConveyorVelocity( double objectVelocity ) {

	conveyorVelocity = objectVelocity;
}//Conveyor_trajectory::setConveyorVelocity

void Conveyor_trajectory::updateFrameCounter( int frameCounter ) {

	objectTrackingFrame = frameCounter;
}//Conveyor_trajectory::updateFrameCounter

//conveyor trajectory storage/retrieval functions
void Conveyor_trajectory::publishTrajectoryToRosparam() {
  
	ROS_INFO("Publishing conveyorTrajectory parameters onto parameter server");
	std::vector< std::vector<double> > positions(5);
	
	nh.setParam( name_id + "/firstSegment/lineStart", convertEigenPosition( conveyorCoefficients[0] ) );
	nh.setParam( name_id + "/firstSegment/lineEnd", convertEigenPosition( conveyorCoefficients[1] ) );
	nh.setParam( name_id + "/secondSegment/circleCenter", convertEigenPosition( conveyorCoefficients[2] ) );
	nh.setParam( name_id + "/secondSegment/circleRadius", conveyorCircleRadius );
	nh.setParam( name_id + "/thirdSegment/lineStart", convertEigenPosition( conveyorCoefficients[3] ) );
	nh.setParam( name_id + "/thirdSegment/lineEnd", convertEigenPosition( conveyorCoefficients[4] ) );	
	
	nh.setParam( name_id + "/segmentLengths", segmentLengths);

	int x = system("rosparam dump `rospack find conveyor_object_tracking`/config/conveyor_trajectory.yaml /conveyorTrajectory");
}//Conveyor_trajectory::publishTrajectoryToRosparam

bool Conveyor_trajectory::fetchTrajectoryParameters() {
  
	if ( nh.hasParam("/" + name_id ) ) {

		nh.getParam("/" + name_id + "/segmentLengths", segmentLengths);
		nh.getParam("/" + name_id + "/secondSegment/circleRadius", conveyorCircleRadius );
		std::vector<double> position(3);
		nh.getParam("/" + name_id + "/firstSegment/lineStart", position );
		conveyorCoefficients[0] = convertDoubleToEigenPosition(position);
		nh.getParam("/" + name_id + "/firstSegment/lineEnd", position );
		conveyorCoefficients[1] = convertDoubleToEigenPosition(position);
		
		nh.getParam("/" + name_id + "/secondSegment/circleCenter", position );
		conveyorCoefficients[2] = convertDoubleToEigenPosition(position);
		
		nh.getParam("/" + name_id + "/thirdSegment/lineStart", position );
		conveyorCoefficients[3] = convertDoubleToEigenPosition(position);
		nh.getParam("/" + name_id + "/thirdSegment/lineEnd", position );
		conveyorCoefficients[4] = convertDoubleToEigenPosition(position);

		recordingActive = false;
		degreeOfCompletion = 3;
		lastObjectTime = ros::Time::now();
		lastObjectPosition = error;

		Vector firstLineDirection = conveyorCoefficients[1] - conveyorCoefficients[0];
		firstLineDirection.normalize();
		firstLineOrthogonal = Vector(-1.0 * firstLineDirection[1], firstLineDirection[0], 0.0);
		Vector secondLineDirection = conveyorCoefficients[4] - conveyorCoefficients[3];
		secondLineDirection.normalize();
		secondLineOrthogonal = Vector(-1.0 * secondLineDirection[1], secondLineDirection[0], 0.0);
		
		ROS_INFO("Successfully fetched previous conveyor trajectory data!");
		return true;
	} else {
		ROS_ERROR("Fetching conveyor trajectory data not possible!");
		return false;
	}
  
}//Conveyor_trajectory::fetchTrajectoryParameters


//getters
Eigen::Vector4f Conveyor_trajectory::getFutureObjectPosition( double timeInSec ) {

	double pathLength = timeInSec * conveyorVelocity; //length in meters
	if ( !positionParametersValid() ) {
		ROS_ERROR("Frame %d: OBJECT_PROJECTION: call while current object position paramaters are not set!",
				objectTrackingFrame);
		return errorRot;
	} else if ( pathLength > segmentLengths[3] ) {
		ROS_ERROR("Frame %d: OBJECT_PROJECTION: call with improper variables: "
				"In %f seconds the object will have surpassed visible conveyor trajectory path!",
				objectTrackingFrame, timeInSec);
		return errorRot;
	} else {
		
		//projection of object position onto first line
		Vector firstLineOrigin = conveyorCoefficients[0];
		Vector firstLineDirection = conveyorCoefficients[1] - firstLineOrigin;
		firstLineDirection.normalize();
		Vector firstLineProjection = projectPointToLine(firstLineOrigin, firstLineDirection, currentProjectedPosition);
		//second line projection
		Vector secondLineOrigin = conveyorCoefficients[3];
		Vector secondLineDirection = conveyorCoefficients[4] - secondLineOrigin;
		secondLineDirection.normalize();
		Vector secondLineProjection = projectPointToLine(secondLineOrigin, secondLineDirection, currentProjectedPosition);
		Vector futureProjectedPosition;
		Vector futureEstimatedPosition;
		if ( currentSegmentPart == 1 ) { //currently on first linear part
			ROS_INFO_ONCE("OBJECT_PROJECTION: Currently on first linear part");
			if ( pathLength <= remainingSegmentPath ) { //future position lies in first segment
				ROS_DEBUG("Frame %d: OBJECT_PROJECTION: In %f seconds the object will be  at %f%% on the first linear segment!",
						objectTrackingFrame, timeInSec, getPercentOnLinear( (remainingSegmentPath - pathLength) , segmentLengths[0] ) );
				futureProjectedPosition = currentProjectedPosition + pathLength * firstLineDirection;
				futureEstimatedPosition = futureProjectedPosition + currentDistanceToProjection * firstLineOrthogonal;
				return convertToVectorWithRotation( futureEstimatedPosition, 0.0 );
				
			} else if ( pathLength <= (remainingSegmentPath + segmentLengths[1]) ) { //future position lies in circular segment
				pathLength -= remainingSegmentPath;
				double futurePointAngle = pathLength / (conveyorCircleRadius + currentDistanceToProjection);
				double futureAngleDeg = getPercentOnCircular(futurePointAngle) * 1.80;
				ROS_INFO("Frame %d: OBJECT_PROJECTION: In %f seconds the object will be at %f%% on the circular segment!",
						objectTrackingFrame, timeInSec, getPercentOnCircular(futurePointAngle) );
				futureProjectedPosition = projectOnCircle( futurePointAngle, conveyorCircleRadius );
				futureEstimatedPosition = projectOnCircle( futurePointAngle, (conveyorCircleRadius + currentDistanceToProjection) );
				return convertToVectorWithRotation( futureEstimatedPosition, futureAngleDeg);
			} else if ( pathLength <= (remainingSegmentPath + segmentLengths[1] + segmentLengths[2]) ) {
				pathLength -= (remainingSegmentPath + segmentLengths[1]);
				ROS_DEBUG("Frame %d: OBJECT_PROJECTION: In %f seconds the object will be at %f%% on the second linear segment!",
						objectTrackingFrame, timeInSec, getPercentOnLinear( (segmentLengths[2] - pathLength) , segmentLengths[2] ) );
				futureProjectedPosition = secondLineOrigin + pathLength * secondLineDirection;
				futureEstimatedPosition = futureProjectedPosition + currentDistanceToProjection * secondLineOrthogonal;
				return convertToVectorWithRotation( futureEstimatedPosition, 180.0);
			} else {
				ROS_ERROR("Frame %d: OBJECT_PROJECTION: In %f seconds the object will have surpassed visible conveyor trajectory path! \n"
						"Object only visibile in next %f seconds!",
						objectTrackingFrame, timeInSec, visibleTimeLeft );
				return errorRot;
			}

		} else if ( currentSegmentPart == 2 ) { //currently on circular part
			ROS_INFO_ONCE("OBJECT_PROJECTION: Currently on circular part!");
			Vector circleStartDirection = conveyorCoefficients[1] - conveyorCoefficients[2];
			circleStartDirection.normalize();
			Vector circleObjectDirection = currentProjectedPosition - conveyorCoefficients[2];
			circleObjectDirection.normalize();
			double currentAngle = acos( circleStartDirection.dot(circleObjectDirection) );

			if ( pathLength <= remainingSegmentPath ) { //future position lies in same segment
				double rotationAngle = (pathLength / (conveyorCircleRadius + currentDistanceToProjection) ) + currentAngle;
				ROS_INFO("Frame %d: OBJECT_PROJECTION: In %f seconds the object will be at %f%% on the circular segment!",
						objectTrackingFrame, timeInSec, getPercentOnCircular( rotationAngle ) );
				double futureAngleDeg = getPercentOnCircular(rotationAngle) * 1.80;
				futureProjectedPosition = projectOnCircle( rotationAngle, conveyorCircleRadius );
				futureEstimatedPosition = projectOnCircle( rotationAngle, (conveyorCircleRadius + currentDistanceToProjection) );
				return convertToVectorWithRotation( futureEstimatedPosition, futureAngleDeg);
			} else if ( pathLength <= (remainingSegmentPath + segmentLengths[2]) ) { //future position lies in second linear segment
				pathLength -= remainingSegmentPath;
				ROS_DEBUG("Frame %d: OBJECT_PROJECTION: In %f seconds the object will be at %f%% on the second linear segment!",
						objectTrackingFrame, timeInSec, getPercentOnLinear( (segmentLengths[2] - pathLength) , segmentLengths[2] ) );
				futureProjectedPosition = secondLineOrigin + pathLength * secondLineDirection;
				futureEstimatedPosition = futureProjectedPosition + currentDistanceToProjection * secondLineOrthogonal;
				return convertToVectorWithRotation( futureEstimatedPosition, 180.0);
			} else { // future position outside visible segment
				ROS_ERROR("Frame %d: OBJECT_PROJECTION: In %f seconds the object will have surpassed visible conveyor trajectory path! \n"
						"Object only visibile in next %f seconds!",
						objectTrackingFrame, timeInSec, visibleTimeLeft);
				return errorRot;
			}

		} else if ( currentSegmentPart == 3 ) { //currently on second linear part

			if ( pathLength <= remainingSegmentPath ) {
				ROS_DEBUG("Frame %d: OBJECT_PROJECTION: In %f seconds the object will be at %f%% on the second linear segment!",
						objectTrackingFrame, timeInSec, getPercentOnLinear( (segmentLengths[2] - pathLength) , segmentLengths[2] ) );
				futureProjectedPosition = secondLineProjection + pathLength * secondLineDirection;
				futureEstimatedPosition = futureProjectedPosition + currentDistanceToProjection * secondLineOrthogonal;
				return convertToVectorWithRotation( futureEstimatedPosition, 180.0);
			} else {
				ROS_ERROR("Frame %d: OBJECT_PROJECTION: In %f seconds the object will have surpassed visible conveyor trajectory path! \n"
						"Object only visibile in next %f seconds!",
						objectTrackingFrame, timeInSec, visibleTimeLeft );
				return errorRot;
			}

		} else { //object currently out of bounds!
			ROS_ERROR("Frame %d: OBJECT_PROJECTION: Object currently not detected on valid conveyor trajectory!",
					objectTrackingFrame );
			return errorRot;
		}
	}
}//Conveyor_trajectory::getObjectProjectionOnTrajectory

double Conveyor_trajectory::getVelocity() {
	return conveyorVelocity;
}//Conveyor_trajectory::getVelocity

double Conveyor_trajectory::getVisibleTimeLeft() {
	
	if ( positionParametersValid() ) {
		double timeSinceParameterCalculation = ( ros::Time::now().toSec() - currentPositionRequestTime.toSec() );
		return ( visibleTimeLeft - timeSinceParameterCalculation );
	} else {
		return -1;
	}  
}//Conveyor_trajectory::getVisibleTimeLeft()

ros::Time Conveyor_trajectory::getTerminationTime() {
	return terminationTime;
}//Conveyor_trajectory::getTerminationTime()

ros::Time Conveyor_trajectory::getParameterTime() {
	return currentPositionRequestTime;
}//Conveyor_trajectory::getParameterTime()

double Conveyor_trajectory::getAngleTowardsObjectDirection( Vector objectCenter, Vector handleDirection ) {
  
	if ( !trajectoryCompleted() ) {
		return 0.0;
	} else if ( (ros::Time::now() - lastObjectTime) > ros::Duration(1.0) ) {
	      updateTrajectory(objectCenter); 
	}
	Vector firstLineDirection = conveyorCoefficients[1] - conveyorCoefficients[0];
	firstLineDirection.normalize();
	double angleTowardsFirstLine = ( acos( firstLineDirection.dot(handleDirection) ) * 180 ) / M_PI;
	return currentAngle - angleTowardsFirstLine;
	
}//Convevor_trajectory::getAngleTowardsObjectDirection()

double Conveyor_trajectory::getPercentOnCircular( double angle ) {
	return ( 100.0 * angle / M_PI );
}//Conveyor_trajectory::getPercentOnCircular

double Conveyor_trajectory::getPercentOnLinear( double remainingLength, double totalLength ) {
	return ( 100 * ( 1.0 - (remainingLength / totalLength) ) );
}//Conveyor_trajectory::getPercentOnLinear


//help functions
visualization_msgs::MarkerArray Conveyor_trajectory::visualizeTrajectory() {

	int circleResolution = 40;

	conveyorMarkers.markers.clear();//delete markers from last frame

	//initialize Sphere marker
	visualization_msgs::Marker trajSphereMarker;
	trajSphereMarker.header.frame_id = "conveyor_plane";
	trajSphereMarker.header.stamp = ros::Time();
	trajSphereMarker.ns = "conveyor_object_tracker";
	trajSphereMarker.id = 90002;
	trajSphereMarker.type = visualization_msgs::Marker::SPHERE_LIST;
	trajSphereMarker.action = visualization_msgs::Marker::ADD;
	trajSphereMarker.lifetime = visualizationLifetime;
	trajSphereMarker.color.r = 0.0;
	trajSphereMarker.color.g = 1.0;
	trajSphereMarker.color.b = 0.0;
	trajSphereMarker.color.a = 0.8;
	trajSphereMarker.scale.x = 0.02;
	trajSphereMarker.scale.y = 0.02;
	trajSphereMarker.scale.z = 0.02;
	
	//initialize trajectory marker
	visualization_msgs::Marker finishedTrajectoryMarker;
	finishedTrajectoryMarker.header.frame_id = "conveyor_plane";
	finishedTrajectoryMarker.header.stamp = ros::Time();
	finishedTrajectoryMarker.ns = "conveyor_object_tracker";
	finishedTrajectoryMarker.id = 90001;
	finishedTrajectoryMarker.type = visualization_msgs::Marker::LINE_STRIP;
	finishedTrajectoryMarker.action = visualization_msgs::Marker::ADD;
	finishedTrajectoryMarker.lifetime = visualizationLifetime;
	finishedTrajectoryMarker.color.r = 0.0;
	finishedTrajectoryMarker.color.g = 1.0;
	finishedTrajectoryMarker.color.b = 0.0;
	finishedTrajectoryMarker.color.a = 0.8;
	finishedTrajectoryMarker.scale.x = 0.005;//controls width of line strip segments

	if ( degreeOfCompletion > 0 ) {
		Vector firstLineStart = conveyorCoefficients[0];
		Vector firstLineEnd = conveyorCoefficients[1];

		//add first linear segment
		
		finishedTrajectoryMarker.points.resize( 2 );
		finishedTrajectoryMarker.points[0].x = firstLineStart[0];
		finishedTrajectoryMarker.points[0].y = firstLineStart[1];
		finishedTrajectoryMarker.points[0].z = OBJECT_CONVEYOR_OFFSET;
		finishedTrajectoryMarker.points[1].x = firstLineEnd[0];
		finishedTrajectoryMarker.points[1].y = firstLineEnd[1];
		finishedTrajectoryMarker.points[1].z = OBJECT_CONVEYOR_OFFSET;


		//add first two points for sphere
		trajSphereMarker.points.resize( 2 );
		trajSphereMarker.points[0].x = firstLineStart[0];
		trajSphereMarker.points[0].y = firstLineStart[1];
		trajSphereMarker.points[0].z = OBJECT_CONVEYOR_OFFSET;
		trajSphereMarker.points[1].x = firstLineEnd[0];
		trajSphereMarker.points[1].y = firstLineEnd[1];
		trajSphereMarker.points[1].z = OBJECT_CONVEYOR_OFFSET;

		if ( degreeOfCompletion > 1 ) {
			Vector circleCenter = conveyorCoefficients[2];

			//add circular segment
			finishedTrajectoryMarker.points.resize( circleResolution + 2 );
			for (int i = 0; i < circleResolution; i++) {
				double angle = ( (double) i / (circleResolution - 1) ) * M_PI;
				Vector pointOnTrajectory = projectOnCircle( angle, conveyorCircleRadius );
				finishedTrajectoryMarker.points[i + 2].x = pointOnTrajectory[0];
				finishedTrajectoryMarker.points[i + 2].y = pointOnTrajectory[1];
				finishedTrajectoryMarker.points[i + 2].z = OBJECT_CONVEYOR_OFFSET;
			}

			//add circle center to sphere marker
			trajSphereMarker.points.resize( 3 );
			trajSphereMarker.points[2].x = circleCenter[0];
			trajSphereMarker.points[2].y = circleCenter[1];
			trajSphereMarker.points[2].z = OBJECT_CONVEYOR_OFFSET;
		}
	}
	
	if ( trajectoryCompleted() ) {
		Vector secondLineStart = conveyorCoefficients[3];
		Vector secondLineEnd = conveyorCoefficients[4];
		//add second linear segment
		finishedTrajectoryMarker.points.resize( circleResolution + 4 );
		finishedTrajectoryMarker.points[circleResolution + 2 ].x = secondLineStart[0];
		finishedTrajectoryMarker.points[circleResolution + 2].y = secondLineStart[1];
		finishedTrajectoryMarker.points[circleResolution + 2].z = OBJECT_CONVEYOR_OFFSET;
		finishedTrajectoryMarker.points[circleResolution + 3].x = secondLineEnd[0];
		finishedTrajectoryMarker.points[circleResolution + 3].y = secondLineEnd[1];
		finishedTrajectoryMarker.points[circleResolution + 3].z = OBJECT_CONVEYOR_OFFSET;

		//add second line endpoints to sphere marker
		trajSphereMarker.points.resize( 5 );
		trajSphereMarker.points[3].x = secondLineStart[0];
		trajSphereMarker.points[3].y = secondLineStart[1];
		trajSphereMarker.points[3].z = OBJECT_CONVEYOR_OFFSET;
		trajSphereMarker.points[4].x = secondLineEnd[0];
		trajSphereMarker.points[4].y = secondLineEnd[1];
		trajSphereMarker.points[4].z = OBJECT_CONVEYOR_OFFSET;
		
	} else {//add recorded points for incomplete segment
	  
		visualization_msgs::Marker recordedPointsMarker;
		recordedPointsMarker.header.frame_id = "conveyor_plane";
		recordedPointsMarker.header.stamp = ros::Time();
		recordedPointsMarker.ns = "conveyor_object_tracker";
		recordedPointsMarker.id = 90000;
		recordedPointsMarker.type = visualization_msgs::Marker::LINE_STRIP;
		recordedPointsMarker.action = visualization_msgs::Marker::ADD;
		recordedPointsMarker.lifetime = ros::Duration(2.0);
		recordedPointsMarker.color.r = 0.0;
		recordedPointsMarker.color.g = 1.0;
		recordedPointsMarker.color.b = 0.0;
		recordedPointsMarker.color.a = 0.8;
		recordedPointsMarker.scale.x = 0.005;//controls width of line strip segments
		recordedPointsMarker.points.resize( objectPositionPoints.size() );

		for (size_t i = 0; i < objectPositionPoints.size(); ++i ) {
			Vector currentPoint = objectPositionPoints.at(i);
			recordedPointsMarker.points[i].x = currentPoint[0];
			recordedPointsMarker.points[i].y = currentPoint[1];
			recordedPointsMarker.points[i].z = OBJECT_CONVEYOR_OFFSET;
		}
		conveyorMarkers.markers.push_back(recordedPointsMarker);
	  
	}

	conveyorMarkers.markers.push_back(finishedTrajectoryMarker);
	conveyorMarkers.markers.push_back(trajSphereMarker);

	return conveyorMarkers;
}//Conveyor_trajectory::visualizeTrajectory

double Conveyor_trajectory::calculateDistance2D( Vector first, Vector second ) {
	return sqrt( pow( first[0] - second[0], 2) + pow( first[1] - second[1], 2) );
}//Conveyor_trajectory::calculateDistance2D

bool Conveyor_trajectory::clockwiseRotation( Vector first, Vector second ) {
	return ( (first[0] * second[1] - first[1] * second[0]) <= 0 );
}//Conveyor_trajectory::clockwiseRotation

Eigen::Vector3f Conveyor_trajectory::projectPointToLine( Vector lineOrigin, Vector lineDirection, Vector point ) {
	lineDirection.normalize();
	return lineOrigin + lineDirection.dot( point - lineOrigin ) * lineDirection;
}//Conveyor_trajectory::projectPointToLine

Eigen::Vector3f Conveyor_trajectory::projectOnCircle( double angleInRad, double radius ) {

	if ( degreeOfCompletion < 1 ) {
		ROS_ERROR("Frame %d: FUNCTION projectOnCircle CALLED WHILE CIRCULAR PATH NOT YET ESTIMATED!!",
				  objectTrackingFrame);
		return error;
	} else if ( (angleInRad > M_PI) || (angleInRad < 0.0) ) {
		ROS_WARN("Frame %d: FUNCTION projectOnCircle CALLED WITH INVALID ANGLE (%f rad)!\n"
						"Returning 180deg vector instead!", objectTrackingFrame, angleInRad);
		angleInRad = M_PI;
	}
	Vector circleStartVector = (conveyorCoefficients[1] - conveyorCoefficients[2]) * ( radius / conveyorCircleRadius );
	Vector translationToConveyorframe = conveyorCoefficients[2];
	Eigen::Quaternionf rotation;
	rotation.w() = cos( -angleInRad / 2.0 );
	rotation.x() = 0.0;
	rotation.y() = 0.0;
	rotation.z() = sin( -angleInRad / 2.0 );

	Eigen::Quaternionf ID;
	ID.setIdentity();

	Point_cloud_ptr inputCloud(new Point_cloud);
	Point toBeTransformed;
	toBeTransformed.x = circleStartVector[0];
	toBeTransformed.y = circleStartVector[1];
	toBeTransformed.z = circleStartVector[2];
	inputCloud->push_back( toBeTransformed );
	pcl::transformPointCloud( *inputCloud, *inputCloud, translationToConveyorframe, rotation );
	Point transformedPoint = inputCloud->front();
	return transformedPoint.getVector3fMap();
}//Conveyor_trajectory::projectOnCircle

//conversion help functions
Eigen::Vector4f Conveyor_trajectory::convertToVectorWithRotation( Vector position, double rotationInDeg ) {
  
	return VectorRot(position[0], position[1], position[2], rotationInDeg);
}//Conveyor_trajectory::convertToVectorWithRotation

std::vector<double> Conveyor_trajectory::convertEigenPosition( Vector input ) {
  
  std::vector<double> output(3);
  output[0] = input[0];
  output[1] = input[1];
  output[2] = input[2];
  return output;
  
}//Conveyor_trajectory::convertEigenPosition

Eigen::Vector3f Conveyor_trajectory::convertDoubleToEigenPosition( std::vector<double> input ) {
  
  return Vector(input[0], input[1], input[2]);
  
}//Conveyor_trajectory::convertDoubleToEigenPosition


}//namespace Conveyor_object_tracking

