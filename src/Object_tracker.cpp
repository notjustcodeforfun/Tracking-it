
//#include <functional>

#include "../include/Object_tracker.h"

#define INFO_RECONSTRUCT ( false )


namespace Conveyor_object_tracking {

Object_tracker::Object_tracker ()
      : tracked_objects( 0 ),
	observed_objects( 0 ),
	conveyorVelocity(99.9), //value in meter/second
	search_tree( new pcl::search::KdTree<Point> ),
	counter(0),
	start(-1),
	end(-1),
	fEnd( ros::TIME_MAX ),
	totalTime( ros::Duration(0.0) ),
	objectDetectedCounter(0),
	objectFrameCalcCtr(-1),
	multiThreaded(true),
	multiObjectTracking(false),
	useObjectFrame(false),
	conveyorTrajectory(), //object holding all information and functions about the trajectory
	objectWRLCreated( false ),
	robotTrajectoryActionClient("FollowObject", true),
	robot_baselink("kr5_base_link"), //name of the robot baselink
	robotActive(false), //whether the robot has started following the object
	gripperActive(false), //whether the gripper is in use (gripping or holding an object)
	followObjectMode(false), //one-time-trigger to switch robot mode to followObject
	pauseTracking(false) //set to true when robot is holding the object to prevent tracking objects not on the conveyor
	
	{
	
	/*Initialize all variables from the yaml config file*/
	std::string nodeID = "object_tracking_node/";
	
	nh.param(nodeID + "multiThreaded", multiThreaded, true);
	nh.param(nodeID + "multiObjectTracking", multiObjectTracking, false);
	{ //pointcloud clustering
		nh.param(nodeID + "clustersize_min", clustersize_min, 1200);
		nh.param(nodeID + "clustersize_max", clustersize_max, 3500);
		nh.param(nodeID + "cluster_min_distance", cluster_min_distance, 0.15);
		nh.param(nodeID + "cluster_min_height", cluster_min_height, 0.04);
	}
	{ //object recognition and reconstruction
		nh.param(nodeID + "object_minheight", object_minheight, 0.08);
		nh.param(nodeID + "object_maxwidth", object_maxwidth, 0.1);
		nh.param(nodeID + "object_handle_minsize", handleMinsize, 20);
		nh.param(nodeID + "object_conveyor_offset", object_conveyor_offset, 0.05);
		nh.param(nodeID + "reconstruct_res", reconstruct_res, 32);
		nh.param(nodeID + "circle_segment_height", circle_segment_height, 0.015);
		if ( nh.hasParam(nodeID + "circle_segment_min_points") ) {
			int circSegMinPointValue;
			nh.getParam(nodeID + "circle_segment_min_points", circSegMinPointValue);
			circle_segment_min_points = (size_t) circSegMinPointValue;
		} else {
			circle_segment_min_points = 85;
		}
		nh.param(nodeID + "matching_distance_threshold", matching_distance_threshold, 0.1);
		nh.param(nodeID + "object_reconstruction_angle", object_reconstruction_angle, 22.5);
		nh.param(nodeID + "voxel_grid_size", voxel_grid_size, 0.001);
		nh.param(nodeID + "icp_slice_height", icp_slice_height, 0.01);
		nh.param(nodeID + "historyLength", historyLength, 30);
		if ( nh.hasParam(nodeID + "wrlFilePath") ) {
			XmlRpc::XmlRpcValue pathRpc;
			nh.getParam(nodeID + "wrlFilePath", pathRpc);
			wrlPath = (string) pathRpc;
		} else {
			wrlPath = "../svn-KUKAOR/ORData/Models/Objects/objectTracking/rotationalBody.wrl";
		}
	}
	{ //conveyor recognition
	      nh.param(nodeID + "recordConveyor", recordConveyorTrajectory, true); 
	}
	{ //robot communication
		nh.param(nodeID + "simulated", simulated, true);
		nh.param(nodeID + "robot_tact_time", robotTactTime, 0.012);
		nh.param(nodeID + "robot_delay", robotDelay, 0.5);
	}
	{ //object gripping
		nh.param(nodeID + "interceptionTime", interceptionTime, 5.0 );
		if ( nh.hasParam(nodeID + "gripperApproachTime") ) {
			double approachTimeValue;
			nh.getParam(nodeID + "gripperApproachTime", approachTimeValue);
			gripperApproachTime = ros::Duration( approachTimeValue);
		} else {
			gripperApproachTime = ros::Duration(2.5);
		}
		if ( nh.hasParam(nodeID + "grippingTime") ) {
			double grippingTimeValue;
			nh.getParam(nodeID + "grippingTime", grippingTimeValue);
			grippingTime = ros::Duration( grippingTimeValue);
		} else {
			grippingTime = ros::Duration(4.5);
		}
		nh.param(nodeID + "verticalApproachDistance", verticalApproachDistance, 0.125);
		nh.param(nodeID + "gripperVerticalTCPStandoff", gripperVerticalTCPStandoff, 0.05);
		nh.param(nodeID + "fixedTCPRotation", fixedTCPRotation, 0.0);
		if ( nh.hasParam(nodeID + "initialRobotPose") ) {
			XmlRpc::XmlRpcValue initialPoseRpc;
			
			nh.getParam(nodeID + "initialRobotPose", initialPoseRpc);
			initialRobotPose.data.resize(initialPoseRpc.size());
			for (size_t i = 0; i < initialPoseRpc.size(); i++) {
				initialRobotPose.data[i] = initialPoseRpc[i];
			}
		} else {
			initialRobotPose.data.resize(6);
			initialRobotPose.data[0] = 0;
			initialRobotPose.data[1] = -1.57079633;
			initialRobotPose.data[2] = 1.57079633;
			initialRobotPose.data[3] = 0;
			initialRobotPose.data[4] = 1.57079633;
			initialRobotPose.data[5] = 1.57079633;
		}
		if ( nh.hasParam(nodeID + "openGripperPose") ) {
			XmlRpc::XmlRpcValue gripperPose;
			nh.getParam(nodeID + "openGripperPose", gripperPose);
			openPosition120.data.resize(gripperPose.size());
			for (size_t i = 0; i < gripperPose.size(); i++) {
				openPosition120.data[i] = gripperPose[i];
			}
		} else {
			openPosition120.data.resize(7);
			openPosition120.data[0] = 0.70711;
			openPosition120.data[1] = -1.57;
			openPosition120.data[2] = 0;
			openPosition120.data[3] = -1.57;
			openPosition120.data[4] = 0;
			openPosition120.data[5] = -1.57;
			openPosition120.data[6] = 0;
		}
		if ( nh.hasParam(nodeID + "grippingCommandEffort") ) {
			double commandEffort;
			nh.getParam(nodeID + "openGripperPose", commandEffort);
			gripEffort.data = commandEffort;
		} else {
			gripEffort.data = 50.0;
		}
	}
	
	
	tfl = boost::shared_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(tfBuf));
	
	cluster_extractor = new pcl::EuclideanClusterExtraction<Point>;
	cluster_extractor->setClusterTolerance( 0.025 ); // max p2p distance in cluster
	cluster_extractor->setMinClusterSize( clustersize_min );
	cluster_extractor->setMaxClusterSize( clustersize_max );
	cluster_extractor->setSearchMethod( search_tree );
      
	sac = new pcl::SACSegmentation<pcl::PointXYZ>;
	sac->setModelType( pcl::SACMODEL_CIRCLE2D );
	sac->setMethodType( pcl::SAC_RANSAC );
	sac->setMaxIterations( 7 );
	sac->setDistanceThreshold( 0.01 );
	sac->setRadiusLimits( 0.015, 0.075 );
	sac->setOptimizeCoefficients( true );
    

}//Object_tracker::Object_tracker()

Object_tracker::~Object_tracker () {
    delete sac;
    delete cluster_extractor;
}//Object_tracker::~Object_tracker()

void Object_tracker::attach_to_node ( ros::NodeHandle& ros_node ) {
  
    nh = ros_node;
  
    points_subscriber = ros_node.subscribe( "conveyor_object_tracker/conveyorframe_object_points", 5, &Object_tracker::points_callback, this );
    reconstructed_object_marker_publisher = ros_node.advertise<visualization_msgs::MarkerArray>( "conveyor_object_tracker/object_parameters", 5 );
    tracked_object_publisher = ros_node.advertise<visualization_msgs::MarkerArray>( "conveyor_object_tracker/tracked_object_viz", 5 );
    cloud_publisher = ros_node.advertise<Point_cloud&>( "conveyor_object_tracker/object_cloud", 5 );
    filtered_object_symm_front_publisher = ros_node.advertise< Point_cloud_XYZ >( "conveyor_object_tracker/filtered_object_symm_front", 5);
    filtered_object_symm_back_publisher = ros_node.advertise< Point_cloud_XYZ >( "conveyor_object_tracker/filtered_object_symm_back", 5);
    filtered_object_nonsymm_publisher = ros_node.advertise< Point_cloud_XYZ >( "conveyor_object_tracker/filtered_object_nonsymm", 5);
    reconstructed_object_publisher = ros_node.advertise< Point_cloud_XYZ >( "conveyor_object_tracker/reconstructed_object", 5);
    prediction_publisher = ros_node.advertise< Point_cloud_XYZ >( "conveyor_object_tracker/prediction_cloud", 5 );
    scenePart_publisher = ros_node.advertise< Point_cloud_XYZ >( "conveyor_object_tracker/scenePart_cloud", 5 );
    reconstrucedObjectVisualization_publisher = ros_node.advertise<visualization_msgs::Marker>( "conveyor_object_tracker/reconstructed_object_viz", 5);
    conveyorTrajectoryVisualization_publisher = ros_node.advertise<visualization_msgs::MarkerArray>( "conveyor_object_tracker/conveyor_trajectory_viz", 5 );
    robotTrajectoryVisualization_publisher = ros_node.advertise<visualization_msgs::MarkerArray>( "conveyor_object_tracker/robot_trajectory_viz", 5 );
    gripperJointPosition_publisher = ros_node.advertise<std_msgs::Float64MultiArray>( "/sdh_controller/joint_group_position_controller/command", 5 );
    gripperPose_publisher = ros_node.advertise<std_msgs::Float64MultiArray>( "/rsi_node/set_joint_position", 5 );
    gripperEffort_publisher = ros_node.advertise<std_msgs::Float64>( "/sdh_controller/joint_group_effort_controller/command", 5 );
    export_speed_publisher = ros_node.advertise< std_msgs::Float64 >( "/conveyor_object_tracker/export/speed", 5 );
    export_object_param_publisher = ros_node.advertise<std_msgs::Float64MultiArray> ("/conveyor_object_tracker/export/objectParam", 5 );
    
    //fetch and finish conveyor trajectory if triggered!
    if ( !recordConveyorTrajectory ) {
	    conveyorTrajectory.fetchTrajectoryParameters();
    }
    
    
    //robot interaction
    robotTrajectoryActionClient.waitForServer();
    //initializes the robot pose - only for simulation
    ros::Duration(0.3).sleep();
    if (simulated) {
	      gripperPose_publisher.publish(initialRobotPose);
    }
    gripperJointPosition_publisher.publish( openPosition120 );
   
}//Object_tracker::attach_to_node()


void Object_tracker::points_callback ( const Point_cloud& input_cloud ) {
	if (input_cloud.empty()) {
		ROS_ERROR("Input cloud empty, but callback called!");
		return;
	} else if (pauseTracking) {
		ROS_INFO_ONCE("Robot currently returning gripped object, object tracking thus currently set inactive");
		return;
	}
	ros::Time fStart = ros::Time::now();//for fps calculation
	double currentFPS;
	double avgFPS;


	reconstructed_object_markers.markers.clear(); tracked_object_markers.markers.clear(); //clear old vizualization messages  todos
	Point_cloud_cptr scene_points( new Point_cloud( input_cloud ) );
	Point_cloud_ptr sp( new Point_cloud( input_cloud ) );

	std::vector<Point> sorted_points_scene( scene_points->points.begin(), scene_points->points.end() );    //typedef pcl::PointXYZRGB Point; in header
	std::sort( sorted_points_scene.begin(), sorted_points_scene.end(), Object_tracker::point_z_compareRGB); //top-down  从高到矮排列
	if ( sorted_points_scene.front().z < object_minheight)  { ROS_WARN("Object too small"); objectDetectedCounter = 0; return; }//If object smaller than minimum height: return
	Point_cloud_ptr object_cloud;
	std::vector<pcl::PointIndices> clustering;
	int obj_size = 0;    
	std::vector<Point_cloud_ptr> objectList;  // 每一帧，一个objectList 用来统计object数量

	if (objectDetectedCounter > historyLength) { //find object(s) in cloud by their predicted position 如果连续有效帧达到一定数量，则可以通过预测的方式来获取目标的points
		if (!multiObjectTracking) {			//非多个物体
			object_cloud = getClusteredScenePart(sp,predictedPos[0],predictedPos[1]);   //todos
			objectList.push_back(object_cloud);
			scenePart_publisher.publish(object_cloud);
			obj_size = object_cloud->points.size();
		} else {
			// for multipleObjects add every predictedPos to object_cloud before publishing
		}
	} else { //cluster the cloud to find object(s)
		search_tree->setInputCloud( scene_points);//scene_points is con.pointer,points to the object_points relative to conveyorframe
		cluster_extractor->setInputCloud( scene_points);
		cluster_extractor->extract( clustering );   //	std::vector<pcl::PointIndices> clustering; clustering是一个向量，每一个元素类型是PointIndices,其.indices为一个向量
		int cluster_counter = 0;

		for (auto cluster : clustering ){  //循环，cluster将分别从clustering的第一个元素到最后一个元素得到值
			if (!multiObjectTracking && cluster.indices.size() > obj_size) { // 此循环是找出数量最多的哪一个cluster（前提是非多个物体）
				Point_cloud_ptr object( new Point_cloud( *scene_points, cluster.indices ) );
				object->header.frame_id = "conveyor_plane";
				objectList.clear();//to delete possible lesser-point object in list （lesser较少的）
				objectList.push_back(object);
				obj_size = cluster.indices.size();
				cluster_counter++;
			} else if (multiObjectTracking && cluster.indices.size() > obj_size) {		//多个物体时，  ???>obj_size???
				Point_cloud_ptr object( new Point_cloud( *scene_points, cluster.indices ) );
				object->header.frame_id = "conveyor_plane";
				objectList.push_back(object);
				obj_size += cluster.indices.size();	// ？？？？？？？
				cluster_counter++;
			}
		}
	}
	

	if (objectList.empty() || (obj_size < (clustersize_min * objectList.size())) ) {//no Object detected in this frame
		
		if ( objectDetectedCounter == 0 ) {
		ROS_INFO("Unidentifiable object detected! (pointcloud is too small)");
		} else if ( objectDetectedCounter == -1 ) {
			ROS_INFO("Nothing detected.");
		}
		
		cloud_publisher.publish( input_cloud ); //publish whole input cloud
		for ( int i = 0; i < tracked_objects.size(); i++ ) {//all tracked objects unobserved in this frame as no object was detected
			tracked_objects[i].unobserved++;
		}
		//delete tracked objects after being unobserved for 'unobserved_frames_threshold' frames
		tracked_objects.erase( std::remove_if( tracked_objects.begin(), tracked_objects.end(), removeUnobserved ), tracked_objects.end() );

		useObjectFrame = false;
		objectDetectedCounter = 0;

		if ( !conveyorTrajectory.trajectoryCompleted() ) {
			conveyorTrajectory.finishRecording();
		}
		
		totalTime += ( ros::Time::now() - fStart );
	} else {
		Point_cloud FullObjCloud;
		for (int i = 0; i < objectList.size(); i++) {
			Point_cloud_ptr tempObj = objectList.at(i);
			Point_cloud_ptr observedObject = calculate_object_parameters( tempObj );  //todos (position, slice radii, height, solidness, wall thickness (if non-solid)
			if ( observedObject != nullptr ) {
				ROS_INFO("ObjectParameters calculated");
				FullObjCloud += *observedObject;
			}
		
		}
		object_cloud = FullObjCloud.makeShared();
		object_cloud->header.frame_id = "conveyor_plane"; //每一帧一个objectcloud用来储存object的parameter


		// match found objects with tracked ones
		match_objects();    //todos

		if (multiObjectTracking) {
			ROS_INFO("Frame %d: %d different object(s) detected.", objectDetectedCounter, (int) tracked_objects.size());
		} else {
			ROS_INFO("Frame %d: Object detected.", objectDetectedCounter);
		}
		
		if (useObjectFrame) {
			ROS_DEBUG("Frame %d: ObjectFrame calculated.", objectDetectedCounter);
		} else {
			ROS_DEBUG("Frame %d: ObjectFrame not calculated.", objectDetectedCounter);
		}


		conveyorTrajectory.updateFrameCounter(objectDetectedCounter); //for ROS_INFO logs todos

		if ( (tracked_objects.size() == 0) && (start != -1) && (end != -1) ) {

			if ( !conveyorTrajectory.trajectoryCompleted() ) {
				ROS_INFO("No more tracked objects, finalizing trajectory");
				conveyorTrajectory.finishRecording();
			}
			useObjectFrame = false;
			objectDetectedCounter = 0;
			ROS_WARN("!!No Objects detected!! Frame counter resetted!!");
		}
//to be continue
		double estimatedVelocity = 0.0;
		int calculatedObjects = 0;
		// visualize estimated object position and velocity using arrow markers
		for( int i = 0; i < tracked_objects.size(); ++i ) {

			if( tracked_objects[i].age < 2) continue;

			double covariance;
			Vector position, velocity;
			tracked_objects[i].predict_position( pcl_conversions::fromPCL(input_cloud.header.stamp), &position, &covariance );
			tracked_objects[i].predict_velocity( pcl_conversions::fromPCL(input_cloud.header.stamp), &velocity, &covariance );

			position[2] = object_conveyor_offset; //z position (height) is fixed for the object!

			//calculate conveyorTrajectory
			if ( objectDetectedCounter > 20 ) {
				//record Trajectory
				conveyorTrajectory.updateTrajectory( position );
				if ( (conveyorTrajectory.trajectoryCompleted() ) && (objectDetectedCounter > 100) ) {
					if (!robotActive) {
						ROS_WARN("Frame %d: Triggering interception calculation!", objectDetectedCounter);
						Vector handleDirection = tfObjectToConveyorCoordinates(tracked_objects[i].get_handleHull().at(0) ) - position;
						handleDirection.normalize();
						robotActive = generateTrajectory( position, tracked_objects[i].getHeight(), handleDirection, interceptionTime ); //send the robot a new trajectory to compute
						
					} else {
						// doNothing
						ROS_WARN_ONCE("Frame %d: Robot is currently executing the trajectory!", objectDetectedCounter);
					}
				}
			}
			//calculate objectVelocity
			double objectVelocity = sqrt( pow( velocity[0], 2) + pow( velocity[1], 2) );
			if (objectVelocity > 0.02) { //threshold at which predicted velocity is considered real
				estimatedVelocity +=  objectVelocity;
				calculatedObjects++;
			}

			//Object velocity visualization
			Vector nextPointPrediction = position + velocity;
			tracked_object_markers.markers.push_back( rosArrowMarker( i, "conveyor_plane", position, nextPointPrediction, 0.2 ) );

			//radius visualization
			std::vector<double> objectRadii = tracked_objects[i].get_radii();
			for (int slice = 0; slice < objectRadii.size(); slice++) {	
				if (objectRadii[slice] > 0.0) {
					Vector circlePosition(position[0], position[1], position[2] + slice * circle_segment_height);
					tracked_object_markers.markers.push_back( rosCircleMarker( (i + 300 + slice), "conveyor_plane", circlePosition, objectRadii[slice] ) );
				}
			}
			//Object height visualization
			Vector heightPosition( position[0], position[1], tracked_objects[i].getHeight() );
			tracked_object_markers.markers.push_back( rosPositionMarker( i + 350, "conveyor_plane", heightPosition, 0.2) );
			
			
			if ( (objectDetectedCounter > 20) && !objectWRLCreated ) {
				std_msgs::Float64MultiArray objectParameters;
				objectParameters.data.resize( 5 );
				objectParameters.data.at(0) = tracked_objects[i].getHeight();
				std::vector<double> wallThickness = tracked_objects[i].get_wallThickness();
				
				objectParameters.data.at(1) = objectRadii.at(0) - wallThickness.at(0);
				objectParameters.data.at(2) = objectRadii.at(0);
				objectParameters.data.at(3) = objectRadii.back() - wallThickness.back();
				objectParameters.data.at(4) = objectRadii.back();
				
				export_object_param_publisher.publish( objectParameters );
			}

			predictedPos = nextPointPrediction;
			predictedPos[2] = object_conveyor_offset;
			history.push_front(velocity);

			if (objectDetectedCounter > historyLength) {
				history.pop_back();
				double mx = 0;
				double my = 0;

				int size = history.size();
				for (std::list<Vector>::iterator it = history.begin(); it != history.end(); ++it) {
					Vector v = *it;
					mx +=v[0];
					my +=v[1];
				}

				mx = mx/size;
				my = my/size;

				predictedPos[0] = position[0] + mx;
				predictedPos[1] = position[1] + my;
				predictedPos[2] = object_conveyor_offset;
			}

		}//visualize tracked objects

		//update global conveyorVelocity based on new observation
		if (objectDetectedCounter < 30) {
			//velocity estimation too inaccurate for the first frames
		} else if ( (calculatedObjects > 0) && (conveyorVelocity > 99.0) ) {

			estimatedVelocity /= calculatedObjects;
			conveyorVelocity = estimatedVelocity;
			conveyorTrajectory.updateConveyorVelocity(conveyorVelocity);
			ROS_DEBUG("Frame %d: ConveyorVelocity: First estimation as %f.", objectDetectedCounter, conveyorVelocity);
			std_msgs::Float64 convSpeed;
			convSpeed.data = conveyorVelocity;
			export_speed_publisher.publish( convSpeed );
		} else if ( (calculatedObjects > 0) && (objectDetectedCounter < 499) ) {
			estimatedVelocity /= calculatedObjects;
			conveyorVelocity = ( (objectDetectedCounter - 30) * conveyorVelocity + estimatedVelocity ) / (objectDetectedCounter - 29 );
			conveyorTrajectory.updateConveyorVelocity(conveyorVelocity);
			ROS_DEBUG("Frame %d: ConveyorVelocity: Estimation updated to %f.", objectDetectedCounter, conveyorVelocity);
			std_msgs::Float64 convSpeed;
			convSpeed.data = conveyorVelocity;
			export_speed_publisher.publish( convSpeed );
		}

		objectDetectedCounter++;
		framesViewed++;
		reconstructed_object_marker_publisher.publish( reconstructed_object_markers );
		tracked_object_publisher.publish( tracked_object_markers );
		cloud_publisher.publish( object_cloud );
		
		ROS_INFO("---");
	}	

	if ( conveyorTrajectory.trajectoryVisualizable() ) {

		conveyorTrajectory.updateFrameCounter( objectDetectedCounter );
		visualization_msgs::MarkerArray conveyorTrajectoryMarker = conveyorTrajectory.visualizeTrajectory();
		conveyorTrajectoryVisualization_publisher.publish( conveyorTrajectoryMarker );
		
	}
	
	

}//Object_tracker::points_callback()

//-----------------object reconstruction functions---------------------
Object_tracker::Point_cloud_ptr Object_tracker::getClusteredScenePart(Point_cloud_ptr scene_points, double x,double y) {  //(sp,predictedPos[0],predictedPos[1])

	std::vector<int> indices;
	for (int i = 0; i < scene_points->points.size(); i++) {
		Point p = scene_points->points[i];
		double dist = std::sqrt((p.x - x)*(p.x - x) + (p.y - y)*(p.y - y));
		if (dist > cluster_min_distance || p.z < cluster_min_height) {
			indices.push_back(i);
		}
	}

	for (int i = indices.size()-1; i >= 0; i--) {
		scene_points->erase(scene_points->points.begin() + indices[i]);
	}
	return scene_points;
}//Object_tracker::getClusteredScenePart

Object_tracker::Point_cloud_ptr Object_tracker::getDenseCloud( Point_cloud_ptr objectCloud ) {
  
	int minCount = 9;
	double sphereSize = 0.012;
	std::vector<bool> indices;
	indices.resize( objectCloud->size(), false );
	
	int calcs = 0;
	for (int i = 0; i < objectCloud->size(); i++) {
	  
		if ( !indices[i] ) {
			calcs++;
			Point p = objectCloud->points[i];
			int count = 0;
			std::vector<int> indicesOfCircle;
			for (int j = 0; j < objectCloud->size(); j++) {
				if (j != i) {
					Point q = objectCloud->points[j];
					double xyDist = std::sqrt( pow( p.x - q.x, 2 ) + pow( p.y - q.y, 2 ) );
					double xyzDist = std::sqrt( pow( xyDist, 2 )  + pow(p.z - q.z, 2) );
					if (xyzDist < sphereSize) {
						count++;
						indicesOfCircle.push_back(j);
					}
					if (count == minCount) {
						indices[i] = true;
						for (int l = 0; l < indicesOfCircle.size(); l++) {
							indices[ indicesOfCircle.at(l) ] = true;
							
						}
						count = 0; //reset for next point
						indicesOfCircle.resize(0); //reset for next point
						j = objectCloud->size(); //break loop
					}
				}	
			}  
		}
	}
	
	Point_cloud denseCloud = *objectCloud;
	denseCloud.clear();
	for (int i = 0; i < indices.size(); i++) {
		if ( indices[i] ) {
			denseCloud.push_back( objectCloud->at(i) );
		}
	}
	
	
	if ( denseCloud.size() < 2 ) { 
		objectCloud->resize(1); //empty cloud would give errors, so return almost empty one
		return objectCloud; 
	} else {
		return denseCloud.makeShared();
	}
	
}//Object_tracker::getDenseCloud

void Object_tracker::threadSAC(Point_cloud_XYZ_ptr segment, double segment_ceiling, sacResult res[], int i) {
	pcl::SACSegmentation<pcl::PointXYZ>* tsac;
	tsac = new pcl::SACSegmentation<pcl::PointXYZ>;
	tsac->setModelType( pcl::SACMODEL_CIRCLE2D );
	tsac->setMethodType( pcl::SAC_RANSAC );
	tsac->setMaxIterations( 7 ); //def 7
	tsac->setDistanceThreshold( 0.01 ); //def 0.01
	tsac->setRadiusLimits( 0.015, 0.075 );
	tsac->setOptimizeCoefficients( true ); //def true
	tsac->setInputCloud( segment );
	pcl::ModelCoefficients coefficients; // for circle: 0 is center_x, 1 is center_y, 2 is radius
	pcl::PointIndices inliers;
	tsac->segment( inliers, coefficients );

	// circles with too few supporters are irrelevant
	if( inliers.indices.size() < 40 ) return;
	// determine center coordinates
	Vector segment_center( coefficients.values[0], coefficients.values[1], segment_ceiling - circle_segment_height * 0.5);

	// determine weight
	double segment_weight = static_cast<double>( inliers.indices.size() );
	std::vector<double> radius(1);
	radius[0] = (double) coefficients.values[2];
	res[i] = sacResult(segment_center, radius, circle_segment_height, segment_weight, i, true);
}//Object_tracker::threadSAC

/**
 * This function calculates the object parameters (position, slice radii, height, solidness, wall thickness (if non-solid)
 */
Object_tracker::Point_cloud_ptr Object_tracker::calculate_object_parameters( Conveyor_object_tracking::Object_tracker::Point_cloud_ptr objectCloud ) {
 
  
	if (pauseTracking) {
		ROS_WARN("ACTIVE:IN:PAUSE calc_obj_param");
		while (pauseTracking) {
			ros::Duration(1.0).sleep();
		}
	}
  
	Point_cloud FullObjCloud = *objectCloud;  //Cloud of a single object
	Observed_object observedObject;
	
	if ( !useObjectFrame ) { //calculate general object position and mesh, (impossible to separate the cloud into parts without knowing object center)
		std::vector<sacSegment> segmentedObject = getSegmentedObject( FullObjCloud.makeShared() ); //vector contains sliced object in bottom-to-top order
		segmentedObject.pop_back(); //segmented object now lacks the top frame cap
		ROS_INFO( "Calculating object position with general RANSAC (%d segments)", (int) segmentedObject.size() );
		observedObject = find_object_position( objectCloud, segmentedObject );
		observed_objects.push_back(observedObject); //push observed object (with estimated parameters) onto list
		objectFrameCalcCtr = 0;
		return observedObject.observedCloud; 
		
	} else { //determine if object is hollow, separate cloud into symmetric and nonsymm. parts (+symmetric backside if hollow), approximate handle
		ROS_INFO("ObjectFrame available");
		FullObjCloud = ( transformConveyorToObjectFrame( *objectCloud, false ) ); //transform to objectFrame  
		
		//calculate regular object parameters (height, position, mesh)
		sacSegment topSegmentSlice = getSegmentedObject( FullObjCloud.makeShared() ).back();
		
		// determine if object is solid or hollow (using topSegmentSlice)
		vector<double> topSliceData = calculate_object_rim( topSegmentSlice.cloud );
		bool objectIsHollow = ( topSliceData.size() > 1 );
		double topRadius = topSliceData.at(0);
		
		std::vector<Point_cloud_ptr> separatedObject = separate_object_clouds( FullObjCloud.makeShared(), topRadius ); //separate into symm_front, symm_back and nonsymm
		
		if ( separatedObject.empty() ) {
			ROS_WARN("Filtering resulted in deletion of whole object front! (no calculation possible)");
			useObjectFrame = false;
			objectFrameCalcCtr = -1;
			return nullptr; // empty cloud	
		} 
		
		Point_cloud_ptr symmetricalObjectCloud_Front = separatedObject.at(0);
		std::vector<sacSegment> segmentedSymmetrical_Front = getSegmentedObject( symmetricalObjectCloud_Front );
		ROS_DEBUG("Calculating front symmetric cloud parameters (%d segments)", (int) segmentedSymmetrical_Front.size() );
		
		observedObject = find_object_position( symmetricalObjectCloud_Front, segmentedSymmetrical_Front );
		
		if ( observedObject.height == 0.0 ) { // object parameters couldn't be calculated
			ROS_WARN("Object parameters could not be calculated");
			useObjectFrame = false;
			objectFrameCalcCtr = -1;
			return nullptr;
		} else {
			FullObjCloud = *(observedObject.observedCloud);
		}
		
		if ( objectIsHollow && (separatedObject.size() == 3) ) { //calculate object parameters from inward (= backside) cloud & combine information with outward cloud
			
			double topThickness = topSliceData.at(1);
			ROS_DEBUG("Object detected as hollow! topThickness is %f", topThickness);
			
			Point_cloud_ptr symmetricalObjectCloud_Back = separatedObject.at(2);
			std::vector<sacSegment> segmentedSymmetrical_Back = getSegmentedObject( symmetricalObjectCloud_Back );
			if (segmentedSymmetrical_Back.size() > 0) { 
				ROS_DEBUG("Calculating inward parameters of the object (%d segments)", (int) segmentedSymmetrical_Back.size() );
			
				Observed_object observed_Back = find_object_position( symmetricalObjectCloud_Back, segmentedSymmetrical_Back);
				
				if ( observed_Back.sliceRadii.size() > 1 ) { // combine information about object frontside (outward) and backside (inward) scans
					
					//check object slice radii for irregular numbers
					double lastRadius = topRadius;
					
					for (int i = observedObject.sliceRadii.size() - 1; i > -1; i--) {
						double curRadius = observedObject.sliceRadii.at(i);
						if (curRadius < 0.01) {
							if (i > 0) {
								observedObject.sliceRadii.at(i) = 0.5 * lastRadius + 0.5 * observedObject.sliceRadii.at(i - 1);
							} else {
								observedObject.sliceRadii.at(i) = lastRadius;
							}
							
						} else {
							lastRadius = curRadius;
						}
					}
				  
					//calculate back weighted avg position
					double frontWeight = observedObject.covariance * observedObject.observedCloud->size();
					double backWeight = observed_Back.covariance * observed_Back.observedCloud->size();
					double frontScale = frontWeight / (frontWeight + backWeight);
					double backScale = backWeight / (frontWeight + backWeight);
					Vector avgCenter = observedObject.position * frontScale + observed_Back.position * backScale;
					FullObjCloud += *(observed_Back.observedCloud);
					std::vector<double> wallThickness;
					int innerSize = (int) observed_Back.sliceRadii.size();
					int outerSize = (int) observedObject.sliceRadii.size();
					int minsize = min(innerSize, outerSize); //exclude top slice from inward cloud
					double previousThickness = fmax(topThickness, 0.002); //minimal Thickness 2mm
					
					observedObject.sliceRadii.push_back( topRadius ); //copy top radius
					wallThickness.resize(outerSize + 1); //include top slice that was previously cut to determine hollowness

					for (int i = outerSize; i > minsize - 1; i--) {
						wallThickness.at(i) = min( max( observedObject.sliceRadii.at(i) - 0.01, 0.002 ), previousThickness );
					}
					
					for (int i = minsize - 1 ; i > -1; i--) {
						//assume thickness is the same as in previousSlice
						wallThickness.at(i) = min( max( observedObject.sliceRadii.at(i) - 0.01, 0.002 ), previousThickness );					
					}
					
					observedObject = Observed_object( pcl_conversions::fromPCL( symmetricalObjectCloud_Back->header.stamp ), avgCenter, 
							observedObject.sliceRadii, wallThickness, observedObject.handleHull, observedObject.height, observedObject.covariance, FullObjCloud.makeShared() );
					
				} else { //no information about object backside calculated -> treat object as solid
					ROS_INFO("No information about object inward side could be calculated -> treating object as if it was solid");
					objectIsHollow = false;
				}

			} else {
				ROS_INFO("object inward side could not be segmented -> treating object as if it was solid");
				objectIsHollow = false;
			}
			
	
		} else {
			ROS_INFO("Object detected as solid");
			objectIsHollow = false;
		}
		
		bool handleCalculated = false;
		if ( (objectDetectedCounter > 10) && (separatedObject.size() > 1) ) {
		  
			
			Point_cloud_ptr nonsymmetricalObjectCloud = separatedObject.at(1);
			if (nonsymmetricalObjectCloud->size() > handleMinsize) {
				ROS_DEBUG("Starting handle extraction");
				pcl::EuclideanClusterExtraction<Point>* handle_cluster_extract = new pcl::EuclideanClusterExtraction<Point>;
				std::vector<pcl::PointIndices> clusterIndices;
				handle_cluster_extract->setInputCloud( nonsymmetricalObjectCloud );
				handle_cluster_extract->setClusterTolerance( 0.025 );
				handle_cluster_extract->setMinClusterSize( handleMinsize );
				handle_cluster_extract->extract( clusterIndices );
				pcl::PointIndices handleIndices;
				handleIndices.indices.resize(1);
				
				for (int i = 0; i < clusterIndices.size(); i++) {
					if ( clusterIndices[i].indices.size() > handleIndices.indices.size() ){
						handleIndices = clusterIndices[i];
					}
				}
				
				if ( (clusterIndices.size() == 0) || (handleIndices.indices.size() < handleMinsize) ) {
					ROS_DEBUG("Too few points in the cloud, no handle could be extracted!");
				} else {
					Point_cloud_XYZ_ptr handleCloud (new Point_cloud_XYZ);
					pcl::copyPointCloud(*nonsymmetricalObjectCloud, handleIndices.indices, *handleCloud);
					ROS_DEBUG("Using clustered part for handle approximation! (%d / %d points from nonsymm)", (int) handleCloud->size(), (int) nonsymmetricalObjectCloud->size() );
					
					std::vector<Vector> hullPoints = getHandleHullpoints( handleCloud );
					
					handleCalculated = (hullPoints.size() > 1);//if everything went well
					if ( handleCalculated) { observedObject.handleHull = hullPoints; }
				}
				
				
			}
			
		}
		
		observed_objects.push_back(observedObject); //push observed object (with estimated parameters) onto list
		//FullObjCloud = ( transformConveyorToObjectFrame( FullObjCloud, true) ); //transform back to conveyorFrame
		//filtered_object_cloud_publisher.publish(FullObjCloud); //publish object cloud
		objectFrameCalcCtr++;
		return FullObjCloud.makeShared();
		
	}
	
}

/**
 * Calculates the median plane of the handle by estimating the line of its projection on the ground (as the object lies orthogonal 
 * on the conveyor plane and thus the handle plane is orthogonal to the conveyor plane as well). With this line calculated the
 * points can be projected on the handle plane and their convex hull can be extracted for a representation in the mesh model.
 * 
 */
std::vector<Object_tracker::Vector> Object_tracker::getHandleHullpoints( Point_cloud_XYZ_ptr handleCloud ) {
  
	Point_cloud_XYZ_ptr handle2d (new Point_cloud_XYZ);
	pcl::copyPointCloud(*handleCloud, *handle2d);
	for (int i = 0; i < handle2d->size(); i++) {
		handle2d->at(i).z = 0.0;
		if ( i < handle2d->size() / 3 ) { handle2d->push_back( PointXYZ(0.0, 0.0, 0.0) ); } //bias so that the handle plane goes through object center
	}
	
	pcl::SACSegmentation<PointXYZ>* lineSac;
	lineSac = new pcl::SACSegmentation<PointXYZ>;
	lineSac->setModelType(pcl::SACMODEL_LINE);
	lineSac->setMethodType(pcl::SAC_RANSAC);
	lineSac->setMaxIterations(15);
	lineSac->setDistanceThreshold(0.01);
	lineSac->setOptimizeCoefficients(true);
	lineSac->setInputCloud(handle2d);
	pcl::ModelCoefficients lineCoefficients;
	pcl::PointIndices inliers;
	lineSac->segment( inliers, lineCoefficients );
	
	PointXYZ a( lineCoefficients.values[0], lineCoefficients.values[1], 0.0 );
	PointXYZ b( lineCoefficients.values[3] - a.x, lineCoefficients.values[4] - a.y, 0.0 );
	PointXYZ c( a.x, a.y, 1.0 );
	{ /*visualization only*/
		std::vector<Vector> planePoints;
		planePoints.push_back( Vector(a.x, a.y, a.z) );
		planePoints.push_back( Vector(b.x, b.y, b.z) );
		reconstructed_object_markers.markers.push_back( rosLineStripMarker( 98765, "object_frame", planePoints, Vector(1.0, 0.0, 0.0), 1.0 ) );
	}
	pcl::ModelCoefficients::Ptr planeCoeffs (new pcl::ModelCoefficients);
	planeCoeffs->values.push_back( (b.y - a.y)*(c.z - a.z) - (c.y - a.y)*(b.z - a.z) );/*a*/
	planeCoeffs->values.push_back( (b.z - a.z)*(c.x - a.x) - (c.z - a.z)*(b.x - a.x) );/*b*/
	planeCoeffs->values.push_back( (b.x - a.x)*(c.y - a.y) - (c.x - a.x)*(b.y - a.y) );/*c*/
	planeCoeffs->values.push_back( -(planeCoeffs->values[0]*a.x + planeCoeffs->values[1]*a.y + planeCoeffs->values[2]*a.z) );/*d*/
	
	pcl::ProjectInliers<PointXYZ> projection;
	projection.setModelType( pcl::SACMODEL_PLANE );
	projection.setCopyAllData( true );
	projection.setInputCloud( handleCloud );
	projection.setModelCoefficients( planeCoeffs );
	projection.filter( *handleCloud );
	
	Point_cloud_XYZ_ptr convexHandle (new Point_cloud_XYZ);
	pcl::ConvexHull<PointXYZ> convex;
	convex.setInputCloud( handleCloud );
	convex.reconstruct( *convexHandle );
	ROS_DEBUG("Convex hull has %d points left", (int) convexHandle->size() );
	
	std::vector<Vector> triStripPoints;
	
	if (convexHandle->size() > 1) {
		for (int i = 0; i < convexHandle->size() - 1; i++) { 
			triStripPoints.push_back( Vector( convexHandle->points[i].x, convexHandle->points[i].y, convexHandle->points[i].z ) );
			triStripPoints.push_back( Vector( convexHandle->points[i+1].x, convexHandle->points[i+1].y, convexHandle->points[i+1].z ) );
			triStripPoints.push_back( Vector( 0.0, 0.0, 0.0 ) );
		}
	} else {
		triStripPoints.push_back( Vector( 0.0, 0.0, 0.0 ) );
	}
	
	return triStripPoints;
}//Object_tracker::getHandlePlane()

/**
 * Checks the inserted PointCloud belonging for a circular disc shape to decide whether the object to which the cloud belongs is hollow or solid
 * 
 * @return distance from center to rim if the object is hollow, or -1.0 if the object is solid
 */
vector<double> Object_tracker::calculate_object_rim( Point_cloud_XYZ_ptr topCapCloud ) {
  
	double distributionThreshold = 1.2; //TODO check for a good value here, possibly choose on basis of the noise amplitude as advised in escaida et al. 2012 paper
	double sliceThickness = circle_segment_height;
	double sliceWidth = 0.5 * circle_segment_height;
	double rimPosition = 0.0;
	double distMedian = 0.0;
	double thickness = 0.0;
	
	vector<double> topSliceData; //store median distance and calculated object rim thickness
	std::vector<int> pointsPerSlice; //store the number of points in each slice
	int zero = 0;
	
/**	if (topCapCloud->size() < circle_segment_min_points) { //too few points in the top slice to be hollow
		topSliceData.push_back(0.0);
		return topSliceData;
	}
**/	
	pointsPerSlice.push_back(zero);
	
	// sort points by distance to centerPoint ( = objectPosition )
	for (int i = 0; i < topCapCloud->size(); i++) {
		pcl::PointXYZ current = topCapCloud->at(i);
		double distToCenter = sqrt( pow( current.x, 2) + pow( current.y, 2) );
		distMedian += distToCenter;
		
		int sliceNumber =  (distToCenter / sliceWidth); //round down to get the slice number from 0 to n
		
		if (pointsPerSlice.size() < (sliceNumber + 1) ) { //streching and initializing the vector to desired size
			int oldSize = (int) pointsPerSlice.size();
			int extraSize = sliceNumber - (int) pointsPerSlice.size() + 1;
			for (int j = 0; j < extraSize; j++) { 
				pointsPerSlice.push_back(zero);
			}
		}
		pointsPerSlice.at(sliceNumber) += 1;
	}
	
	distMedian /= (double) topCapCloud->size();
	
	topSliceData.push_back(distMedian);
	if (pointsPerSlice.size() < 3) {
		return topSliceData;
	}
	
	for (int i = 0; i < pointsPerSlice.size(); i++) {
		int expectedPointCount = ( (int) topCapCloud->size() ) * (2 * i + 1) / pow( ( (int) pointsPerSlice.size() ), 2 );
		if ( (expectedPointCount > 0) && ( (pointsPerSlice.at(i) / expectedPointCount) >= distributionThreshold ) ) { //check for threshold!
				if (i == 0) {
					return topSliceData;
				} else {
					rimPosition = ( i * sliceWidth );
					thickness = 1.8 * (distMedian - rimPosition);
					topSliceData.push_back(thickness);
					return topSliceData;
				}
		}
	}
	
	return topSliceData; //only containing median distance
}//Object_tracker::calculate_object_rim()

Conveyor_object_tracking::Observed_object Object_tracker::find_object_position( Point_cloud_ptr object, std::vector<sacSegment> segmentedObject ) {
  
	if (pauseTracking) {
		ROS_WARN("ACTIVE:IN:PAUSE find_obj_pos");
	}
	counter++;
	if (counter == 1) {
		ROS_INFO_ONCE("Starting ransac");
		start = ros::Time::now().toSec();
	}
	
	if ( segmentedObject.empty() ) {
		segmentedObject = getSegmentedObject( object );
	}
	sacResult revolutionCalculated = calculateSolidOfRevolution( segmentedObject, true );

	if (!revolutionCalculated.valid) {
		ROS_WARN("Find_Object_Position: INVALID RANSAC CALCULATION");
		Observed_object observedObject;
		observedObject.height = 0.0;
		return observedObject; //return nothing as no object parameters could be calculated
	}

	Vector position = revolutionCalculated.center;
	std::vector<double> radii = revolutionCalculated.radii;
	std::vector<double> thicknesses; thicknesses.push_back(0.0); // size==1 meaning object is solid
	std::vector<Vector> handle; handle.push_back( Vector(0,0,0) );//empty handle (unneccesary at this point)
	double height = radii.size() * circle_segment_height;
	double weight = revolutionCalculated.weight;

	if (useObjectFrame) { //transform object center from objectFrame to ConveyorFrame
		Point_cloud objCenterCloud;
		Point centerPoint;
		centerPoint.x = position[0];
		centerPoint.y = position[1];
		centerPoint.z = 0.0;
		objCenterCloud.push_back(centerPoint);
		objCenterCloud = ( transformConveyorToObjectFrame( objCenterCloud, true) );
		centerPoint = objCenterCloud.front();
		Vector objCenter(centerPoint.x, centerPoint.y, centerPoint.z);
		
		return Observed_object( pcl_conversions::fromPCL(object->header.stamp), objCenter, radii, thicknesses, handle, height, (weight / object->size() ), object );
	} else {
		return Observed_object( pcl_conversions::fromPCL(object->header.stamp), position, radii, thicknesses, handle, height, (weight / object->size() ), nullptr );
	}

}//Object_tracker::find_object_position()

/**
 * Checks for the left point if a point on the right side is within a given threshold
 */
bool Object_tracker::mirror_match_point( Point left, Point right ) {

	Vector threshold(0.008, 0.01, 0.004);

	bool matching_x;
	bool matching_y;
	bool matching_z;

	double diff_x = left.x + right.x;
	(diff_x < 0.0) ? matching_x = diff_x > -threshold[0] : matching_x = diff_x < threshold[0];

	if (!matching_x) {
		return false;
	} else {
		double diff_y = left.y - right.y;
		(diff_y < 0.0) ? matching_y = diff_y > -threshold[0] : matching_y = diff_y < threshold[0];
	}
	if (!matching_y) {
		return false;
	} else {
		double diff_z = left.z - right.z;
		(diff_z < 0.0) ? matching_z = diff_z > -threshold[0] : matching_z = diff_z < threshold[0];
		return matching_z;
	}

}//Object_tracker::mirror_match_point

/**
 * checks for each point in @targetCloud if a matching point in @mirrorCloud is found
 * @returns two indices vectors, first containing all symmetrical points in targetCloud, second all non-symmetrical
 */
std::vector< Object_tracker::Indices > Object_tracker::mirror_match_clouds( Point_cloud_ptr targetCloud, Point_cloud_ptr mirrorCloud ) {
 
	std::vector< Indices > separatedCloud;
	Indices inlierList;
	Indices outlierList;
	for (int targetPos = 0; targetPos < targetCloud->size(); targetPos++) {
		Point targetPoint = targetCloud->at(targetPos);
		bool twinFound = false;
		for (int mirrorPos = 0; (mirrorPos < mirrorCloud->size()) && !twinFound; mirrorPos++) {
			Point mirrorPoint = mirrorCloud->at(mirrorPos);
			if (mirror_match_point(targetPoint, mirrorPoint)) {
				twinFound = true;
			}
		}
		(twinFound) ? inlierList.push_back(targetPos) : outlierList.push_back(targetPos);
	}
	
	separatedCloud.push_back( inlierList );
	separatedCloud.push_back( outlierList );
	return separatedCloud;
}//Object_tracker::mirror_match_clouds()

/**Object_tracker::separate_object_clouds
 * Removes the outliers by checking that every point from the left side of the object
 * has a corresponding point within a defined sphere on the right side. As the differentiation
 * of the object into left and right requires a good estimation of the middlepoint, this
 * calculation can only be done in objectFrame
 *
 * @param object The object cloud which is to be filtered
 * @return Vector containing a reference to the three separated clouds of the object, in order symmetricFront, nonsymmetric, symmetricBack
 */
std::vector< Object_tracker::Point_cloud_ptr > Object_tracker::separate_object_clouds( Point_cloud_ptr object, double topRadius ) {

	std::vector< Point_cloud_ptr > separatedObject;
	Point_cloud frontSymmetrical;
	Point_cloud backSymmetrical;
	Point_cloud nonSymmetrical;
	
	// cut out parts with too high distance to other ones
	object = getDenseCloud( object );

	Point_cloud_ptr leftFront( new Point_cloud );
	leftFront->header = object->header;
	Point_cloud_ptr leftBack( new Point_cloud );
	leftBack->header = object->header;
	
	Point_cloud_ptr rightFront( new Point_cloud );
	rightFront->header = object->header;
	Point_cloud_ptr rightBack( new Point_cloud );
	rightBack->header = object->header;
	
	//separate pointCloud in front/back and left/right clouds (x-z plane dividing left and right, y-z plane front and back)
	for (int i = 0; i < object->size(); i++) {
		Point p = object->at(i);
		double distFromCenter = 999.9;//sqrt( pow(p.x, 2) + pow(p.y, 2) );
		if (p.x > 0.0 && p.x < object_maxwidth) { //left side
			if ( (p.y > 0.0) && (distFromCenter > 0.5 * topRadius) ) {
				leftFront->push_back(p);
			} else if ( (p.y < 0.0) && (distFromCenter > 0.2 * topRadius) ) {
				leftBack->push_back(p);
			}
		} else if (p.x > -object_maxwidth) {//right side
			if ( (p.y > 0.0) && (distFromCenter > 0.5 * topRadius) ) {
				rightFront->push_back(p);
			} else if ( (p.y < 0.0) && (distFromCenter > 0.2 * topRadius) ) {
				rightBack->push_back(p);
			}
		}
	}

	//check and split front clouds in symmetrical/nonsymmetrical part clouds
	std::vector< Indices > leftFrontSplitted = mirror_match_clouds( leftFront, rightFront );
	if (leftFrontSplitted.at(0).size() > 0) {
		Point_cloud leftSymmetric;
		pcl::copyPointCloud( *leftFront, leftSymmetric ); 
		frontSymmetrical = Point_cloud( leftSymmetric, leftFrontSplitted.at(0) );// adds symmetric part of left cloud
	} else {
		//ROS_DEBUG("SeparatePointClouds -FrontFiltering: no inliers found, returning empty object");
		return separatedObject;
	}
			
	std::vector< Indices > rightFrontSplitted = mirror_match_clouds( rightFront, frontSymmetrical.makeShared() );
	if (rightFrontSplitted.at(0).size() > 0) {
		Point_cloud rightSymmetric;
		pcl::copyPointCloud( *rightFront, rightSymmetric );
		frontSymmetrical += Point_cloud( rightSymmetric, rightFrontSplitted.at(0) );
	} else {
		//ROS_DEBUG("SeparatePointClouds -FrontFiltering: no inliers found, returning empty object");
		return separatedObject;
	}
	
	separatedObject.push_back( frontSymmetrical.makeShared() ); //symmetric front part of cloud completely filtered
	
	//adding nonsymmetric parts to the cloud
	if (leftFrontSplitted.at(1).size() > 0) { nonSymmetrical += Point_cloud( *leftFront, leftFrontSplitted.at(1) ); }
	if (rightFrontSplitted.at(1).size() > 0) { nonSymmetrical += Point_cloud( *rightFront, rightFrontSplitted.at(1) ); }

	//ROS_DEBUG("SeparatePointClouds -FrontSplit complete (sym_front: %dpts, nonsym_front: %dpts)", (int) frontSymmetrical.size(), (int) nonSymmetrical.size() );
	
	//check and split back clouds in symmetrical/nonysmm. part clouds
	bool backsideValid = false;
	std::vector< Indices > leftBackSplitted;
	if ( !leftBack->empty() && !rightBack->empty() ) {
		leftBackSplitted = mirror_match_clouds(leftBack, rightBack);
		Indices leftSymmIndices = leftBackSplitted.at(0);
		if (!leftSymmIndices.empty()) { 
			Point_cloud leftSymmetric;
			pcl::copyPointCloud( *leftBack, leftSymmetric);
			backSymmetrical = Point_cloud( leftSymmetric, leftSymmIndices ); 
			std::vector< Indices > rightBackSplitted = mirror_match_clouds( rightBack, backSymmetrical.makeShared() );
			if (rightBackSplitted.at(0).size() > 0) {
				Point_cloud rightSymmetric;
				pcl::copyPointCloud( *rightBack, rightSymmetric);
				backSymmetrical += Point_cloud( rightSymmetric, rightBackSplitted.at(0) );
				backsideValid = true;
				
				//adding nonsymmetric parts to the cloud
				if (leftBackSplitted.at(1).size() > 0) { nonSymmetrical += Point_cloud( *leftBack, leftBackSplitted.at(1) ); }
				if (rightBackSplitted.at(1).size() > 0) { nonSymmetrical += Point_cloud( *rightBack, rightBackSplitted.at(1) ); }
				
			}
			
		} else {
			//ROS_DEBUG("SeparatePointClouds: No symmetric portion in backside of object found");
		}
		
	} else {
		//ROS_DEBUG("SeparatePointClouds: no backside to be calculated! ");
	}
	
	filtered_object_symm_front_publisher.publish( transformConveyorToObjectFrame(frontSymmetrical, true) );
	
	if (nonSymmetrical.size() > 0) {
		separatedObject.push_back( nonSymmetrical.makeShared() );
		filtered_object_nonsymm_publisher.publish( transformConveyorToObjectFrame(nonSymmetrical, true) );
	}
	if (backsideValid) {
		if (separatedObject.size() != 2) {
			nonSymmetrical.resize(1);
			separatedObject.push_back( nonSymmetrical.makeShared() ); //invalid but needed for correct sizing of the seperatedObject vector
		}
		separatedObject.push_back( backSymmetrical.makeShared() );
		filtered_object_symm_back_publisher.publish( transformConveyorToObjectFrame(backSymmetrical, true) );
	}
	
	return separatedObject;	

}//Object_tracker::separate_object_clouds()

void Object_tracker::update_object_position_object_frame( Point_cloud_ptr object ) {
  
  

	// sort points by z coordinate (height) ascending
	std::sort( object->points.begin(), object->points.end(), [](const Point& a, const Point& b) { return a.z < b.z; });
	double height = object->points.back().z;
	// number of slices
	
	int num_slices = ( height - object->points.front().z ) / ( circle_segment_height );
	std::vector<Point_cloud_ptr> segments;
	double weight = 0.0; // sum of the weights
	Vector center( 0.0, 0.0, 0.0 ); // weighted sum of the centers
	std::vector<double> radii; radii.resize(num_slices, 0.0); //vector containing each slices radius and zero values for non calculated slices;
	std::vector<double> thicknesses; thicknesses.resize(num_slices, 0.0);
	std::vector<Vector> handle; handle.push_back( Vector(0,0,0) );//empty stub, not needed at this point
	int supporters = 0;
	double segment_ceiling = object->points.front().z + circle_segment_height;
	int copy_index = 0;

	// throw first (=bottom) segment away
	while( copy_index < object->size() && object->points[copy_index].z < segment_ceiling ) ++copy_index;


	for( int i = 2; i < num_slices; i++ ) {
		Point_cloud_ptr segment( new Point_cloud );
		segment->clear();
		segment_ceiling += circle_segment_height;
		// extract height segment into 2D cloud for middlepoint + radius recognition
		while( ( copy_index < object->size() ) && ( object->points[copy_index].z < segment_ceiling ) ){
			Point point;
			point.x = object->points[copy_index].x;
			point.y = object->points[copy_index].x;
			point.z = 0.0;
			segment->push_back( point );
			++copy_index;
		}
		segments.push_back(segment);
	}

	Vector prevCtr;
	//int i = 0;
	for (int i = 0; i < segments.size(); i++) {

		Point_cloud slice = *segments[i];
		double segment_weight = slice.size();
		double segment_cx = 0.0;
		double segment_cy = 0.0;
		double segment_radius = 0.0;

		double sum_px = 0.0;
		double sum_py = 0.0;
		double sum_pxQuad = 0.0;
		double sum_pyQuad = 0.0;
		double sum_pyCubic = 0.0;
		double sum_px_py = 0.0;
		double sum_pxQuad_py = 0.0;

		int N = slice.size();

		std::sort( slice.begin(), slice.end(), [](const Point& a, const Point& b) { return a.x < b.x; });

		//Fill in Values: calculate most plausible radius
		segment_radius = (slice.back().x - slice.front().x) / 2;
		segment_cx = slice.front().x + segment_radius;
		//Fill in Values: calculates sums
		for (int i = 0; i < N; i++) {
			double pointX = slice.at(i).x;
			double pointY = slice.at(i).y;
			sum_py += pointY;
		}
		segment_cy = sum_py / N;
		

		// determine center 3d coordinates
		Vector segment_center( segment_cx, segment_cy, segment_ceiling - circle_segment_height * 0.5);
		// add weighted coordinates to sum
		weight += segment_weight;
		center += segment_center * segment_weight;
		radii[i] = segment_radius;

		supporters++;

		prevCtr = segment_center;

    	}//i++;

	if( supporters < 2 ) return;

	center /= weight;
	//transform center back into conveyor_frame
	Point_cloud_ptr mpCloud( new Point_cloud );
	mpCloud->header = object->header;
	Point centerPoint;
	centerPoint.x = center[0];
	centerPoint.y = center[1];
	centerPoint.z = 0;
	mpCloud->push_back( centerPoint );

	Point_cloud transformedCenter;
	transformedCenter = transformConveyorToObjectFrame(*mpCloud, true);
	Vector centerTransformed(transformedCenter.front().x, transformedCenter.front().y, object_conveyor_offset);

	observed_objects.push_back( Observed_object( pcl_conversions::fromPCL(mpCloud->header.stamp), centerTransformed, radii, thicknesses, handle, height, (weight / object->size()), object ) );

}//Object_tracker::update_object_position()

/**
 * match observed objects with tracked objects
 */
void Object_tracker::match_objects () {

	std::priority_queue<Matching, std::vector<Matching>, std::greater<Matching>> matchings; // priority queue is max heap by default, we want min heap (smallest on top) so use std::greater todos
	for( int observed_id = 0; observed_id < observed_objects.size(); observed_id++ ){
		for( int tracked_id = 0; tracked_id < tracked_objects.size(); tracked_id++ ){
			Matching m = calculate_object_distance( tracked_id, observed_id ); // get pairwise distance between tracked and observed objects
			if( m.value < matching_distance_threshold ) { // only small distances make sense
				matchings.push( m );  //observed id and tracked id and .. will be saved.
			}
		}
	}
	
	if (observed_objects.size() == 0) { //all tracked objects are unobserved in this frame
	  
		ROS_DEBUG("No observed objects, matching impossible.");
		for( int tracked_id = 0; tracked_id < tracked_objects.size(); tracked_id++ ) {
			tracked_objects[tracked_id].unobserved++;
		}
		// delete unobserved objects after max_unobserved iterations
		tracked_objects.erase( std::remove_if( tracked_objects.begin(), tracked_objects.end(), removeUnobserved ), tracked_objects.end() );
		
	} else if (tracked_objects.size() == 0) { // add new tracked objects for unmatched observations
		
		for( int observed_id = 0; observed_id < observed_objects.size(); observed_id++ ) {
			tracked_objects.push_back( Tracked_object( observed_objects[observed_id], getKinectCoordinates(observed_objects[observed_id].position)) ); //todos
		}
		
	} else { // update matched objects
		ROS_INFO("Updating matched objects");
		std::vector<bool> observed_matched( observed_objects.size(), false );
		std::vector<bool> tracked_matched( tracked_objects.size(), false );
		while( !matchings.empty() ) { //todos
			Matching m = matchings.top();

			if( !observed_matched[m.observed_id] && !tracked_matched[m.tracked_id] ) {
				observed_matched[m.observed_id] = true;
				tracked_matched[m.tracked_id] = true;
				if ((objectDetectedCounter > 29) && ( (objectDetectedCounter % 5) == 0) ) {//to prevent partially visible objects from being stored as reconstructed
					reconstruct_by_angle(m.observed_id , m.tracked_id); //if viewpoints to old cloud differ more than a predefined view angle
				}
				tracked_objects[m.tracked_id].update_position( observed_objects[m.observed_id] );
				Vector trackedPosition = tracked_objects[m.tracked_id].get_position();
				useObjectFrame = sendObjectTransformation( trackedPosition , m.tracked_id); //update objectTransformation, if successful switch next calculation into objectFrame
				recreateTrackedObject( tracked_objects[m.tracked_id] );
			}

			matchings.pop();
		}

		// all tracked objects without a matching are unobserved
		for( int tracked_id = 0; tracked_id < tracked_matched.size(); tracked_id++ ){
			if( !tracked_matched[tracked_id] ){
				tracked_objects[tracked_id].unobserved++;
			}
		}

		// delete unobserved after max_unobserved iterations
		tracked_objects.erase( std::remove_if( tracked_objects.begin(), tracked_objects.end(), removeUnobserved ), tracked_objects.end() );

		// add new tracked objects for unmatched observations
		for( int observed_id = 0; observed_id < observed_matched.size(); observed_id++ ){
			if( !observed_matched[observed_id] ){
				tracked_objects.push_back( Tracked_object( observed_objects[observed_id], getKinectCoordinates(observed_objects[observed_id].position)) );
			}
		}
	}
	if (tracked_objects.size() == 0) {
		useObjectFrame = false;
		ROS_DEBUG("Frame %d: Tracked_objects list is empty!", objectDetectedCounter);
	} else {
		ROS_DEBUG("Frame %d: Tracked_objects list now holds %d objects!", objectDetectedCounter, (int) tracked_objects.size());
	}

	// reset observed objects after they have been matched
	observed_objects.clear();

}//Object_tracker::match_objects()

void Object_tracker::recreateTrackedObject( Tracked_object to ) {

	Vector position = to.get_position();
	std::vector<double> outerRadii = to.get_radii();
	std::vector<double> wallThicknesses = to.get_wallThickness();
	std::vector<Vector> handleHull = to.get_handleHull();
	
	Point_cloud outwardCloud;
	outwardCloud.header = to.getObject().header;
	Point_cloud inwardCloud;
	inwardCloud.header = to.getObject().header;
	Point anglePoint;
	bool objectIsHollow = (wallThicknesses.size() > 2);
	for (int i = 0; i < outerRadii.size(); i++) {
		double outerSliceRadius = outerRadii[i];
		double innerSliceRadius = outerSliceRadius;
		if (objectIsHollow) { 
			innerSliceRadius -= wallThicknesses[i]; 
			//ROS_INFO("Slice %d, outerRadius: %f, innerRadius: %f, wallThickness: %f", i, outerSliceRadius, innerSliceRadius, wallThicknesses[i] ); 
		}
		for (int j = 0; j < reconstruct_res; j++) {
			double angle = (2 * j *M_PI) / reconstruct_res;
			anglePoint.x = position[0] + cos(angle) * outerRadii[i];
			anglePoint.y = position[1] + sin(angle) * outerRadii[i];
			anglePoint.z = position[2] + i * circle_segment_height;
			outwardCloud.push_back(anglePoint);
			if (objectIsHollow) {
				anglePoint.x = position[0] + cos(angle) * (outerRadii[i] - wallThicknesses[i]);
				anglePoint.y = position[1] + sin(angle) * (outerRadii[i] - wallThicknesses[i]);
				anglePoint.z = position[2] + i * circle_segment_height;
				inwardCloud.push_back(anglePoint);
			}
		}
	}
	
	if (!objectIsHollow) { //to prevent passing an empty cloud
		anglePoint.x = position[0];
		anglePoint.y = position[1];
		anglePoint.z = position[2];
		inwardCloud.push_back(anglePoint);
	}
	
	visualization_msgs::Marker trackedObjMeshMarker = revolutionMeshMarker( position, outwardCloud, inwardCloud, handleHull );
	reconstrucedObjectVisualization_publisher.publish( trackedObjMeshMarker ); 
  
	if (robotActive && !objectWRLCreated) {
		objectToWRL( position, outwardCloud, inwardCloud, handleHull );
	}
  
}//Object_tracker::recreateTrackedObject()

visualization_msgs::Marker Object_tracker::revolutionMeshMarker( Vector center, Point_cloud outwardCloud, Point_cloud inwardCloud, std::vector<Vector> handleHull ) {
	
	bool objectIsHollow = ( inwardCloud.size() == outwardCloud.size() );
  
	visualization_msgs::Marker revolutionMeshMarker;
	revolutionMeshMarker.header.frame_id = "conveyor_plane";
	revolutionMeshMarker.header.stamp = ros::Time().now();
	revolutionMeshMarker.ns = "conveyor_object_tracker";
	revolutionMeshMarker.id = 20103;
	revolutionMeshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	revolutionMeshMarker.action = visualization_msgs::Marker::ADD;
	revolutionMeshMarker.lifetime = ros::Duration(15.0);
	revolutionMeshMarker.color.r = 0.7;
	revolutionMeshMarker.color.g = 0.0;
	revolutionMeshMarker.color.b = 0.0;
	revolutionMeshMarker.color.a = 1.0;
	revolutionMeshMarker.scale.x = 1.0;
	revolutionMeshMarker.scale.y = 1.0;
	revolutionMeshMarker.scale.z = 1.0;
	int resolution = reconstruct_res;
	int segments = outwardCloud.size() / resolution;
	int totalPointSize = 6 * (segments -  1) * resolution; // 2 * height-1 * width = triangles needed for full outer coverage
	if (objectIsHollow) { totalPointSize = 2 * totalPointSize + 12 * resolution; } //double outside + top edge + two bottom caps
	    else { totalPointSize += 6 * resolution; } //top&bottom cap
	if (handleHull.size() > 1) { totalPointSize += ( handleHull.size() - 1 )* 3; } //triangles for handle
	revolutionMeshMarker.points.resize( totalPointSize ); 
	size_t idx = 0;
	size_t currentPosition = 0;
	size_t nextPosition = 1;
	for (int i = 0; i < segments ; i++)  {
		for (int j = 0; j < resolution; j++) {
			currentPosition = (i * resolution + j);
			nextPosition = currentPosition + 1;
			if (j == resolution - 1) { nextPosition = i * resolution; }
			
			if ( i == 0 ) { //bottom cap
				revolutionMeshMarker.points[idx].x = outwardCloud.at(nextPosition).x; // 2nd point in circle
				revolutionMeshMarker.points[idx].y = outwardCloud.at(nextPosition).y;
				revolutionMeshMarker.points[idx].z = outwardCloud.at(nextPosition).z;
				idx++;
				revolutionMeshMarker.points[idx].x = outwardCloud.at(currentPosition).x; //1st point in circle
				revolutionMeshMarker.points[idx].y = outwardCloud.at(currentPosition).y;
				revolutionMeshMarker.points[idx].z = outwardCloud.at(currentPosition).z;
				idx++;
				revolutionMeshMarker.points[idx].x = center[0]; //center point
				revolutionMeshMarker.points[idx].y = center[1];
				revolutionMeshMarker.points[idx].z = outwardCloud.at(currentPosition).z;
				idx++;
				
				if (objectIsHollow) {
					revolutionMeshMarker.points[idx].x = inwardCloud.at(currentPosition).x; //1st point in circle
					revolutionMeshMarker.points[idx].y = inwardCloud.at(currentPosition).y;
					revolutionMeshMarker.points[idx].z = inwardCloud.at(currentPosition).z;
					idx++;
					revolutionMeshMarker.points[idx].x = inwardCloud.at(nextPosition).x; // 2nd point in circle
					revolutionMeshMarker.points[idx].y = inwardCloud.at(nextPosition).y;
					revolutionMeshMarker.points[idx].z = inwardCloud.at(nextPosition).z;
					idx++;
					revolutionMeshMarker.points[idx].x = center[0]; //center point
					revolutionMeshMarker.points[idx].y = center[1];
					revolutionMeshMarker.points[idx].z = inwardCloud.at(currentPosition).z;
					idx++;
				}
			}
			
			if (i > 0) { //DOWNfacing triangles
				revolutionMeshMarker.points[idx].x = outwardCloud.at(currentPosition).x; //1st point in circle
				revolutionMeshMarker.points[idx].y = outwardCloud.at(currentPosition).y;
				revolutionMeshMarker.points[idx].z = outwardCloud.at(currentPosition).z;
				idx++;
				revolutionMeshMarker.points[idx].x = outwardCloud.at(nextPosition - resolution).x; // lower circle 2nd point
				revolutionMeshMarker.points[idx].y = outwardCloud.at(nextPosition - resolution).y;
				revolutionMeshMarker.points[idx].z = outwardCloud.at(nextPosition - resolution).z;
				idx++;
				revolutionMeshMarker.points[idx].x = outwardCloud.at(nextPosition).x; //2nd point in circle
				revolutionMeshMarker.points[idx].y = outwardCloud.at(nextPosition).y;
				revolutionMeshMarker.points[idx].z = outwardCloud.at(nextPosition).z;
				idx++;
				if (objectIsHollow) {
					revolutionMeshMarker.points[idx].x = inwardCloud.at(nextPosition).x; //2nd point in circle
					revolutionMeshMarker.points[idx].y = inwardCloud.at(nextPosition).y;
					revolutionMeshMarker.points[idx].z = inwardCloud.at(nextPosition).z;
					idx++;
					revolutionMeshMarker.points[idx].x = inwardCloud.at(currentPosition - resolution).x; // lower circle 1st point
					revolutionMeshMarker.points[idx].y = inwardCloud.at(currentPosition - resolution).y;
					revolutionMeshMarker.points[idx].z = inwardCloud.at(currentPosition - resolution).z;
					idx++;
					revolutionMeshMarker.points[idx].x = inwardCloud.at(currentPosition).x; //1st point in circle
					revolutionMeshMarker.points[idx].y = inwardCloud.at(currentPosition).y;
					revolutionMeshMarker.points[idx].z = inwardCloud.at(currentPosition).z;
					idx++;
				}
			}
			if ( i < segments - 1 ) { //UPfacing triangles 
				revolutionMeshMarker.points[idx].x = outwardCloud.at(currentPosition).x; //1st point in circle
				revolutionMeshMarker.points[idx].y = outwardCloud.at(currentPosition).y;
				revolutionMeshMarker.points[idx].z = outwardCloud.at(currentPosition).z;
				idx++;
				revolutionMeshMarker.points[idx].x = outwardCloud.at(nextPosition).x; //2nd point in circle
				revolutionMeshMarker.points[idx].y = outwardCloud.at(nextPosition).y;
				revolutionMeshMarker.points[idx].z = outwardCloud.at(nextPosition).z;
				idx++;
				revolutionMeshMarker.points[idx].x = outwardCloud.at(currentPosition + resolution).x; // upper circle 1st point
				revolutionMeshMarker.points[idx].y = outwardCloud.at(currentPosition + resolution).y;
				revolutionMeshMarker.points[idx].z = outwardCloud.at(currentPosition + resolution).z;
				idx++;
				if (objectIsHollow) {
					revolutionMeshMarker.points[idx].x = inwardCloud.at(nextPosition).x; //2nd point in circle
					revolutionMeshMarker.points[idx].y = inwardCloud.at(nextPosition).y;
					revolutionMeshMarker.points[idx].z = inwardCloud.at(nextPosition).z;
					idx++;
					revolutionMeshMarker.points[idx].x = inwardCloud.at(currentPosition).x; //1st point in circle
					revolutionMeshMarker.points[idx].y = inwardCloud.at(currentPosition).y;
					revolutionMeshMarker.points[idx].z = inwardCloud.at(currentPosition).z;
					idx++;
					revolutionMeshMarker.points[idx].x = inwardCloud.at(nextPosition + resolution).x; // upper circle 2nd point
					revolutionMeshMarker.points[idx].y = inwardCloud.at(nextPosition + resolution).y;
					revolutionMeshMarker.points[idx].z = inwardCloud.at(nextPosition + resolution).z;
					idx++;
				}
			}
			
			if ( i == segments - 1 ) { //top cap
			  
				if (objectIsHollow) {
					//triangle out to in
					revolutionMeshMarker.points[idx].x = outwardCloud.at(currentPosition).x; //1st point in circle
					revolutionMeshMarker.points[idx].y = outwardCloud.at(currentPosition).y;
					revolutionMeshMarker.points[idx].z = outwardCloud.at(currentPosition).z;
					idx++;
					revolutionMeshMarker.points[idx].x = outwardCloud.at(nextPosition).x; //2nd point in circle
					revolutionMeshMarker.points[idx].y = outwardCloud.at(nextPosition).y;
					revolutionMeshMarker.points[idx].z = outwardCloud.at(nextPosition).z; 
					idx++;
					revolutionMeshMarker.points[idx].x = inwardCloud.at(currentPosition).x; //1st point in circle
					revolutionMeshMarker.points[idx].y = inwardCloud.at(currentPosition).y;
					revolutionMeshMarker.points[idx].z = inwardCloud.at(currentPosition).z;
					idx++;
					//triangle in to out
					revolutionMeshMarker.points[idx].x = inwardCloud.at(nextPosition).x; //2nd point in circle
					revolutionMeshMarker.points[idx].y = inwardCloud.at(nextPosition).y;
					revolutionMeshMarker.points[idx].z = inwardCloud.at(nextPosition).z; 
					idx++;
					revolutionMeshMarker.points[idx].x = inwardCloud.at(currentPosition).x; //1st point in circle
					revolutionMeshMarker.points[idx].y = inwardCloud.at(currentPosition).y;
					revolutionMeshMarker.points[idx].z = inwardCloud.at(currentPosition).z;
					idx++;
					revolutionMeshMarker.points[idx].x = outwardCloud.at(nextPosition).x; //2nd point in circle
					revolutionMeshMarker.points[idx].y = outwardCloud.at(nextPosition).y;
					revolutionMeshMarker.points[idx].z = outwardCloud.at(nextPosition).z;
					idx++;
					
				} else {
					//triangle out to center
					revolutionMeshMarker.points[idx].x = outwardCloud.at(currentPosition).x; //1st point in circle
					revolutionMeshMarker.points[idx].y = outwardCloud.at(currentPosition).y;
					revolutionMeshMarker.points[idx].z = outwardCloud.at(currentPosition).z;
					idx++;
					revolutionMeshMarker.points[idx].x = outwardCloud.at(nextPosition).x; //2nd point in circle
					revolutionMeshMarker.points[idx].y = outwardCloud.at(nextPosition).y;
					revolutionMeshMarker.points[idx].z = outwardCloud.at(nextPosition).z; 
					idx++;
					revolutionMeshMarker.points[idx].x = center[0]; // center point
					revolutionMeshMarker.points[idx].y = center[1];
					revolutionMeshMarker.points[idx].z = outwardCloud.at(currentPosition).z + circle_segment_height;
					idx++;
				  
				}
		
			}
		}
	}
	for (int i = 0; i < handleHull.size() - 1; i++) { //triggered only if handleHull.size() > 1 , so only if there is a handlehull stored
		Vector first = tfObjectToConveyorCoordinates( handleHull.at(i) );
		revolutionMeshMarker.points[idx].x = first[0];
		revolutionMeshMarker.points[idx].y = first[1];
		revolutionMeshMarker.points[idx].z = first[2];
		idx++;
		Vector second = tfObjectToConveyorCoordinates( handleHull.at(i + 1) );
		revolutionMeshMarker.points[idx].x = second[0];
		revolutionMeshMarker.points[idx].y = second[1];
		revolutionMeshMarker.points[idx].z = second[2];
		idx++;
		revolutionMeshMarker.points[idx].x = center[0];
		revolutionMeshMarker.points[idx].y = center[1];
		revolutionMeshMarker.points[idx].z = center[2];
		idx++;
	}
	return revolutionMeshMarker;
}//Object_tracker::revolutionMeshMarker()

void Object_tracker::objectToWRL( Vector center, Point_cloud outwardCloud, Point_cloud inwardCloud, std::vector<Vector> handleHull ) {

	if ( !objectWRLCreated ) {
		
	Vector robotPosition = transformConveyorToRobotFrame(center);
		  
		
		ofstream wrl(wrlPath);
		if (wrl.is_open()) {
	  
			//wrl header and general material declaration
			wrl << "#VRML V2.0 utf8\n"
			    << "\n"
			    << "#RobotCoords: " << std::to_string(robotPosition[0]) << " " << std::to_string(robotPosition[1]) << " " << std::to_string(robotPosition[2]) << "\n"
			    << "#ObjectCoords: " << std::to_string(center[0]) << " " << std::to_string(center[1]) << " " << std::to_string(center[2]) << "\n"
			    << "\n"
			    << "Group {\n"
			    << "\t" << "children\n"
			    << "\t" << "Group {\n"
			    << "\t\t" << "children\n"
			    << "\t\t" << "Shape {\n"
			    << "\t\t\t" << "appearance\n"
			    << "\t\t\t" << "Appearance {\n"
			    << "\t\t\t\t" << "material\n"
			    << "\t\t\t\t" << "Material {\n"
			    << "\t\t\t\t\t" << "diffuseColor 0.8 0.8 0.8\n"
			    << "\t\t\t\t\t" << "specularColor 1.0 1.0 1.0\n"
			    << "\t\t\t\t\t" << "shininess 0.5\n"
			    << "\t\t\t\t\t" << "transparency 0.0\n"
			    << "\t\t\t\t\t" << "emissiveColor 0.2 0.2 0.2\n"
			    << "\t\t\t\t" << "}\n"
			    << "\t\t\t" << "}\n"
			    << "\n";
	      
	      
		//calculate object points and triangle faceSets
			wrl << "\t\t" << "geometry\n"
			    << "\t\t\t" << "IndexedFaceSet {\n"
			    << "\t\t\t\t" << "coord Coordinate {\n"
			    << "\t\t\t\t\t" << "point [\n";
			Point_cloud outwardConveyor = transformConveyorToObjectFrame( outwardCloud, true );
			Point_cloud inwardConveyor = transformConveyorToObjectFrame( inwardCloud, true );
			int resolution = reconstruct_res;
			int segments = outwardCloud.size() / resolution;
			int totalPointSize = outwardCloud.size() + 2;
			int inwardPointsStart = (int) outwardCloud.size();
			bool objectIsHollow = ( inwardCloud.size() == outwardCloud.size() );
			if (objectIsHollow) { totalPointSize += inwardCloud.size(); }
			
			std::vector< std::string > indexedFaceSet;
			std::vector< std::string > inwardCloudPoints;

			int currentPosition = 0;
			int nextPosition = 1;
			stringstream sstream;
			
			for (int i = 0; i < segments ; i++)  { //outward part of object
				for (int j = 0; j < resolution; j++) {
					currentPosition = (i * resolution + j);
					nextPosition = currentPosition + 1;
					
					if (j == resolution - 1) { nextPosition = i * resolution; }
					//outward point
					sstream << outwardCloud.at(currentPosition).x << " "
						<< outwardCloud.at(currentPosition).y << " "
						<< outwardCloud.at(currentPosition).z << ",\n";
					wrl << "\t\t\t\t\t\t" << sstream.str();
					sstream.str("");
					if (objectIsHollow) { //inward point
					 
						sstream << inwardCloud.at(currentPosition).x << " "
							<< inwardCloud.at(currentPosition).y << " "
							<< inwardCloud.at(currentPosition).z << ",\n";
						inwardCloudPoints.push_back( sstream.str() );
						sstream.str("");
					}
					
					
					if (i == 0) { //bottom cap
						sstream << std::to_string(currentPosition) << ", "	//1st outward point in circle
							<< std::to_string(totalPointSize - 2) << ", "	//center outward point bottom
							<< std::to_string(nextPosition) << ", "		//2nd outward point in circle
							<< "-1,\n";
						indexedFaceSet.push_back(sstream.str());
						sstream.str("");
						
						if (objectIsHollow) {
							sstream << std::to_string(inwardPointsStart + currentPosition) << ", "	//1st inward point in circle
								<< std::to_string(inwardPointsStart + nextPosition) << ", "	//center point bottom
								<< std::to_string(totalPointSize - 2) << ", "			//2nd inward point in circle
								<< "-1,\n";
							indexedFaceSet.push_back(sstream.str());
							sstream.str("");
						}
						
					} else if ( i > 0 ) { //triangles downwards 
						sstream << std::to_string(currentPosition) << ", "		//1st outward point in circle
							<< std::to_string(nextPosition - resolution) << ", " 	//lower outward circle 2nd point
							<< std::to_string(nextPosition) << ", "			//2nd outward point in circle
							<< "-1,\n";
						indexedFaceSet.push_back(sstream.str());
						sstream.str("");
						
						if (objectIsHollow) {
							sstream << std::to_string(inwardPointsStart + nextPosition) << ", "			//1st inward point in circle
								<< std::to_string(inwardPointsStart + currentPosition - resolution) << ", " 	//lower circle inward 2nd point
								<< std::to_string(inwardPointsStart + currentPosition) << ", "			//2nd inward point in circle
								<< "-1,\n";
							indexedFaceSet.push_back(sstream.str());
							sstream.str("");
						}
					}

					if (i == segments - 1) { //top cap
						
						if (objectIsHollow) {
							//triangle out to in
								sstream << std::to_string(currentPosition) << ", "		//1st outward point in circle
								<< std::to_string(nextPosition) << ", "				//2nd outward point in circle
								<< std::to_string(inwardPointsStart + currentPosition) << ", "	//1st inward point in circle
								<< "-1,\n";
							indexedFaceSet.push_back(sstream.str());
							sstream.str("");
							
							//triangle in to out
							sstream << std::to_string(inwardPointsStart + nextPosition) << ", "	//1st inward point in circle
								<< std::to_string(inwardPointsStart + currentPosition) << ", "	//2nd inward point in circle
								<< std::to_string(nextPosition) << ", "				//2nd outward point in circle
								<< "-1,\n";
							indexedFaceSet.push_back(sstream.str());
							sstream.str("");
							
						} else {
							//triangle out to center
							sstream << std::to_string(currentPosition) << ", "	//1st outward point in circle
								<< std::to_string(nextPosition) << ", "		//2nd outward point in circle
								<< std::to_string(totalPointSize - 1) << ", "	//center point top
								<< "-1,\n";
							indexedFaceSet.push_back(sstream.str());
							sstream.str("");
							
						}
						
					} else if ( i < segments - 1 ) { //triangles upwards
						sstream << std::to_string(currentPosition) << ", " 		//1st point in circle
							<< std::to_string(nextPosition) << ", " 		//2nd point in circle
							<< std::to_string(currentPosition + resolution) << ", " //upper circle 1st point
							<< "-1,\n";
						indexedFaceSet.push_back(sstream.str());
						sstream.str("");
						
						if (objectIsHollow) {
							sstream << std::to_string(inwardPointsStart + nextPosition) << ", " 		//1st inward point in circle
								<< std::to_string(inwardPointsStart + currentPosition) << ", " 		//2nd inward point in circle
								<< std::to_string(inwardPointsStart + nextPosition + resolution) << ", "//upper circle 1st inward point
								<< "-1,\n";
							indexedFaceSet.push_back(sstream.str());
							sstream.str("");
						}
					}
					
				}
			}
			//add inside points for hollow objects
			wrl << "\t\t\t\t\t\t#inside points \n";
			for (int i = 0; i < inwardCloudPoints.size(); i++) {
				wrl << "\t\t\t\t\t\t" << inwardCloudPoints.at(i);
			}
			
			{//add top and bottom center points
				wrl << "\t\t\t\t\t\t#center point bottom \n";
				sstream << std::to_string(center[0]) << " " 
					<< std::to_string(center[1]) << " " 
					<< std::to_string(outwardCloud.at(0).z) << ",\n";
				wrl << "\t\t\t\t\t\t" << sstream.str();
				sstream.str("");
				wrl << "\t\t\t\t\t\t#center point top \n";
				sstream << std::to_string(center[0]) << " " 
					<< std::to_string(center[1]) << " " 
					<< std::to_string( outwardCloud.at(outwardCloud.size() - 1).z) << "\n";
				wrl << "\t\t\t\t\t\t" << sstream.str();
				sstream.str("");
			}
			
			
			if (handleHull.size() > 1) { //add handle to wrl file
				int idxFaceSetCount = totalPointSize;
				wrl << "\t\t\t\t\t\t#handle coordinates:\n";
				indexedFaceSet.push_back("#handle coordinates:\n");
				for (int i = 0; i < handleHull.size(); i++) {
					Vector first = tfObjectToConveyorCoordinates( handleHull.at(i) );
					sstream << first[0] << " " << first[1] << " " << first[2] << ",\n";
					wrl << "\t\t\t\t\t\t" << sstream.str();
					sstream.str("");
				}
				int centerDownPos = totalPointSize - 2;
				for (int i = 0; i < handleHull.size() - 1; i++) {
					int firstPos = idxFaceSetCount + i;
					int secPos = idxFaceSetCount + i + 1;
					
					sstream << std::to_string(firstPos) << ", "		//1st point of hull
						<< std::to_string(secPos) << ", "		//2nd point of hull
						<< std::to_string(centerDownPos) << ", "	//center point
						<< "-1,\n";
					indexedFaceSet.push_back(sstream.str());
					sstream.str("");
				}
			}
			
			
			
						
			wrl << "\t\t\t\t\t" << "]\n"
			      << "\t\t\t\t" << "}\n"
			      << "\n";
		
		//store indexedFaceSet	
			wrl << "\t\t\t\t" << "coordIndex [\n";
			for (size_t i = 0; i < indexedFaceSet.size(); i++) {
				  wrl << "\t\t\t\t\t" << indexedFaceSet.at(i);
			}

			wrl << "\t\t\t\t" << "]\n"
			      << "\t\t\t" << "ccw TRUE\n"
			      << "\t\t\t" << "solid FALSE\n"
			      << "\t\t\t" << "convex TRUE\n"
			      << "\t\t\t" << "creaseAngle 10\n"
			      << "\t\t\t" << "}\n"
			      << "\t\t" << "}\n"
			      << "\t" << "}\n"
				    << "}\n";

	      
			wrl.close();
			objectWRLCreated = true;
			ROS_WARN("\n\n\nWRL file printed!\n\n\n");
		} else {
			ROS_ERROR("Unable to store reconstructed object as wrl file!");
		}
	}
  
  
}//Object_tracker:objectToWRL()

bool Object_tracker::removeUnobserved(Tracked_object to) {
	return to.unobserved >= unobserved_frames_threshold;
}//Object_tracker::removeUnobserved()

Object_tracker::Matching Object_tracker::calculate_object_distance ( const int tracked_id, const int observed_id  ) const {
    const Observed_object* observed = &observed_objects[observed_id];
    const Tracked_object* tracked = &tracked_objects[tracked_id];
    Vector tracked_position;
    double covariance;

    tracked->predict_position( observed->time, &tracked_position, &covariance );

    /* calculate component-wise extended mahalanobis distance */
    Vector matching = ( tracked_position - observed->position ).array().abs();
    covariance = sqrt( covariance + observed->covariance );
    matching /= covariance;

    return Matching(tracked_id, observed_id, matching.norm());
}//Object_tracker::calculate_object_distance()

void Object_tracker::reconstruct_by_angle( const int tracked_id, const int observed_id ) {
	
	Observed_object observation = observed_objects[observed_id];
	Tracked_object* match = &tracked_objects[tracked_id];
	if (match->getObject().empty()) {
		ROS_WARN("Reconstruct_by_angle: Empty match object, adding object of observation!!");
		Point_cloud_ptr observedObject = observation.observedCloud;
		match->update_object( *observedObject, getKinectCoordinates(observation.position), 0.0 );
	}
	std::vector<sacSegment> segmentedMatchedObject = getSegmentedObject( match->getObject().makeShared() );

	if (match->getDegreesReconstructed() > (360.0 - 0.5 * object_reconstruction_angle) ) {//construction completed
		ROS_INFO("Frame %d: OBJ_RECONSTRUCT: Completed (%f degrees of object reconstructed).", objectDetectedCounter, match->getDegreesReconstructed() );
		reconstructed_object_publisher.publish( transformConveyorToObjectFrame( match->getObject() , true ) );
		calculateSolidOfRevolution( segmentedMatchedObject, false );
		
	} else if ( (match->getObject().size() == 0) && useObjectFrame) {//first construction

		Point_cloud reconObj = *(observation.observedCloud);
		constructedObject = reconObj;
		_last_construction_position = getKinectCoordinates(observation.position);
		match->update_object( reconObj, getKinectCoordinates(observation.position), 180.0 );
		ROS_DEBUG("Frame %d: OBJ_RECONSTRUCT: ObjectReconstruction: Added objectCloud.", objectDetectedCounter);
		reconstructed_object_publisher.publish( transformConveyorToObjectFrame( match->getObject() , true ) );
		calculateSolidOfRevolution( getSegmentedObject( reconObj.makeShared() ), false );

	} else if (( match->getObject().size() == 0) || ( getKinectCoordinates(match->get_position())[1] < -0.5 ) ) {//No construction possible

		ROS_DEBUG( "Frame %d: OBJ_RECONSTRUCT: No object stored & no filtered object data available!", objectDetectedCounter);

	} else {//construction possible, continue with further calculations

		Vector old_position = match->getLastObjectPosition();
		Vector new_position = getKinectCoordinates(observation.position);
		old_position.normalize();
		new_position.normalize();
		double angleDiffGrad;
		double angleDiffRad;
		Eigen::Quaternionf rotation;

		if ( (old_position[1] > 0.0) && (new_position[1] < 0.0) ) {// object moved from back to front rail, turning 180 degrees
			angleDiffRad = 0.5 * M_PI;
			angleDiffGrad = 180.0;
			rotation.w() = 0.0;
			rotation.x() = 0.0;
			rotation.y() = 0.0;
			rotation.z() = 1.0;
			ROS_DEBUG( "Frame %d: OBJ_RECONSTRUCT: AngleDiff is 180 (changed conveyor rail)." , objectDetectedCounter);
		} else if ( old_position[1] > 0.0 ){ //back side of conveyor -> moving right
			angleDiffRad = acos( old_position.dot(new_position) );
			angleDiffGrad = angleDiffRad * 180 / M_PI;
			ROS_DEBUG( "Frame %d: OBJ_RECONSTRUCT: AngleDiff is %f (rotation CCW)." , objectDetectedCounter, angleDiffGrad);
			rotation.w() = cos( angleDiffRad / 2.0 );
			rotation.x() = 0.0;
			rotation.y() = 0.0;
			rotation.z() = sin( angleDiffRad / 2.0 );
		} else { //front side of conveyor -> moving left
			angleDiffRad = - acos( old_position.dot(new_position) );
			angleDiffGrad = angleDiffRad * 180 / M_PI;
			ROS_DEBUG( "Frame %d: OBJ_RECONSTRUCT: AngleDiff is %f (rotation CW)." , objectDetectedCounter, angleDiffGrad);
			rotation.w() = cos( angleDiffRad / 2.0 );
			rotation.x() = 0.0;
			rotation.y() = 0.0;
			rotation.z() = sin( angleDiffRad / 2.0 );
		}

		Point_cloud oldObject = match->getObject();
		Point_cloud_ptr objectTL( new Point_cloud);
		pcl::copyPointCloud(oldObject, *objectTL);
		pcl::transformPointCloud(*objectTL, *objectTL, Vector(0,0,0), rotation);

		Point_cloud_ptr newPart = observation.observedCloud;
		Point_cloud newPartCutted;

		if (angleDiffGrad != angleDiffGrad) {
			//ROS_INFO("Frame %d: OBJ_RECONSTRUCT: ERROR angleDiff IS NAN.", objectDetectedCounter);
		} else if ( fabs(angleDiffGrad) < object_reconstruction_angle ) {
			//ROS_INFO("Frame %d: OBJ_RECONSTRUCT: Using old cloud from %f degrees old view.", objectDetectedCounter, angleDiffGrad );
			reconstructed_object_publisher.publish( transformConveyorToObjectFrame( *objectTL , true ) );
			calculateSolidOfRevolution( getSegmentedObject( oldObject.makeShared() ), false );
		} else {
			newPartCutted = cut_object(newPart, angleDiffGrad);
			for (int i = 0; i < newPartCutted.size(); i++) { //only for visualization
			      newPartCutted.at(i).r = (match->getDegreesReconstructed() / 180.0);
			      newPartCutted.at(i).g = (match->getDegreesReconstructed() / 360.0);
			      newPartCutted.at(i).b = (match->getDegreesReconstructed() / 360.0);
			}
			if (angleDiffGrad < 0.0) { angleDiffGrad *= -1.0; }
			constructedObject = (newPartCutted + *objectTL);
			_last_construction_position = getKinectCoordinates(observation.position);

			match->update_object( (newPartCutted  + *objectTL ), getKinectCoordinates(observation.position), angleDiffGrad );
			ROS_DEBUG("Frame %d: OBJ_RECONSTRUCT: Mixing clouds from %f degrees different view. \n", objectDetectedCounter, angleDiffGrad );
			reconstructed_object_publisher.publish( transformConveyorToObjectFrame( match->getObject() , true ) );
			std::vector<sacSegment> segmentedObject = getSegmentedObject( match->getObject().makeShared() );
			calculateSolidOfRevolution( segmentedObject, false );
			

		}
		//ROS_INFO("Frame %d: OBJ_RECONSTRUCT: Object viewed from %f degrees, %d points in object. \n", objectDetectedCounter, match->getDegreesReconstructed(), (int) match->getObject().size() );

	}

}//Object_tracker::reconstruct_by_angle()

Object_tracker::Point_cloud Object_tracker::cut_object( Point_cloud_ptr object, double angle) {

	Indices inlierIndices;
	double maxInclination = tan(angle * M_PI / 180.0);

	if (angle == 180.0) { //no cut
		ROS_DEBUG("Frame %d: CUTTING: Angle is %f, maxInclination is %f Return WHOLE OBJECT!", objectDetectedCounter, angle, maxInclination);
		return *object;
	} else if (angle > 0.0) { //Obj moving right, rotating CCW, return pie piece of left handside
		for (int i = 0; i < object->size(); i++) {
			Point p = object->at(i);
			double inclination = (p.y / p.x);
			if ( (p.x > 0.0) && (p.y >= 0.0) && (p.x < object_maxwidth) && ( inclination <= maxInclination ) ) {
				inlierIndices.push_back(i);
			}
		}
		ROS_DEBUG("Frame %d: CUTTING: Angle is %f, maxInclination is %f Return LEFT PIECE!", objectDetectedCounter, angle, maxInclination);

	} else if (angle < 0.0) { //Obj moving left, rotating CW, return pie piece of right object cloud
		for (int i = 0; i < object->size(); i++) {
			Point p = object->at(i);
			double inclination = (p.y / p.x);
			if ( (p.x < 0.0) && (p.y >= 0.0) && (p.x > -object_maxwidth) && ( inclination >= maxInclination ) ) {
				inlierIndices.push_back(i);
			}
		}
		ROS_INFO("Frame %d: CUTTING: Angle is %f, maxInclination is %f Return RIGHT PIECE!", objectDetectedCounter, angle, maxInclination);

	} else {
		ROS_INFO( "ERROR!! Angle is NAN!" );

	}

	Point_cloud cutCloud( *object, inlierIndices);
	ROS_INFO("Frame %d: CUTTING: %d of %d left after cutting.", objectDetectedCounter, (int) cutCloud.size(), (int) object->size() );

	return cutCloud;
}//Object_tracker::cut_object()

/**
 * Segments the object into horizontal slices of a given height and checks if the point-size of each slice meets the minimum number 
 * to be able to calculate its circle center using RANSAC
 * @return Vector of SacSegment containing the object cloud of each slice, its vertical position in the object (height) and its validity for RANSAC calculation. Sorted from bottom to top slice.
 */
std::vector<Object_tracker::sacSegment> Object_tracker::getSegmentedObject( Point_cloud_ptr objectCloud ) {
  
  
	// sort points by z coordinate (height)
	std::sort( objectCloud->points.begin(), objectCloud->points.end(), [](const Point& a, const Point& b) { return a.z < b.z; });
	// number of circles to calculate
	double height = objectCloud->points.back().z;
	int num_slices = ( height - objectCloud->points.front().z ) / ( circle_segment_height );
	double segment_ceiling = objectCloud->points.front().z + circle_segment_height;
	int copy_index = 0;

	std::vector<sacSegment> segments;


	for( int i = 0; i < num_slices; i++ ) {
	    Point_cloud_XYZ_ptr segment( new Point_cloud_XYZ );
		segment->clear();
		segment_ceiling += circle_segment_height;

		// extract height segment into 2D cloud for ransac
		while( copy_index < objectCloud->size() && objectCloud->points[copy_index].z < segment_ceiling ){
		    pcl::PointXYZ point;
		    point.x = objectCloud->points[copy_index].x;
		    point.y = objectCloud->points[copy_index].y;
		    point.z = 0.0;
		    segment->push_back( point );
		    ++copy_index;
		}

		segment->header.frame_id = objectCloud->header.frame_id;
		segment->header.stamp = objectCloud->header.stamp;
		bool validSegment = segment->size() >= circle_segment_min_points; // segments with too few points won't lead to stable estimations
		
		segments.push_back( sacSegment(segment, segment_ceiling, validSegment ) );
		
	}
	
	return segments;
}//Object_tracker::getSegmentedObject()

/**
 * Calculates the sliced object centers for an upright solid of revolution (with its rotation axis parallel to z direction)
 * @return Vector containing ransac circle results for each horizontal slice, or an empty vector
 */
Object_tracker::sacResult Object_tracker::calculateSolidOfRevolution(std::vector<sacSegment> segmentedObject, bool singleFrame ) {
    
	double weight = 0.0; // sum of the weights
	Vector center( 0, 0, 0 ); // weighted sum of the centers
	std::vector<double> radii; //weighted sum of all radii;
	int supporters = 0;
	pcl::PCLHeader objectHeader = segmentedObject.at(0).cloud->header;
	Point_cloud reconstructedCloud;
	reconstructedCloud.header = objectHeader;
	double height = segmentedObject.back().cloud->points.back().z;
	
	// number of circles to calculate
	int num_slices = segmentedObject.size();
	
	radii.resize(num_slices, 0.0);

	vector<thread> threads;
	sacResult results[100];


	Vector prevCtr;
	//int i = 0;
	for (int i = 0; i < num_slices; i++) {
		if (!segmentedObject[i].valid) {continue;}
		auto seg = segmentedObject.at(i);
		if (multiThreaded) {
			    threads.push_back(std::thread(&Object_tracker::threadSAC, this,seg.cloud,seg.height, results, i));
		} else {
			    sac->setInputCloud( seg.cloud );
			    pcl::ModelCoefficients coefficients; // for circle: 0 is center_x, 1 is center_y, 2 is radius
			    pcl::PointIndices inliers;
			    sac->segment( inliers, coefficients );
			    // circles with too few supporters are irrelevant
			    if( inliers.indices.size() < 40 ) continue;
			    // determine center coordinates
			    Vector segment_center( coefficients.values[0],
				coefficients.values[1],
				seg.height - circle_segment_height * 0.5);
			    double segment_radius = coefficients.values[2];

			    // determine weight
			    double segment_weight = static_cast<double>( inliers.indices.size() );
			    // add weighted coordinates to sum
			    weight += segment_weight;
			    center += segment_center * segment_weight;
			    radii[i] = segment_radius;

			    supporters++;

			    if( useObjectFrame && (i != 0) ) {

				    //lines between segment centers
				    std::vector<Vector> lineStripPoints;
				    lineStripPoints.push_back( prevCtr );
				    lineStripPoints.push_back( segment_center );
				    reconstructed_object_markers.markers.push_back( rosLineStripMarker( (i + 500), objectHeader.frame_id, lineStripPoints, Vector(1.0, 1.0, 0.0), 0.2 ) );

				    //estimated object radius
				    reconstructed_object_markers.markers.push_back( rosCircleMarker( (i + 600), objectHeader.frame_id, segment_center, segment_radius) );

			    }
			    prevCtr = segment_center;
		}
	}//i++;
	
	if (multiThreaded) {
		for(int i = 0; i < threads.size();i++){ threads[i].join(); }
		for(int i = 0; i < num_slices; i++) {
			if (results[i].valid) {
				weight += results[i].weight;
				center += results[i].center * results[i].weight;
				radii[i] = results[i].radii.front();
				supporters++;

				if (i > 0 && results[i-1].valid) {
					//visualize middlepoint variation between segments
					std::vector<Vector> lineStripPoints;
					lineStripPoints.push_back(results[i-1].center);
					lineStripPoints.push_back(results[i].center);
					reconstructed_object_markers.markers.push_back( rosLineStripMarker( (i + 800), objectHeader.frame_id, lineStripPoints, Vector(1.0, 1.0, 0.0), 0.2 ) );
				}
			}
		}
	}
	
	center /= weight;
	center[2] = object_conveyor_offset;
	bool validCalculation = (supporters > 1);

	return sacResult(center, radii, height, weight, -1, validCalculation );
}//Object_tracker::calculateSolidOfRevolution


//---------------Transformation and format conversion functions--------------------------
Eigen::Vector3f Object_tracker::getKinectCoordinates ( Vector convCoords ) {

	Vector kinectCoords(100.f, 100.f, 100.f);

	try{
		conveyor_to_kinect2_TF = tfBuf.lookupTransform("kinect2_depth_frame", "conveyor_plane", ros::Time(0));
		Point_cloud_XYZ middlepointCloud = *(new Point_cloud_XYZ);
		pcl::PointXYZ middlepoint(convCoords.x(), convCoords.y(), convCoords.z());
		middlepointCloud.push_back(middlepoint);

		Vector translation = tfTranslationToVector(conveyor_to_kinect2_TF);
		Eigen::Quaternionf rotation = tfRotationToQuaternion(conveyor_to_kinect2_TF);

		pcl::transformPointCloud(middlepointCloud, middlepointCloud, translation, rotation);
		pcl::PointXYZ middlepointTF = middlepointCloud.front();
		kinectCoords = Vector( middlepointTF.x, middlepointTF.y, middlepointTF.z);

	} catch (tf2::TransformException &ex) {
		ROS_WARN("Could not look up conveyor to kinect2 transformation!");

	}
	return kinectCoords;
}//Object_tracker::getKinectCoordinates

std_msgs::Float64MultiArray Object_tracker::transformConveyorToRobotCoordinates ( VectorRot convCoordsAndRot, double gripperRotation ) {

	Vector robotRotationBaseToTCP(90.0, 0.0, 180.0);
	Vector convCoords(convCoordsAndRot[0], convCoordsAndRot[1], convCoordsAndRot[2]);
	
	Vector robotCoords = transformConveyorToRobotFrame(convCoords);
	
	std_msgs::Float64MultiArray cartesianTCPPose;
	cartesianTCPPose.data.resize(6);
	cartesianTCPPose.data[0] = robotCoords[0];
	cartesianTCPPose.data[1] = robotCoords[1];
	cartesianTCPPose.data[2] = robotCoords[2];
	cartesianTCPPose.data[3] = robotRotationBaseToTCP[0] - convCoordsAndRot[3] + fixedTCPRotation + gripperRotation;
	cartesianTCPPose.data[4] = robotRotationBaseToTCP[1];
	cartesianTCPPose.data[5] = robotRotationBaseToTCP[2];
	return cartesianTCPPose;
	
}//Object_tracker::transformConveyorToRobotCoordinates

Eigen::Vector3f Object_tracker::transformConveyorToRobotFrame ( Vector conveyorCoordinates ) {
	
	Vector robotCoords;
	try{
		Vector kinectCoords = getKinectCoordinates( conveyorCoordinates );
		kinect2_to_robot_TF = tfBuf.lookupTransform("kinect2_depth_frame", robot_baselink, ros::Time(0));
		Point_cloud_XYZ transformationCloud = *(new Point_cloud_XYZ);
		pcl::PointXYZ conveyorPoint(kinectCoords.x(), kinectCoords.y(), kinectCoords.z());
		transformationCloud.push_back(conveyorPoint);

		Vector robotTranslation = tfTranslationToVector(kinect2_to_robot_TF);
		Eigen::Quaternionf robotRotation = tfRotationToQuaternion(kinect2_to_robot_TF);
		Eigen::Quaternionf ID;
		ID.setIdentity();
		//translate from kinect2 to object_frame
		pcl::transformPointCloud( transformationCloud, transformationCloud, -robotTranslation, ID);
		pcl::transformPointCloud( transformationCloud, transformationCloud, Vector(0,0,0), robotRotation.inverse());
		
		pcl::PointXYZ conveyorPointTF = transformationCloud.front();
		robotCoords[0] = conveyorPointTF.x;
		robotCoords[1] = conveyorPointTF.y;
		robotCoords[2] = conveyorPointTF.z;
	} catch (tf2::TransformException &ex) {
		ROS_WARN("Could not look up kinect to robot transformation!");
	}
	return robotCoords;
}//Object_tracker::transformConveyorToRobotFrame

Eigen::Vector3f Object_tracker::tfTranslationToVector( geometry_msgs::TransformStamped transformation ) { 
	Vector translation;
	translation.x() = transformation.transform.translation.x;
	translation.y() = transformation.transform.translation.y;
	translation.z() = transformation.transform.translation.z;
	return translation;
}//Object_tracker::tfTranslationToVector

Eigen::Quaternionf Object_tracker::tfRotationToQuaternion( geometry_msgs::TransformStamped transformation ) {
	Eigen::Quaternionf rotation;
	rotation.w() = transformation.transform.rotation.w;
	rotation.x() = transformation.transform.rotation.x;
	rotation.y() = transformation.transform.rotation.y;
	rotation.z() = transformation.transform.rotation.z;
	return rotation; 
}//Object_tracker::tfRotationToQuaternion

Eigen::Vector3f Object_tracker::tfObjectToConveyorCoordinates( Vector objCoords ) {

		Point_cloud centerCloud = *(new Point_cloud);
		Point center;
		center.x = objCoords[0];
		center.y = objCoords[1];
		center.z = objCoords[2];
		centerCloud.push_back(center);
		Point_cloud transformedCloud = transformConveyorToObjectFrame( centerCloud, true );
		Point centerConv = transformedCloud.front();
		Vector convCoords(centerConv.x, centerConv.y, centerConv.z);

		return convCoords;
}//Object_tracker::tfObjectToConveyorCoordinates

Eigen::Vector3f Object_tracker::tfConveyorToObjectCoordinates( Vector convCoords ) {

		Point_cloud centerCloud = *(new Point_cloud);
		Point center;
		center.x = convCoords[0];
		center.y = convCoords[1];
		center.z = convCoords[2];
		centerCloud.push_back(center);

		Point_cloud transformedCloud = transformConveyorToObjectFrame( centerCloud, false );
		Point centerObj = transformedCloud.front();
		Vector objCoords(centerObj.x, centerObj.y, centerObj.z);

		return objCoords;
}//Object_tracker::tfObjectToConveyorCoordinates

bool Object_tracker::sendObjectTransformation( Vector objectMiddle, int id ) {
	Vector origin = getKinectCoordinates( objectMiddle );
	Vector normalPoint = getKinectCoordinates( Vector(objectMiddle.x(), objectMiddle.y(), objectMiddle.z() + 0.1));
	Vector conveyorNormal = normalPoint - origin;
	conveyorNormal.normalize();

	Vector objectWidth = normalPoint.cross(origin);
	objectWidth.normalize();
	Vector objectDepth = -objectWidth.cross(conveyorNormal);
	objectDepth.normalize();

	Eigen::Quaternionf rotateToZ;
	rotateToZ.setFromTwoVectors(Vector(0,0,1), conveyorNormal);
	rotateToZ.normalize();

	Vector prerotatedX = rotateToZ._transformVector(Vector(1,0,0));
	Eigen::Quaternionf rotateToX;
	rotateToX.setFromTwoVectors(prerotatedX, objectWidth);

	Eigen::Quaternionf rotateToObjectCoords = rotateToX * rotateToZ;

	kinect2_to_object_TF.header.stamp = ros::Time::now();
	kinect2_to_object_TF.header.frame_id = "kinect2_depth_frame";
	kinect2_to_object_TF.child_frame_id = "object_frame"; //+ std::to_string(id);
	kinect2_to_object_TF.transform.translation.x = origin[0];
	kinect2_to_object_TF.transform.translation.y = origin[1];
	kinect2_to_object_TF.transform.translation.z = origin[2];
	kinect2_to_object_TF.transform.rotation.w = rotateToObjectCoords.w();
	kinect2_to_object_TF.transform.rotation.x = rotateToObjectCoords.x();
	kinect2_to_object_TF.transform.rotation.y = rotateToObjectCoords.y();
	kinect2_to_object_TF.transform.rotation.z = rotateToObjectCoords.z();

	tfb.sendTransform(kinect2_to_object_TF);
	return (origin != Vector(100.f, 100.f, 100.f));
}//Object_tracker::sendObjectTransformation

Object_tracker::Point_cloud Object_tracker::transformConveyorToObjectFrame(Point_cloud inputCloud, bool inverse) {
	Point_cloud objectCloud;
	Vector toKinectTranslation;
	toKinectTranslation[0] = conveyor_to_kinect2_TF.transform.translation.x;
	toKinectTranslation[1] = conveyor_to_kinect2_TF.transform.translation.y;
	toKinectTranslation[2] = conveyor_to_kinect2_TF.transform.translation.z;
	Vector toObjectTranslation;
	toObjectTranslation[0] = kinect2_to_object_TF.transform.translation.x;
	toObjectTranslation[1] = kinect2_to_object_TF.transform.translation.y;
	toObjectTranslation[2] = kinect2_to_object_TF.transform.translation.z;

	Eigen::Quaternionf toKinectRotation;
	toKinectRotation.w() = conveyor_to_kinect2_TF.transform.rotation.w;
	toKinectRotation.x() = conveyor_to_kinect2_TF.transform.rotation.x;
	toKinectRotation.y() = conveyor_to_kinect2_TF.transform.rotation.y;
	toKinectRotation.z() = conveyor_to_kinect2_TF.transform.rotation.z;
	Eigen::Quaternionf toObjectRotation;
	toObjectRotation.w() = kinect2_to_object_TF.transform.rotation.w;
	toObjectRotation.x() = kinect2_to_object_TF.transform.rotation.x;
	toObjectRotation.y() = kinect2_to_object_TF.transform.rotation.y;
	toObjectRotation.z() = kinect2_to_object_TF.transform.rotation.z;

	Eigen::Quaternionf ID;
	ID.setIdentity();

	if (!inverse) {
		//translate from conveyor_plane to kinect2
		pcl::transformPointCloud( inputCloud, objectCloud, toKinectTranslation, toKinectRotation);
		//translate from kinect2 to object_frame
		pcl::transformPointCloud( objectCloud, objectCloud, -toObjectTranslation, ID);
		pcl::transformPointCloud( objectCloud, objectCloud, Vector(0,0,0), toObjectRotation.inverse());
		objectCloud.header.frame_id = kinect2_to_object_TF.child_frame_id;
	} else {
		//translate from object_frame to kinect2
		pcl::transformPointCloud( inputCloud, objectCloud, toObjectTranslation, toObjectRotation);
		//translate from kinect2 to conveyor_plane
		pcl::transformPointCloud( objectCloud, objectCloud, -toKinectTranslation, ID);
		pcl::transformPointCloud( objectCloud, objectCloud, Vector(0,0,0), toKinectRotation.inverse());
		objectCloud.header.frame_id = "conveyor_plane";
	}
	return objectCloud;
}//Object_tracker::transformConveyorToObjectFrame

Eigen::Vector3f Object_tracker::convertPoseToTranslation( VectorRot vecRot ) {
	 return Vector(vecRot[0], vecRot[1], vecRot[2]);
}//Object_tracker::convertPoseToTranslation


//-------------Robot communication functions----------------------
bool Object_tracker::generateTrajectory( Vector currentPosition, double objectHeight, Vector handleDirection, double interceptionTime ) {
  
	
	if (objectHeight <= object_minheight) { ROS_ERROR("No object height stored, so no trajectory can be generated!"); return false; }
	
	trajectory_marker_msg.markers.clear();
	//update current object position
	conveyorTrajectory.updateTrajectory( currentPosition );
	
	double gripperRotation = conveyorTrajectory.getAngleTowardsObjectDirection( currentPosition, handleDirection);
	ROS_INFO("Handle angle towards object heading is %f deg clockwise.", gripperRotation);
	double grippingHeight = objectHeight - gripperVerticalTCPStandoff;
	int trajectoryTacts = floor( ( conveyorTrajectory.getVisibleTimeLeft() - interceptionTime ) / robotTactTime );
	int requiredTacts = 2 * ceil( gripperApproachTime.toSec() /robotTactTime) + ceil( grippingTime.toSec() /robotTactTime);
	int tactsLeftAfterExecution = trajectoryTacts - requiredTacts;
	int oneSecondTact = floor( 1 / robotTactTime );
	
	if (tactsLeftAfterExecution < -oneSecondTact) {
	    ROS_ERROR("Object is not visible for enough time to be gripped! (Only visible for %f%% seconds)", conveyorTrajectory.getVisibleTimeLeft() );
	    trajectory_marker_msg.markers.clear();
	    return false;
	} else if (tactsLeftAfterExecution < 0) {
		ROS_WARN("Object not visible for enough time to perform interception at intended time. Trying to grip one second earlier");
		interceptionTime -= 1.0;
		trajectory_marker_msg.markers.clear();
		
		
	}
	//fetch generated trajectory
	std::vector<Eigen::Vector4f> trajectory = conveyorTrajectory.generateTrajectory( grippingHeight, verticalApproachDistance, interceptionTime, robotTactTime, gripperApproachTime, grippingTime );

	int trajectoryDimension = trajectory.size();

	if (trajectoryDimension == 0) {
		ROS_ERROR("Empty trajectory returned although one should have been generated!");
		return false;
	} else {
		ROS_DEBUG("Frame %d: Trajectory after interception in %f seconds returned %d single points.",
			objectDetectedCounter, interceptionTime, trajectoryDimension );
		double visibleTimeLeft = conveyorTrajectory.getVisibleTimeLeft();
		ROS_INFO("Frame %d: Object visible for the next %f seconds.", objectDetectedCounter, visibleTimeLeft );
		
		VectorRot velocityDirPoint = trajectory[1] + trajectory[0];

		std_msgs::Float64MultiArray interceptionPose = transformConveyorToRobotCoordinates(trajectory[1], gripperRotation);
		
		std_msgs::Float64MultiArray interceptionVelocityPose = transformConveyorToRobotCoordinates(velocityDirPoint, gripperRotation);
		
		for (int i = 0; i < 3; i++) {
			interceptionVelocityPose.data[i] -= interceptionPose.data[i];
		}
		for (int i = 3; i < 6; i++) {
		    interceptionVelocityPose.data[i] = 0.0; //no rotational velocity
		}

		{//generate actionGoal and send to rsi_node
			industrial_kuka_ros::FollowObjectGoal goal;

			goal.header.frame_id = "conveyor_plane";
			goal.header.stamp = ros::Time().now();
			goal.interceptionTime = ( conveyorTrajectory.getParameterTime() + ros::Duration(interceptionTime) );
			goal.interceptionVelocityDir = interceptionVelocityPose;
			goal.interceptionPose = interceptionPose;
			goal.dropPose = initialRobotPose;

			for (int i = 2; i < trajectoryDimension; i++) {
				goal.trajectoryPoses.push_back( transformConveyorToRobotCoordinates( trajectory[i], gripperRotation ) );	
			}
			robotTrajectoryActionClient.sendGoal(goal,
						      boost::bind(&Object_tracker::robot_trajectory_done, this, _1, _2),
						      Client::SimpleActiveCallback(),
						      boost::bind(&Object_tracker::robot_trajectory_feedback , this, _1) );
		}
		
		{//visualization of the trajectory which is sent to rsi_node
			double interceptVis = ( (conveyorTrajectory.getParameterTime() + ros::Duration(interceptionTime)) - ros::Time().now() ).toSec();
			int robotTrajectoryText = 91012;
			std::vector<Vector> robotTrajectoryLine;
			for (int i = 2; i < trajectoryDimension; i++) {
				if ( (i % 10) == 0 ) {
					robotTrajectoryLine.push_back( convertPoseToTranslation(trajectory[i]) );
				}

			}
			double completionTime = interceptVis + trajectoryDimension * robotTactTime;
			trajectory_marker_msg.markers.push_back( rosLineStripMarker(91011, "conveyor_plane", 
										    robotTrajectoryLine, Vector(1.0, 0.0, 0.0), completionTime ) );
			trajectory_marker_msg.markers.push_back( rosArrowMarker( 90011, "conveyor_plane",
										 convertPoseToTranslation(trajectory[1]),
										 convertPoseToTranslation( velocityDirPoint ),
										 interceptVis ) );
			robotTrajectoryVisualization_publisher.publish( trajectory_marker_msg );

		}

		return true;
	}

}//Object_tracker::generateTrajectory

void Object_tracker::robot_trajectory_done( const actionlib::SimpleClientGoalState& state, const industrial_kuka_ros::FollowObjectResultConstPtr& result) {

	robotActive = false;
	double interceptionDelay = result->interception_delay;
	
  
	if ( state == actionlib::SimpleClientGoalState::ABORTED ) {
		
		if (interceptionDelay < 0.01) { //lengthen time to interception (only when trajectory was valid)
			interceptionTime += ( -1.2 * interceptionDelay + robotDelay );
		} else if (interceptionDelay < 0.5) {
			interceptionTime += (1.5 * interceptionDelay + robotDelay);
		}
		ROS_WARN("Frame %d: Robot was unable to execute trajectory (was %f sec. too slow)!\n"
			  "Interception now targeted in %f sec. !", objectDetectedCounter, -interceptionDelay, interceptionTime);
		
	} else if ( state == actionlib::SimpleClientGoalState::SUCCEEDED ) {
		ROS_INFO("Frame %d: The object could be successfully followed!\n"
			"----Resetting for next gripping after object was dropped ----", objectDetectedCounter);
		if (interceptionDelay > 2.0) { //shorten time delay for next cycle
			interceptionTime -= (0.5 * interceptionDelay); 
			ROS_INFO("Frame %d: Interception could've been faster -> Trying to intercept in %f sec. now!", objectDetectedCounter, interceptionTime);
		}
		//useObjectFrame = false;//object was gripped!
		//objectDetectedCounter = -1;//object was gripped!
		//objectFrameCalcCtr = -1;//object was gripped!
		objectWRLCreated = false; //generate new object wrl file as previous object is no longer on the conveyor
		
		ros::Duration(0.5).sleep(); //wait until gripper is in open position (object was dropped)
		gripperJointPosition_publisher.publish( openPosition120 );
		gripperActive = false; //enable gripping of new object
		ROS_INFO("The robot is now inactive and waiting for the next gripping cycle!");
		
	}

}//Object_tracker::robot_trajectory_goal

void Object_tracker::robot_trajectory_feedback ( const industrial_kuka_ros::FollowObjectFeedbackConstPtr& followObjectFeedback ) {
	uint8_t activePhase = followObjectFeedback->activePhase;
	ros::Time phaseCompletionTime = followObjectFeedback->phaseCompletionTime;
	double secondsToCompletion = (phaseCompletionTime - ros::Time::now()).toSec();
	robotActive = true;
		
	if (activePhase == 1) { /* Interception execution phase */
		
		ROS_INFO("Frame %d: Robot currently executing interception phase (will be completed in %f seconds).", objectDetectedCounter, secondsToCompletion);
		//TODO implement openRAVE grip position calculation here
		
	} else if ( activePhase == 2 ) { /* FollowObject execution phase */

		ROS_INFO_ONCE("Interception point reached!");
		ROS_INFO("Frame %d: Robot currently in FollowObject phase (will be completed in %f seconds).",objectDetectedCounter, secondsToCompletion);
		pauseTracking = true;
		ros::Duration startWhileApproaching(1.0); //amount of time which SDH begins the gripping process prior to the robot reaching final height
		(gripperApproachTime - startWhileApproaching).sleep();
		ROS_WARN("SENDING GRIPPING COMMAND TO SDH");
		gripperActive = true;
		gripperEffort_publisher.publish ( gripEffort );
		
		
	} else if ( activePhase == 3 ) { /* taking gripped object to Dropzone phase */
	  
		ROS_INFO("Frame %d: Robot executing return-and-drop phase (will be completed in %f seconds).",objectDetectedCounter, secondsToCompletion);
		secondsToCompletion = (phaseCompletionTime - ros::Time::now()).toSec();
		useObjectFrame = false;//object was gripped!
		objectDetectedCounter = -1;//object was gripped!
		objectFrameCalcCtr = -1;//object was gripped!
		ros::Duration( secondsToCompletion ).sleep();
		
		pauseTracking = false;

	} else {
		ROS_ERROR("Robot triggered an erroneous phase! (This should not happen)");
	}

}//Object_tracker::robot_trajectory_feedback


//---------------Visualization helper functions------------------
visualization_msgs::Marker Object_tracker::rosPositionMarker( int markerID, std::string frameID, Vector position, double lifetimeInSec ) {

	visualization_msgs::Marker PositionMarker;
	PositionMarker.header.frame_id = frameID;
	PositionMarker.header.stamp = ros::Time().now();
	PositionMarker.ns = "conveyor_object_tracker";
	PositionMarker.id = markerID;
	PositionMarker.type = visualization_msgs::Marker::SPHERE;
	PositionMarker.action = visualization_msgs::Marker::ADD;
	PositionMarker.lifetime = ros::Duration(lifetimeInSec);
	PositionMarker.color.r = 0.0;
	PositionMarker.color.g = 1.0;
	PositionMarker.color.b = 0.0;
	PositionMarker.color.a = 0.8;
	PositionMarker.scale.x = 0.025;
	PositionMarker.scale.y = 0.025;
	PositionMarker.scale.z = 0.025;
	PositionMarker.pose.position.x = position[0];
	PositionMarker.pose.position.y = position[1];
	PositionMarker.pose.position.z = position[2];
	return PositionMarker;
}//Object_tracker::rosPositionMarker

visualization_msgs::Marker Object_tracker::rosCircleMarker( int markerID, std::string frameID, Vector position, double radius ) {

	visualization_msgs::Marker circleMarker;
	circleMarker.header.frame_id = frameID;
	circleMarker.header.stamp = ros::Time().now();
	circleMarker.ns = "conveyor_object_tracker";
	circleMarker.type = visualization_msgs::Marker::CYLINDER;
	circleMarker.action = visualization_msgs::Marker::ADD;
	circleMarker.lifetime = ros::Duration(0.2);
	circleMarker.color.r = 1.0;
	circleMarker.color.g = 0.0;
	circleMarker.color.b = 0.0;
	circleMarker.color.a = 0.8;
	circleMarker.scale.x = radius * 2;
	circleMarker.scale.y = radius * 2;
	circleMarker.scale.z = 0.004;
	circleMarker.id = markerID;
	circleMarker.pose.position.x = position[0];
	circleMarker.pose.position.y = position[1];
	circleMarker.pose.position.z = position[2];
	return circleMarker;
}//Object_tracker::rosCircleMarker

visualization_msgs::Marker Object_tracker::rosArrowMarker( int markerID, std::string frameID, Vector arrowBase, Vector arrowTip, double lifetimeInSec ) {

	visualization_msgs::Marker arrowMarker;
	arrowMarker.header.frame_id = frameID;
	arrowMarker.header.stamp = ros::Time().now();
	arrowMarker.ns = "conveyor_object_tracker";
	arrowMarker.type = visualization_msgs::Marker::ARROW;
	arrowMarker.action = visualization_msgs::Marker::ADD;
	arrowMarker.lifetime = ros::Duration(lifetimeInSec);
	arrowMarker.color.r = 1.0;
	arrowMarker.color.g = 0.0;
	arrowMarker.color.b = 1.0;
	arrowMarker.color.a = 0.8;
	arrowMarker.scale.x = 0.01;
	arrowMarker.scale.y = 0.01;
	arrowMarker.scale.z = 0.005;
	arrowMarker.id = markerID;
	arrowMarker.points.resize( 2 );
	arrowMarker.points[0].x = arrowBase[0];
	arrowMarker.points[0].y = arrowBase[1];
	arrowMarker.points[0].z = arrowBase[2];
	arrowMarker.points[1].x = arrowTip[0];
	arrowMarker.points[1].y = arrowTip[1];
	arrowMarker.points[1].z = arrowTip[2];
	return arrowMarker;
}//Object_tracker::rosArrowMarker

visualization_msgs::Marker Object_tracker::rosLineStripMarker( int markerID, std::string frameID, std::vector<Vector> lineStripPoints, Vector color, double lifetimeInSec ) {

	visualization_msgs::Marker lineStripMarker;
	lineStripMarker.header.frame_id = frameID;
	lineStripMarker.header.stamp = ros::Time().now();
	lineStripMarker.ns = "conveyor_object_tracker";
	lineStripMarker.type = visualization_msgs::Marker::LINE_STRIP;
	lineStripMarker.action = visualization_msgs::Marker::ADD;
	lineStripMarker.lifetime = ros::Duration(lifetimeInSec);
	lineStripMarker.color.r = color[0];
	lineStripMarker.color.g = color[1];
	lineStripMarker.color.b = color[2];
	lineStripMarker.color.a = 0.8;
	lineStripMarker.scale.x = 0.001;
	lineStripMarker.scale.y = 0.001;
	lineStripMarker.scale.z = 0.001;
	lineStripMarker.id = markerID;
	lineStripMarker.points.resize( lineStripPoints.size() );
	for (size_t i = 0; i < lineStripPoints.size(); ++i ) {
		lineStripMarker.points[i].x = lineStripPoints.at(i)[0];
		lineStripMarker.points[i].y = lineStripPoints.at(i)[1];
		lineStripMarker.points[i].z = lineStripPoints.at(i)[2];
	}
	return lineStripMarker;
}//Object_tracker::rosLineStripMarker

visualization_msgs::Marker Object_tracker::rosTriangleStripMarker( int markerID, std::string frameID, std::vector<Vector> trianglePoints, double lifetimeInSec ) {

	visualization_msgs::Marker triStripMarker;
	triStripMarker.header.frame_id = frameID;
	triStripMarker.header.stamp = ros::Time().now();
	triStripMarker.ns = "conveyor_object_tracker";
	triStripMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	triStripMarker.action = visualization_msgs::Marker::ADD;
	triStripMarker.lifetime = ros::Duration(lifetimeInSec);
	triStripMarker.color.r = 0.0;
	triStripMarker.color.g = 0.0;
	triStripMarker.color.b = 1.0;
	triStripMarker.color.a = 1.0;
	triStripMarker.scale.x = 1.0;
	triStripMarker.scale.y = 1.0;
	triStripMarker.scale.z = 1.0;
	triStripMarker.id = markerID;
	triStripMarker.points.resize( trianglePoints.size() );
	for (int i = 0; i < trianglePoints.size(); i++) {
		triStripMarker.points[i].x = trianglePoints.at(i)[0];
		triStripMarker.points[i].y = trianglePoints.at(i)[1];
		triStripMarker.points[i].z = trianglePoints.at(i)[2];
	}
	return triStripMarker;
}//Object_tracker::rosLineStripMarker

visualization_msgs::Marker Object_tracker::rosTextMarker( int markerID, std::string frameID, Vector position, std::string text , double lifetimeInSec) {

	visualization_msgs::Marker textMarker;
	textMarker.header.frame_id = frameID;
	textMarker.header.stamp = ros::Time().now();
	textMarker.ns = "conveyor_object_tracker";
	textMarker.id = markerID;
	textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	textMarker.action = visualization_msgs::Marker::ADD;
	textMarker.lifetime = ros::Duration(lifetimeInSec);
	textMarker.text = text;
	textMarker.color.r = 1.0;
	textMarker.color.g = 0.0;
	textMarker.color.b = 0.0;
	textMarker.color.a = 0.9;
	textMarker.scale.z = 0.005;
	textMarker.pose.position.x = position[0];
	textMarker.pose.position.y = position[1];
	textMarker.pose.position.z = position[2];
	return textMarker;
}//Object_tracker::rosTextMarker

//-------------------general helper functions--------------
bool Object_tracker::point_z_compare ( const PointXYZ& a, const PointXYZ& b ) {
	return( a.z > b.z );
}//Object_tracker::point_z_compare

bool Object_tracker::point_z_hdist_compare( const PointXYZ& a, const PointXYZ& b ) {
	double hdistA = sqrt( pow(a.x, 2) + pow(a.y, 2) );
	double hdistB = sqrt( pow(b.x, 2) + pow(b.y, 2) );
	
	return( (a.z > b.z) || ( (a.z == b.z) && (hdistA < hdistB) ) );
}

bool Object_tracker::point_z_compareRGB ( const Point& a, const Point& b ) {
	return( a.z > b.z );
}//Object_tracker::point_z_compareRGB


}//namespace Conveyor_object_tracking