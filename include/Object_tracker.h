#ifndef CONVEYOR_OBJECT_TRACKING__OBJECT_TRACKER_H
#define CONVEYOR_OBJECT_TRACKING__OBJECT_TRACKER_H

#include <sys/time.h>

#include <queue>
#include <functional>

// ros includes
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

//pcl includes for object parameter retrieval
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_ros/point_cloud.h>

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <thread>
#include <pthread.h>
#include <future>
#include <mutex>

#include <algorithm>    // std::sort
#include <vector>       // std::vector
#include <X11/Xlib.h>

#include <industrial_kuka_ros/FollowObjectAction.h>
#include <industrial_kuka_ros/SetMode.h>
#include "Tracked_object.h"
#include "Conveyor_trajectory.h"

typedef actionlib::SimpleActionClient<industrial_kuka_ros::FollowObjectAction> Client;

using namespace std;


namespace Conveyor_object_tracking {

class Object_tracker {
public:
    typedef pcl::PointXYZRGB Point;
    typedef pcl::PointXYZ PointXYZ;
    typedef Eigen::Vector3f Vector;
    typedef Eigen::Vector4f VectorRot;//for conveyor trajectory, 3-dimensional position vector + rotationAxis rotation in degree
    typedef std::vector<int> Indices;

    typedef typename pcl::PointCloud<Point> Point_cloud;
    typedef typename pcl::PointCloud<PointXYZ> Point_cloud_XYZ;
    typedef typename Point_cloud::Ptr Point_cloud_ptr;
    typedef typename Point_cloud_XYZ::Ptr Point_cloud_XYZ_ptr;
    typedef const typename Point_cloud::ConstPtr Point_cloud_cptr;
    
    static const int unobserved_frames_threshold = 3;

    struct sacResult{
    	Vector center; std::vector<double> radii; double height; double weight; int index; bool valid;
    	sacResult() {
			center = Vector(0,0,0); radii.resize(1, 0.0); height = 0.0;
			weight = 0; index = -1; valid = false;
		}
    	sacResult(Vector c, std::vector<double> r, double h, double w, int i, bool v) {
    		center = c; radii = r; height = h; weight = w; index = i; valid = v;
    	}
    };
    struct sacSegment{
    	Point_cloud_XYZ_ptr cloud;
    	double height;
    	bool valid;
    	sacSegment() {cloud = nullptr; height = 0; valid = false;};
    	sacSegment( Point_cloud_XYZ_ptr c, double h, bool v ) {
    		cloud = c; height = h; valid =v;
    	}
    };

    Object_tracker ();
    virtual ~Object_tracker ();

    
//FUNCTIONS
    
    void attach_to_node ( ros::NodeHandle& );
    void points_callback ( const Point_cloud& );
    
  //Object reconstruction functions
    static bool removeUnobserved( Tracked_object to );
    
  //Transformation and format conversion functions
    Vector getKinectCoordinates( Vector convCoords ); //returns the coordinates transformed from conveyorFrame to kinect2Frame
    Vector tfObjectToConveyorCoordinates ( Vector objCoords ); //returns the conveyorCoordinates matching the objectCoordinates
    Vector tfConveyorToObjectCoordinates( Vector convCoords ); //transforms the Point in conveyorCoordinates to the correct point in objectCoordinate system
    bool sendObjectTransformation( Vector objectMiddle , int id ); //calculates the transformation into ObjectFrame, returns whether calculation was successful
    Point_cloud transformConveyorToObjectFrame( Point_cloud inputCloud, bool inverse ); //transforms the cloud from conveyorFrame to ObjectFrame and back (via inverse=true)
        
  //Robot communication functions
    
  //Visualization helper functions
    
  //general helper functions 
    static bool point_z_compare( const PointXYZ& a, const PointXYZ& b );
    static bool point_z_compareRGB( const Point& a, const Point& b );
    static bool point_z_hdist_compare( const PointXYZ& a, const PointXYZ& b );
    

    
    
private:
    /* Matching class represents matching distance between an observed and a tracked object, used for priority_queue */
    class Matching {
    public:
        Matching () {} // default operator is required for container initialization
        Matching ( const int tid, const int oid, const double v )
            : tracked_id(tid), observed_id(oid), value(v) {}

        int tracked_id;
        int observed_id;
        double value;

        // these operators are required for stl sorting functions, in our case the priority queue
        inline const bool operator< (const Matching& other) const { return this->value < other.value; }
        inline const bool operator> (const Matching& other) const { return this->value > other.value; }
    };// class Matching
    
    //ROS Node Handle
    ros::NodeHandle nh;
    
    //clustering
    int clustersize_min;
    int clustersize_max;
    double cluster_min_distance;
    double cluster_min_height;
    
    double object_conveyor_offset;
    int handleMinsize;
    double object_minheight;
    double object_maxwidth;
    
    
    
    int reconstruct_res;
    string wrlPath;
    

    //ROS communication
    ros::Subscriber points_subscriber;
    ros::Publisher reconstructed_object_marker_publisher;
    ros::Publisher tracked_object_publisher;
    ros::Publisher cloud_publisher;
    ros::Publisher filtered_object_symm_front_publisher;
    ros::Publisher filtered_object_symm_back_publisher;
    ros::Publisher filtered_object_nonsymm_publisher;
    ros::Publisher reconstructed_object_publisher;
    ros::Publisher prediction_publisher;
    ros::Publisher scenePart_publisher;
    ros::Publisher ransacObjectVisualization_publisher;
    ros::Publisher reconstrucedObjectVisualization_publisher;
    ros::Publisher conveyorTrajectoryVisualization_publisher;
    ros::Publisher robotTrajectoryVisualization_publisher;
    ros::Publisher robotTrajectoryGoal_publisher;
    
    //exports
    ros::Publisher export_speed_publisher;
    ros::Publisher export_object_param_publisher;
   
    
    //communication to/from robot
    bool simulated;
    double robotDelay;
    double robotTactTime;
    
    std_msgs::Float64MultiArray initialRobotPose;
    Client robotTrajectoryActionClient;
    ros::Publisher gripperJointPosition_publisher;
    ros::Publisher gripperEffort_publisher;
    ros::Publisher gripperPose_publisher;
    
    bool followObjectMode;
    
    //visualization
    visualization_msgs::MarkerArray reconstructed_object_markers;
    visualization_msgs::MarkerArray tracked_object_markers;
    visualization_msgs::MarkerArray trajectory_marker_msg;

    pcl::search::Search<Point>::Ptr search_tree;
    pcl::EuclideanClusterExtraction<Point>* cluster_extractor;
    pcl::SACSegmentation<PointXYZ>* sac;

    std::vector<Tracked_object> tracked_objects;
    std::vector<Observed_object> observed_objects;

    double interval;
    double conveyorVelocity; //estimated velocity of the conveyor (the conveyor is set to a constant speed)
    Conveyor_trajectory conveyorTrajectory; //conveyor trajectory reconstruction class

    bool useObjectFrame; //trigger to calculate center and radius based on objectFrame or ConveyorFrame
    int objectFrameCalcCtr; //how many times the center was calculated in objectFrame

    //object cloud and reconstruction
    Point_cloud constructedObject;
    Vector _last_construction_position;
    double object_reconstruction_angle;
    
    bool objectWRLCreated;
    bool pauseTracking;


    tf2_ros::Buffer tfBuf;
    boost::shared_ptr<tf2_ros::TransformListener> tfl;
    geometry_msgs::TransformStamped conveyor_to_kinect2_TF;
    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped kinect2_to_robot_TF;
    geometry_msgs::TransformStamped kinect2_to_object_TF;
    
    double circle_segment_height;
    size_t circle_segment_min_points;
    double matching_distance_threshold;
    double voxel_grid_size;
    Point_cloud_XYZ_ptr pattern_cloud;
    std::vector<PointXYZ> sorted_points_pattern;
    int num_parts_pattern;
     double icp_slice_height;
    int pattern_obj;
    Vector pattern_prediction;
    
    //conveyor trajectory reconstruction
    bool recordConveyorTrajectory; 

    //speed, threading and optimization
    bool multiObjectTracking;
    int counter;
    double start;
    double end;
    std::vector<std::pair<Eigen::Matrix4f, double> > threadResults;
    bool multiThreaded;
    std::mutex mtx;
    int framesViewed;
    ros::Time fEnd;
    ros::Duration totalTime;
    int objectDetectedCounter;//for the first three times when you see the object make clustering of the whole scene
     int historyLength;
    Vector predictedPos;
    std::list<Vector> history;

    // disallow copy and assign
    Object_tracker( const Object_tracker& );
    void operator= ( const Object_tracker& );
    
  //robot communication  
    string robot_baselink;
    Vector error = Vector(0,0,0); //error vector that is returned when object is out of bounds
    double interceptionTime;
    bool robotActive;
    double verticalApproachDistance;
    //gripper variables and commands
    double gripperVerticalTCPStandoff;
    double fixedTCPRotation;
    ros::Duration gripperApproachTime;
    ros::Duration grippingTime;
    bool gripperActive;
    std_msgs::Float64MultiArray openPosition120;
    std_msgs::Float64 gripEffort;
 
 
    
//FUNCTIONS    
  //Object reconstruction functions
    Point_cloud_ptr calculate_object_parameters( Conveyor_object_tracking::Object_tracker::Point_cloud_ptr object );
    vector<double> calculate_object_rim( Point_cloud_XYZ_ptr topCapCloud );
    std::vector<Vector> getHandleHullpoints( Point_cloud_XYZ_ptr handleCloud );
    Point_cloud_ptr getClusteredScenePart( Point_cloud_ptr scene_points,double x,double z );
    Point_cloud_ptr getDenseCloud( Point_cloud_ptr objectCloud );
    void threadSAC( Point_cloud_XYZ_ptr segment, double segment_ceiling, sacResult res[], int i );
    Conveyor_object_tracking::Observed_object find_object_position( Point_cloud_ptr object, std::vector<sacSegment> segmentedObject );
    bool mirror_match_point(Point left, Point right);
    std::vector< Indices > mirror_match_clouds( Point_cloud_ptr targetCloud, Point_cloud_ptr mirrorCloud );
    std::vector< Point_cloud_ptr > separate_object_clouds( Point_cloud_ptr object, double topRadius );
    Point_cloud filter_object( Point_cloud_ptr object );
    void update_object_position_object_frame( Point_cloud_ptr object );
    void match_objects();
    Matching calculate_object_distance ( const int, const int ) const;
    void reconstruct_by_angle ( const int, const int);
    Point_cloud cut_object( Point_cloud_ptr object, double angle ); //cuts out a part of the object, defined by the cutting angle
    std::vector<sacSegment> getSegmentedObject( Point_cloud_ptr objectCloud );
    sacResult calculateSolidOfRevolution( std::vector<sacSegment> segmentedObject, bool singleFrame );
    void recreateTrackedObject( Tracked_object to );
    void objectToWRL( Vector center, Point_cloud outwardCloud, Point_cloud inwardCloud, std::vector<Vector> handleHull );

  //Transformation and format conversion functions    
    std_msgs::Float64MultiArray transformConveyorToRobotCoordinates( VectorRot convCoords, double gripperRotation ); //returns the pose transformed from conveyorFrame to kukaBaseFrame as message
    Eigen::Vector3f transformConveyorToRobotFrame ( Vector conveyorCoordinates );
    //help functions to use TransformStamped messages with eigen
    Eigen::Vector3f tfTranslationToVector( geometry_msgs::TransformStamped transformation );
    Eigen::Quaternionf tfRotationToQuaternion( geometry_msgs::TransformStamped transformation );
    Eigen::Vector3f convertPoseToTranslation( VectorRot vecRot );
     
  //Robot communication functions
    bool generateTrajectory( Vector currentPosition, double objectHeight, Vector handleDirection, double interceptionTime ); //if trajectory could be generated
    void robot_trajectory_done( const actionlib::SimpleClientGoalState& state, const industrial_kuka_ros::FollowObjectResultConstPtr& result );
    void robot_trajectory_feedback ( const industrial_kuka_ros::FollowObjectFeedbackConstPtr& followObjectFeedback );
    
  //Visualization functions
    visualization_msgs::Marker rosPositionMarker( int markerID, std::string frameID, Vector position, double lifetimeInSec );
    visualization_msgs::Marker rosCircleMarker( int markerID, std::string frameID, Vector position, double radius );
    visualization_msgs::Marker rosArrowMarker( int markerID, std::string frameID, Vector arrowBase, Vector arrowTip, double lifetimeInSec );
    visualization_msgs::Marker rosLineStripMarker( int markerID, std::string frameID, std::vector<Vector> lineStripPoints, Vector color, double lifetimeInSec );
    visualization_msgs::Marker rosTriangleStripMarker( int markerID, std::string frameID, std::vector<Vector> trianglePoints, double lifetimeInSec );
    visualization_msgs::Marker rosTextMarker( int markerID, std::string frameID, Vector position, std::string text , double lifetimeInSec );
    visualization_msgs::Marker revolutionMeshMarker(  Vector center, Point_cloud outwardCloud, Point_cloud inwardCloud, std::vector<Vector> handleHull );
    

};//class Object_tracker

}//namespace Conveyor_object_tracking

#endif//CONVEYOR_OBJECT_TRACKING__OBJECT_TRACKER_H
