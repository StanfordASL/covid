#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <utility>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <pointcloud_processing_msgs/handle_position.h>

// Approximate time policy
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
using namespace message_filters;

// Darknet detection
#include <darknet_ros_msgs/BoundingBoxes.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>

// Transforms
#include <tf2_ros/transform_listener.h>

// Initialize publishers
ros::Publisher pub_goals;

// Initialize transformer
tf2_ros::Buffer* tfBuffer;
tf2_ros::TransformListener* tfListener;

const float DISTANCE_THRESHOLD = 0.75;
const float ANGLE_THRESHOLD = 1;
const float SPRAY_DISTANCE = 0.9;
const float GROUND_OFFSET = 0.44;
const float HANDLE_HEIGHT = .92;
const float HEIGHT_TOLERANCE = .25;
const float HEIGHT_OFFSET = .18;
geometry_msgs::PoseArray goals;

float dist(geometry_msgs::Point& p1, geometry_msgs::Point& p2) {
    float x_diff = p1.x - p2.x;
    float y_diff = p1.y - p2.y;
    float z_diff = p1.z - p2.z;
    return sqrt(pow(x_diff, 2) + pow(y_diff, 2) + pow(z_diff, 2));
}

void generate_sub_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, const darknet_ros_msgs::BoundingBox& box) {
    int width = 640;
    int xmin = box.xmin;
    int xmax = box.xmax;
    int ymin = box.ymin;
    int ymax = box.ymax;
    std::vector<int> indices;
    for (int column = xmin; column < xmax; column += 2){
        for (int row = ymin; row < ymax; row += 2){
            indices.push_back(row * width + column);
        }
    }
    pcl::PointIndices::Ptr roi(new pcl::PointIndices ());
    roi->indices = indices;
    pcl::ExtractIndices<pcl::PointXYZ> extract_roi;
    extract_roi.setInputCloud(input_cloud);
    extract_roi.setIndices(roi);
    extract_roi.setNegative(false);
    extract_roi.filter(*output_cloud);
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input_cloud, const darknet_ros_msgs::BoundingBoxesConstPtr& input_detection)
{
    // Get Transform to Map
    if (!tfBuffer->canTransform(input_cloud->header.frame_id, "map", input_cloud->header.stamp)) { 
        ROS_WARN("Transform not yet available");
        return;
    }  
    geometry_msgs::TransformStamped mapTransform = tfBuffer->lookupTransform(input_cloud->header.frame_id, "map", input_cloud->header.stamp);

    // Match door boxes with handle boxes
    std::vector<std::pair<int, int>> full_doors;
    int num_boxes = input_detection->bounding_boxes.size();
    for (int i = 0; i < num_boxes; i++) {
        std::string door_name = input_detection->bounding_boxes[i].Class;
        if (door_name == "handle") continue;
        int door_xmin = input_detection->bounding_boxes[i].xmin;
        int door_xmax = input_detection->bounding_boxes[i].xmax;
        int door_ymin = input_detection->bounding_boxes[i].ymin;
        int door_ymax = input_detection->bounding_boxes[i].ymax; 

        for (int j = 0; j < num_boxes; j++) {
            std::string handle_name = input_detection->bounding_boxes[j].Class;
            if (handle_name != "handle") continue;
            int handle_x = (input_detection->bounding_boxes[j].xmin + input_detection->bounding_boxes[j].xmax) / 2; 
            int handle_y = (input_detection->bounding_boxes[j].ymin + input_detection->bounding_boxes[j].ymax) / 2;
            // Check that handle is inside door
            if (    handle_x <= door_xmax &&
                    handle_x >= door_xmin &&
                    handle_y <= door_ymax &&
                    handle_y >= door_ymin) {
                std::pair<int, int> full_door(i, j);
                full_doors.push_back(full_door);
            }
        }
    }
    if (full_doors.size() == 0) return;

    geometry_msgs::PoseArray new_goals;
    new_goals.header.frame_id = "map";

    // Convert input cloud to pcl
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::fromROSMsg(*input_cloud, *raw_cloud);


    for (int i = 0; i < full_doors.size(); i++) {
        int door_idx = full_doors[i].first;
        int handle_idx = full_doors[i].second;

        pcl::PointCloud<pcl::PointXYZ>::Ptr handle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        generate_sub_cloud(raw_cloud, handle_cloud, input_detection->bounding_boxes[handle_idx]);
        pcl::PointXYZ handle_centroid;
        pcl::computeCentroid(*handle_cloud, handle_centroid); 


        pcl::PointCloud<pcl::PointXYZ>::Ptr door_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        generate_sub_cloud(raw_cloud, door_cloud, input_detection->bounding_boxes[door_idx]);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        pcl::PointIndices::Ptr inlier_indices (new pcl::PointIndices ()); 
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);
        seg.setInputCloud (door_cloud);
        seg.segment (*inlier_indices, *coefficients);
        // Transform centroid to map frame
        geometry_msgs::PoseStamped p_old_neg, p_old_pos;

  
        float sign = 1.0; 
        p_old_pos.pose.position.x = handle_centroid.x + SPRAY_DISTANCE * sign * coefficients->values[0];
        p_old_pos.pose.position.y = handle_centroid.y;// - coefficients->values[1];
        p_old_pos.pose.position.z = handle_centroid.z + SPRAY_DISTANCE * sign * coefficients->values[2];  
        float yaw = atan2(-sign * coefficients->values[2], sign * coefficients->values[0]);
        tf2::Quaternion q = tf2::Quaternion(yaw, 0, 0); // Axes shit, just roll with it
        p_old_pos.pose.orientation.x = q.x();
        p_old_pos.pose.orientation.y = q.y();
        p_old_pos.pose.orientation.z = q.z();
        p_old_pos.pose.orientation.w = q.w();
        sign = -1.0;
        p_old_neg.pose.position.x = handle_centroid.x + SPRAY_DISTANCE * sign * coefficients->values[0];
        p_old_neg.pose.position.y = handle_centroid.y;// - coefficients->values[1];
        p_old_neg.pose.position.z = handle_centroid.z + SPRAY_DISTANCE * sign * coefficients->values[2];  
        yaw = atan2(-sign * coefficients->values[2], sign * coefficients->values[0]);
        q = tf2::Quaternion(yaw, 0, 0); // Axes shit, just roll with it
        p_old_neg.pose.orientation.x = q.x();
        p_old_neg.pose.orientation.y = q.y();
        p_old_neg.pose.orientation.z = q.z();
        p_old_neg.pose.orientation.w = q.w();


        geometry_msgs::PoseStamped p_new_pos, p_new_neg, p_new;
        p_old_pos.header.frame_id = input_cloud->header.frame_id;     
        p_old_neg.header.frame_id = input_cloud->header.frame_id;     


        tfBuffer->transform(p_old_pos, p_new_pos, "map", input_cloud->header.stamp, input_cloud->header.frame_id); 
        tfBuffer->transform(p_old_neg, p_new_neg, "map", input_cloud->header.stamp, input_cloud->header.frame_id); 


        p_new = abs(p_new_pos.pose.position.y) < abs(p_new_neg.pose.position.y) ? p_new_pos : p_new_neg;

        if (abs(HANDLE_HEIGHT - p_new.pose.position.z - GROUND_OFFSET) < HEIGHT_TOLERANCE) {
            p_new.pose.position.z += HEIGHT_OFFSET;
            new_goals.poses.push_back(p_new.pose);
        }
    }

    //tf2::Quaternion identity(0, 0, 0);
    for (geometry_msgs::Pose& new_goal : new_goals.poses) {

        //tf2::Quaternion q = tf2::Quaternion(new_goal.orientation.x, new_goal.orientation.y, new_goal.orientation.z, new_goal.orientation.w);
        //float angle = identity.angleShortestPath(q);
        //std::cout << angle << std::endl;
        //if ( angle > 1.55 + .3 || angle < 1.55 - .3)  {
        //    continue; //bogus?
        //}

        bool unique = true;
        for (geometry_msgs::Pose& goal: goals.poses) {
            tf2::Quaternion q1(goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w);
            tf2::Quaternion q2(new_goal.orientation.x, new_goal.orientation.y, new_goal.orientation.z, new_goal.orientation.w);
            if (dist(goal.position, new_goal.position) < DISTANCE_THRESHOLD && q1.angleShortestPath(q2) < ANGLE_THRESHOLD) {
                goal = new_goal;
                unique = false;
            }
        }
        if (unique) {
            goals.poses.push_back(new_goal);
            ROS_INFO("Found a new handle");
        }
    }
    if (goals.poses.size() != 0) {
        pub_goals.publish(goals);
    }
}



int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pointcloud_processing");
  ros::NodeHandle nh;

  goals.header.frame_id = "map";

  // Initialize subscribers to darknet detection and pointcloud;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_image(nh, "d435i/aligned_depth_to_color/points", 1);
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> sub_box(nh, "darknet_ros/bounding_boxes", 1);

  // Initialize transform listener
  tfBuffer = new tf2_ros::Buffer;
  tfListener = new tf2_ros::TransformListener(*tfBuffer);

  // Synchronize darknet detection with pointcloud
  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, darknet_ros_msgs::BoundingBoxes> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(50), sub_image, sub_box);
  sync.registerCallback(boost::bind(&cloud_cb, _1, _2));

  // Create a ROS publisher for the output handle points
  pub_goals = nh.advertise<geometry_msgs::PoseArray> ("goal_gen/goals", 1);


  // Spin
  ros::spin ();
}
