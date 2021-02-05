#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
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
ros::Publisher pub_handles;
ros::Publisher pub_handle_cloud;

// Initialize transformer
tf2_ros::Buffer* tfBuffer;
tf2_ros::TransformListener* tfListener;


void generate_sub_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud, const darknet_ros_msgs::BoundingBox& box) {
    int width = 640;
    int xmin = box.xmin;
    int xmax = box.xmax;
    int ymin = box.ymin;
    int ymax = box.ymax;
    std::vector<int> indices;
    for (int column = xmin; column < xmax; column++){
        for (int row = ymin; row < ymax; row++){
            indices.push_back(row * width + column);
        }
    }
    pcl::PointIndices::Ptr roi(new pcl::PointIndices ());
    roi->indices = indices;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_roi;
    extract_roi.setInputCloud(input_cloud);
    extract_roi.setIndices(roi);
    extract_roi.setNegative(false);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    extract_roi.filter(*output_cloud);
    /*
    pcl::VoxelGrid<pcl::PointXYZRGB> voxelgrid;
    voxelgrid.setInputCloud(roi_cloud);
    voxelgrid.setLeafSize (0.01, 0.02, 0.02); //size of the grid
    voxelgrid.filter (*output_cloud);
    */
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input_cloud, const darknet_ros_msgs::BoundingBoxesConstPtr& input_detection)
{
    // Get Transform to Map
    if (!tfBuffer->canTransform(input_cloud->header.frame_id, "map", input_cloud->header.stamp)) { 
        ROS_WARN("Transform not yet available");
        return;
    }  
    geometry_msgs::TransformStamped transformStamped = tfBuffer->lookupTransform(input_cloud->header.frame_id, "map", input_cloud->header.stamp);

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

    // Set up output msgs
    visualization_msgs::Marker door_centroid_list, handle_centroid_list;
    door_centroid_list.header.frame_id = handle_centroid_list.header.frame_id = "d435i_depth_optical_frame";
    door_centroid_list.type = handle_centroid_list.type = 7;
    door_centroid_list.color.a = handle_centroid_list.color.a = 1.0;
    door_centroid_list.color.r = handle_centroid_list.color.r = 1.0;
    door_centroid_list.action  = handle_centroid_list.action = 0;
    door_centroid_list.scale.x = handle_centroid_list.scale.x = 0.1;
    door_centroid_list.scale.y = handle_centroid_list.scale.y = 0.1;
    door_centroid_list.scale.z = handle_centroid_list.scale.z = 0.1;

    // Convert input cloud to pcl
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); 
    pcl::fromROSMsg(*input_cloud, *raw_cloud);



    for (int i = 0; i < full_doors.size(); i++) {
        int door_idx = full_doors[i].first;
        int handle_idx = full_doors[i].second;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr handle_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        generate_sub_cloud(raw_cloud, handle_cloud, input_detection->bounding_boxes[handle_idx]);

        sensor_msgs::PointCloud2 handle_cloud_ros;
        pcl::toROSMsg(*handle_cloud, handle_cloud_ros);

      // Set output frame as the input frame
        handle_cloud_ros.header.frame_id             = input_cloud->header.frame_id;
        pub_handle_cloud.publish(handle_cloud_ros);

        // Compute the handle centroid
        pcl::PointXYZRGB handle_centroid;
        pcl::computeCentroid(*handle_cloud, handle_centroid); 

        // Transform centroid to map frame (currently commented out)
        geometry_msgs::PoseStamped p_old;
        p_old.pose.position.x = handle_centroid.x; //- coefficients->values[0];
        p_old.pose.position.y = handle_centroid.y; //- coefficients->values[1];
        p_old.pose.position.z = handle_centroid.z; //- coefficients->values[2];
        //geometry_msgs::PoseStamped p_new;
        p_old.header.frame_id = input_cloud->header.frame_id;     
        //tfBuffer->transform(p_old, p_new, "map", input_cloud->header.stamp, input_cloud->header.frame_id);

        handle_centroid_list.points.push_back(p_old.pose.position);
    }
    pub_handles.publish(handle_centroid_list);
}



int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pointcloud_processing");
  ros::NodeHandle nh;

  // Initialize subscribers to darknet detection and pointcloud;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_image(nh, "d435i/depth/color/points", 1);
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> sub_box(nh, "darknet_ros/bounding_boxes", 1);

  // Initialize transform listener
  tfBuffer = new tf2_ros::Buffer;
  tfListener = new tf2_ros::TransformListener(*tfBuffer);

  // Synchronize darknet detection with pointcloud
  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, darknet_ros_msgs::BoundingBoxes> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(50), sub_image, sub_box);
  sync.registerCallback(boost::bind(&cloud_cb, _1, _2));

  // Create a ROS publisher for the output point cloud
  pub_handle_cloud = nh.advertise<sensor_msgs::PointCloud2> ("goal_gen/handle_cloud", 1);
  // Create a ROS publisher for the output handle points
  pub_handles = nh.advertise<visualization_msgs::Marker> ("goal_gen/handles", 1);


  // Spin
  ros::spin ();
}
