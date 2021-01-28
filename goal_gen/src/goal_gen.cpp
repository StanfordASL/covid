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
ros::Publisher pub_door;
ros::Publisher pub_door_centroid;
ros::Publisher pub_handle;
ros::Publisher pub_handle_centroid;
ros::Publisher pub_handle_centroid_list;

// Initialize transformer
tf2_ros::Buffer* tfBuffer;
tf2_ros::TransformListener* tfListener;


void generate_sub_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud, const darknet_ros_msgs::BoundingBox& box) {

    int width = input_cloud->width;
    int xmin = box.xmin;
    int xmax = box.xmax;
    int ymin = box.ymin;
    int ymax = box.ymax;

    // Extract the relevant handle points
    std::vector<int> indices;
    for (int column = xmin; column <= xmax; column++){
        for (int row = ymin; row <= ymax; row++){
            indices.push_back(row * width + column);
        }
    }
    pcl::PointIndices::Ptr roi(new pcl::PointIndices ());
    roi->indices = indices;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_roi;
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
    geometry_msgs::TransformStamped transformStamped = tfBuffer->lookupTransform(input_cloud->header.frame_id, "map", input_cloud->header.stamp);

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
    door_centroid_list.header.frame_id = handle_centroid_list.header.frame_id = "map";
    door_centroid_list.type = handle_centroid_list.type = 7;
    door_centroid_list.color.a = handle_centroid_list.color.a = 1.0;
    door_centroid_list.color.r = handle_centroid_list.color.r = 1.0;
    door_centroid_list.action  = handle_centroid_list.action = 0;
    door_centroid_list.scale.x = handle_centroid_list.scale.x = 0.5;
    door_centroid_list.scale.y = handle_centroid_list.scale.y = 0.5;
    door_centroid_list.scale.z = handle_centroid_list.scale.z = 0.5;

    // Convert input cloud to pcl
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input_cloud, *raw_cloud);

    for (int i = 0; i < full_doors.size(); i++) {
        int handle_idx = full_doors[i].first;
        int door_idx = full_doors[i].second;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr handle_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        generate_sub_cloud(raw_cloud, handle_cloud, input_detection->bounding_boxes[handle_idx]);

        // Compute the handle centroid
        pcl::PointXYZ handle_centroid;
        pcl::computeCentroid(*handle_cloud, handle_centroid); 

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr door_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        generate_sub_cloud(raw_cloud, door_cloud, input_detection->bounding_boxes[door_idx]);

        // ----------------------VoxelGrid----------------------------------
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr door_cloud_downsample (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> voxelgrid;
        voxelgrid.setInputCloud(door_cloud);
        voxelgrid.setLeafSize (0.01, 0.01, 0.01); //size of the grid
        voxelgrid.filter (*door_cloud_downsample);
        
       // ---------------------StatisticalOutlierRemoval--------------------
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr door_cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> noise;
        noise.setInputCloud (door_cloud_downsample);
        noise.setMeanK (50);
        noise.setStddevMulThresh (1.0);
        noise.filter (*door_cloud_filtered);
     
        // ---------------------RANSAC_PlanarSegmentation--------------------
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr door_cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        pcl::PointIndices::Ptr inlier_indices (new pcl::PointIndices ()); 
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);
        seg.setInputCloud (door_cloud_filtered);
        seg.segment (*inlier_indices, *coefficients);
        

        // Transform centroid to map frame
        geometry_msgs::PoseStamped p_old;
        p_old.pose.position.x = handle_centroid.x - coefficients->values[0];
        p_old.pose.position.y = handle_centroid.y - coefficients->values[1];
        p_old.pose.position.z = handle_centroid.z - coefficients->values[2];
        geometry_msgs::PoseStamped p_new;
        p_old.header.frame_id = input_cloud->header.frame_id;     
        tfBuffer->transform(p_old, p_new, "map", input_cloud->header.stamp, input_cloud->header.frame_id);


        handle_centroid_list.points.push_back(p_new.pose.position);
    }


    // Publish output msg
    pub_handle_centroid.publish (handle_centroid_list);
}

    /*

    
    // Unwrap darknet detection
    std::string object_name = input_detection->bounding_boxes[i].Class;
    int xmin                = input_detection->bounding_boxes[i].xmin;
    int xmax                = input_detection->bounding_boxes[i].xmax;
    int ymin                = input_detection->bounding_boxes[i].ymin;
    int ymax                = input_detection->bounding_boxes[i].ymax;




  // Initialize containers for clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud                  (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_door             (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_handle           (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Initialize container for centroids' markers
  visualization_msgs::Marker centroid_door_list, centroid_handle_list;
  centroid_door_list.header.frame_id = centroid_handle_list.header.frame_id = input_cloud->header.frame_id;
  centroid_door_list.type = centroid_handle_list.type = 7;
  centroid_door_list.color.a = centroid_handle_list.color.a = 1.0;
  centroid_door_list.color.r = centroid_handle_list.color.r = 1.0;
  centroid_door_list.action = centroid_handle_list.action = 0;
  centroid_door_list.scale.x = centroid_handle_list.scale.x = 0.05;
  centroid_door_list.scale.y = centroid_handle_list.scale.y = 0.05;
  centroid_door_list.scale.z = centroid_handle_list.scale.z = 0.05;

  // Initialize message with handle detections
  //pointcloud_processing_msgs::handle_position handle_list;

  // Initialize container for auxiliary clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_roi            (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_voxelgrid      (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_sor            (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_inplane        (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_outplane       (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Convert sensor_msgs::PointCloud2 to pcl::PointCloud
  pcl::fromROSMsg( *input_cloud, *cloud);

  // Get cloud width and height
  int width = input_cloud->width;
  int height = input_cloud->height;

  // Number of objects detected
  int num_boxes = input_detection->bounding_boxes.size();



  // For each object detected
  for(int i(0); i<num_boxes; i++){
    
    // Unwrap darknet detection
    std::string object_name = input_detection->bounding_boxes[i].Class;
    int xmin                = input_detection->bounding_boxes[i].xmin;
    int xmax                = input_detection->bounding_boxes[i].xmax;
    int ymin                = input_detection->bounding_boxes[i].ymin;
    int ymax                = input_detection->bounding_boxes[i].ymax;


    // -------------------ROI extraction------------------------------

    // Initialize container for inliers of the ROI
    pcl::PointIndices::Ptr inliers_roi (new pcl::PointIndices ());

    // Get inliers
    std::vector<int> indices;
    for (int column(xmin); column<=xmax; column++){
      for (int row(ymin); row<=ymax; row++){
        // Pixel coordinates to pointcloud index
        indices.push_back(row*width+column);
      }
    }

    inliers_roi->indices = indices;

    // Create the filtering ROI object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_roi;
    // Extract the inliers  of the ROI
    extract_roi.setInputCloud (cloud);
    extract_roi.setIndices (inliers_roi);
    extract_roi.setNegative (false);
    extract_roi.filter (*cloud_filtered_roi);
    
    
    // ----------------------VoxelGrid----------------------------------

    // Perform the downsampling
    pcl::VoxelGrid<pcl::PointXYZRGB> sor_voxelgrid;
    sor_voxelgrid.setInputCloud (cloud_filtered_roi);
    sor_voxelgrid.setLeafSize (0.01, 0.01, 0.01); //size of the grid
    sor_voxelgrid.filter (*cloud_filtered_voxelgrid);

    
   // ---------------------StatisticalOutlierRemoval--------------------

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_noise;
    // Remove noise
    sor_noise.setInputCloud (cloud_filtered_voxelgrid);
    sor_noise.setMeanK (50);
    sor_noise.setStddevMulThresh (1.0);
    sor_noise.filter (*cloud_filtered_sor);
 
 
    // ---------------------RANSAC_PlanarSegmentation--------------------
 
    // Initialize containers for plane coefficients and inliers of the plane
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices ());
 
   // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
 
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered_sor);
    seg.segment (*inliers_plane, *coefficients);
 
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_plane;
 
    // Extract the inliers
    extract_plane.setInputCloud (cloud_filtered_sor);
    extract_plane.setIndices (inliers_plane);
    extract_plane.setNegative (false);
    extract_plane.filter (*cloud_filtered_inplane);
      
    // Extract the outliers
 
    extract_plane.setNegative (true);
    extract_plane.filter (*cloud_filtered_outplane);
 
 
    // ----------------------Compute centroid-----------------------------
 
    // Centroid of cloud without plane
    //Eigen::Vector4f centroid_out;
    pcl::PointXYZ centroid_out;
    pcl::computeCentroid(*cloud_filtered_outplane,centroid_out); 
 
    // Centroid of plane cloud
    pcl::PointXYZ centroid_in;
    pcl::computeCentroid(*cloud_filtered_inplane,centroid_in); 
 
    // ----------------Set attributes of centroid marker------------------
 
    geometry_msgs::Point centroid;
 
    if (object_name == "handle"){
      centroid.x = centroid_out.x;
      centroid.y = centroid_out.y;
      centroid.z = centroid_out.z;
    }
    else{
      centroid.x = centroid_in.x;
      centroid.y = centroid_in.y;
      centroid.z = centroid_in.z;
      std::cout << centroid.x << " " << centroid.y << " " << centroid.z << " " << std::endl;
    }
 
 
 
    // ---------------Store resultant cloud and centroid------------------
    if (object_name != "handle"){
      *cloud_door += *cloud_filtered_inplane;
      centroid_door_list.points.push_back(centroid);
    }
    else {
      *cloud_handle += *cloud_filtered_outplane;
      centroid_handle_list.points.push_back(centroid);
     // handle_list.handles.push_back(centroid);
    }

  }

  // Create a container for the result data.
  sensor_msgs::PointCloud2 output_door;
  sensor_msgs::PointCloud2 output_handle;

  // Convert pcl::PointCloud to sensor_msgs::PointCloud2
  pcl::toROSMsg(*cloud_door,output_door);
  pcl::toROSMsg(*cloud_handle,output_handle);

  // Set output frame as the input frame
  output_door.header.frame_id             = input_cloud->header.frame_id;
  output_handle.header.frame_id           = input_cloud->header.frame_id;

  // Publish the data.
  pub_door.publish (output_door);
  pub_handle.publish (output_handle);

  // Publish markers
  pub_door_centroid.publish (centroid_door_list);
  pub_handle_centroid.publish (centroid_handle_list);

  // Publish handle position
  //pub_handle_centroid_list.publish (handle_list);

}

*/


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pointcloud_processing");
  ros::NodeHandle nh;

  // Initialize subscribers to darknet detection and pointcloud
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud(nh, "d435i/depth/color/points", 1);
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> sub_box(nh, "darknet_ros/bounding_boxes", 1);

  // Initialize transform listener
  tfBuffer = new tf2_ros::Buffer;
  tfListener = new tf2_ros::TransformListener(*tfBuffer);

  // Synchronize darknet detection with pointcloud
  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, darknet_ros_msgs::BoundingBoxes> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), sub_cloud, sub_box);
  sync.registerCallback(boost::bind(&cloud_cb, _1, _2));

  // Create a ROS publisher for the output point cloud
  pub_door = nh.advertise<sensor_msgs::PointCloud2> ("goal_gen/door", 1);
  pub_handle = nh.advertise<sensor_msgs::PointCloud2> ("goal_gen/handle", 1);

  // Create a ROS publisher for the output point cloud centroid markers
  pub_door_centroid = nh.advertise<visualization_msgs::Marker> ("goal_gen/door_centroid", 1);
  pub_handle_centroid = nh.advertise<visualization_msgs::Marker> ("goal_gen/handle_centroid", 1);

  // Create a ROS publisher for handle position
 // pub_handle_centroid_list = nh.advertise<pointcloud_processing_msgs::handle_position> ("detected_handles", 1);

  // Spin
  ros::spin ();
}
