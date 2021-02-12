#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <utility>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace message_filters;


// Initialize publishers

void goal_cb (const visualization_msgs::MarkerConstPtr& goals)
{


}




int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "commander");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("goal_gen/goals", 10, goal_cb);

  pub_door = nh.advertise<sensor_msgs::PointCloud2> ("goal_gen/door", 1);
  pub_handle = nh.advertise<sensor_msgs::PointCloud2> ("goal_gen/handle", 1);

  // Create a ROS publisher for the output point cloud centroid markers
  pub_door_centroid = nh.advertise<visualization_msgs::Marker> ("goal_gen/door_centroid", 1);
  pub_handle_centroid = nh.advertise<visualization_msgs::Marker> ("goal_gen/handle_centroid", 1);

  // Spin
  ros::spin ();
}
