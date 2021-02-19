#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <utility>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <list>
#include <cmath>


// Initialize publishers
ros::Publisher pub_targets;

float dist(geometry_msgs::Point& p1, geometry_msgs::Point& p2) {
    float x_diff = p1.x - p2.x;
    float y_diff = p1.y - p2.y;
    float z_diff = p1.z - p2.z;
    return sqrt(pow(x_diff, 2) + pow(y_diff, 2) + pow(z_diff, 2));
}

void goal_cb (const visualization_msgs::MarkerConstPtr& msg)
{
    for (geometry_msgs::Point goal : msg->points) {
        bool unique = true;
        for (geometry_msgs::Point& target: targets) {
            if (dist(goal, target) < DISTANCE_THRESHOLD) {
                target.x = 0.9 * target.x + 0.1 * goal.x;
                target.y = 0.9 * target.y + 0.1 * goal.y;
                target.z = 0.9 * target.z + 0.1 * goal.z;

                unique = false;
            }
        }
        if (unique) {
            targets.push_back(goal);
        }

    }

    visualization_msgs::Marker targets_msg;
    targets_msg.header.frame_id = msg->header.frame_id;
    targets_msg.type = 7;
    targets_msg.color.a = 1.0;
    targets_msg.color.g = 1.0;
    targets_msg.action = 0;
    targets_msg.scale.x = 0.1;
    targets_msg.scale.y = 0.1;
    targets_msg.scale.z = 0.1;
    for (geometry_msgs::Point target : targets) {
        targets_msg.points.push_back(target);
    }
    if (targets.size() != 0) {
        pub_targets.publish(targets_msg);
    }
}




int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "commander");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("goal_gen/goals", 10, goal_cb);
  pub_targets = nh.advertise<visualization_msgs::Marker> ("goal_gen/targets", 1);




  ros::spin ();
}
