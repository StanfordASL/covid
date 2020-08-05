/* teleop.cpp
 * ijkl keys for horizontal movement
 * ed keys for vertical movement
 * ad keys for yaw movement
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>

#include <curses.h>
#include <map>

// Map for movement keys [x, y, z, theta]
std::map<char, std::vector<float>> moveBindings
{
    {'i', {1, 0, 0, 0}},
    {'j', {0, 1, 0, 0}},
    {'k', {-1, 0, 0, 0}},
    {'l', {0, -1, 0, 0}},
    {'e', {0, 0, 1, 0}},
    {'s', {0, 0, 0, 1}},
    {'d', {0, 0, -1, 0}},
    {'f', {0, 0, 0, -1}}
};

int main(int argc, char **argv)
{
    initscr();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);

    ros::init(argc, argv, "teleop_node");
    ros::NodeHandle nh; 
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
    ros::Rate rate(30);
    geometry_msgs::PoseStamped pose_cmd; 
    pose_cmd.pose.position.z = 1;
    tf2::Quaternion quaternion_cmd;
    quaternion_cmd.setRPY(0, 0, 0);

    char key(' ');
    const float LINEAR_SPEED(0.3);
    const float ROTATIONAL_SPEED(0.3);
    float d_x(0), d_y(0), d_z(1), d_th(0), th(0);

    while (ros::ok()) {
        key = getch();
        if (key == 'q') {
            keypad(stdscr, FALSE);
            nodelay(stdscr, FALSE);
            endwin();
            break;
        } else if (moveBindings.count(key) == 1) {
            d_x = cos(th) * moveBindings[key][0] - sin(th) * moveBindings[key][1];
            d_y = cos(th) * moveBindings[key][1] + sin(th) * moveBindings[key][0];
            d_z = moveBindings[key][2];
            d_th = moveBindings[key][3];
        } else {
            d_x = 0;
            d_y = 0;
            d_z = 0;
            d_th = 0;
        }

        // Update the Twist message
        pose_cmd.header.stamp = ros::Time::now(); 
        pose_cmd.header.frame_id = "map_ned";
        pose_cmd.pose.position.x += d_x * LINEAR_SPEED;
        pose_cmd.pose.position.y += d_y * LINEAR_SPEED;
        pose_cmd.pose.position.z += d_z * LINEAR_SPEED;
        th += d_th * ROTATIONAL_SPEED;


        quaternion_cmd.setRPY(0, 0, th);        
        quaternion_cmd.normalize();
        tf2::convert(quaternion_cmd, pose_cmd.pose.orientation);

        pub.publish(pose_cmd);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
