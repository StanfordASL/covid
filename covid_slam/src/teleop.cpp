#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


#include <curses.h>
#include <map>

// Map for movement keys [x, y, z, theta]
std::map<char, std::vector<float>> moveBindings
{
    {'i', {1, 0, 0, 0}},
    {'j', {0, 1, 0, 0}},
    {'k', {-1, 0, 0, 0}},
    {'l', {0, -1, 0, 0}},
    {'w', {0, 0, 1, 0}},
    {'a', {0, 0, 0, 1}},
    {'s', {0, 0, -1, 0}},
    {'d', {0, 0, 0, -1}}
};

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
}


/*
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 0;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

int getch(void){
    if (kbhit()) {
        ch = getchar();
    } else {
        ch = " ";
    }
    return ch;
}

*/

int main(int argc, char **argv)
{
    initscr();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);

    ros::init(argc, argv, "teleop_node");
    ros::NodeHandle nh; 
    ros::Publisher pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);
    ros::Rate rate(30);
    geometry_msgs::TwistStamped twist_cmd; 

    char key(' ');
    const float LINEAR_SPEED(1);
    const float ROTATIONAL_SPEED(1);
    float x(0), y(0), z(0), th(0);
    while (ros::ok()) {
        key = getch();

        if (key == 'q') {
            keypad(stdscr, FALSE);
            nodelay(stdscr, FALSE);
            endwin();
            break;
        } else if (moveBindings.count(key) == 1) {
            x = moveBindings[key][0];
            y = moveBindings[key][1];
            z = moveBindings[key][2];
            th = moveBindings[key][3];
        } else {
            x = 0;
            y = 0;
            z = 0;
            th = 0;
        }

        // Update the Twist message
        twist_cmd.header.stamp = ros::Time::now(); 
        twist_cmd.header.frame_id = "map_ned";
        twist_cmd.twist.linear.x = x * LINEAR_SPEED;
        twist_cmd.twist.linear.y = y * LINEAR_SPEED;
        twist_cmd.twist.linear.z = z * LINEAR_SPEED;
        twist_cmd.twist.angular.x = 0;
        twist_cmd.twist.angular.y = 0;
        twist_cmd.twist.angular.z = th * ROTATIONAL_SPEED;

        pub.publish(twist_cmd);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
