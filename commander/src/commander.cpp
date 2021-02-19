#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <utility>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <list>
#include <cmath>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>


const float TAKEOFF_ALTITUDE = 1.0;

enum State {
  TAKEOFF,
  EXPLORE,
  LAND
} state;


class Commander {
  private:
    State state;
    geometry_msgs::PoseStamped current_pose, goal_pose;
    visualization_msgs::Marker goals;

    mavros_msgs::State px4_state;
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::SetMode land_set_mode;
    mavros_msgs::CommandBool arm_cmd;


    ros::NodeHandle nh;
    ros::Rate rate;
    ros::Subscriber state_sub;
    ros::Publisher goal_pose_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::Subscriber goals_sub;
    ros::Subscriber pose_sub;
    ros::Time start_time;
    ros::Time last_request;

    void pose_cb (const geometry_msgs::PoseStamped::ConstPtr& msg) {
      current_pose = *msg;
    }
    void goals_cb (const visualization_msgs::Marker::ConstPtr& msg) {
      goals = *msg;
    }
    void state_cb (const mavros_msgs::State::ConstPtr& msg) {
      px4_state = *msg;
    }

  public:
    void connect_mavros () {
      while (ros::ok() && !px4_state.connected) {
        ros::spinOnce();
        rate.sleep();
      }
    }

    Commander() : rate(20.0){
      state_sub = nh.subscribe<mavros_msgs::State> ("mavros/state", 10, &Commander::state_cb, this);
      goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped> ("mavros/setpoint_position/local", 10);
      arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming");
      set_mode_client = nh.serviceClient<mavros_msgs::SetMode> ("mavros/set_mode");
      goals_sub = nh.subscribe("goal_gen/goals", 1, &Commander::goals_cb, this);
      pose_sub = nh.subscribe("mavros/local_position/pose", 1, &Commander::pose_cb, this);

      offb_set_mode.request.custom_mode = "OFFBOARD";
      land_set_mode.request.custom_mode = "LAND";
      arm_cmd.request.value = true;

      goal_pose.pose.position.x = 0;
      goal_pose.pose.position.y = 0;
      goal_pose.pose.position.z = TAKEOFF_ALTITUDE;
      start_time = ros::Time::now();
      connect_mavros();
    }



    void explore() {

    }

    void land() {
    }

    void takeoff() {
      static int i = 10;
      if (i > 0) {
        i--;
      } else if (px4_state.mode != "OFFBOARD") {
        if (ros::Time::now() - last_request > ros::Duration(1.0)) {
          ROS_INFO("Waiting to switch to Offboard");
          set_mode_client.call(offb_set_mode); 
          last_request = ros::Time::now();
        }
      } else if (!px4_state.armed) {
        if (ros::Time::now() - last_request > ros::Duration(1.0)) {
          ROS_INFO("Waiting to arm");
          arming_client.call(arm_cmd);
          last_request = ros::Time::now();
        }
      } else if (current_pose.pose.position.z > TAKEOFF_ALTITUDE - 0.1) {
        ROS_INFO("Waiting to rise");
        state = EXPLORE;
      }
    }

    void run() {
      ROS_INFO("here");
      state = TAKEOFF;
      while(ros::ok()) {
        if (state == TAKEOFF) {
          takeoff();
        } else if (state == EXPLORE) {
          explore();
        } else if (state == LAND) {
          land();
        }
        goal_pose_pub.publish(goal_pose);
        ros::spinOnce();
        rate.sleep();
      } 
    }
};


int main (int argc, char** argv)
{
  ros::init (argc, argv, "commander");
  Commander commander;
  commander.run();
}
