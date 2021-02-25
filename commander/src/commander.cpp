#include <ros/ros.h>
#include <math.h>
//#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <algorithm>
#include <vector>
//#include <utility>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <list>
#include <cmath>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ActuatorControl.h>
#include <tf2/LinearMath/Quaternion.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const float TAKEOFF_ALTITUDE = 1.0;
const float YAW_PERIOD = 12.0;
const float DIST_THRESH = 0.15;
const float MAX_H_SPEED = 0.4;
const float MAX_V_SPEED = 0.4;
const float FINAL_GOAL_DIST = 4.0;
const float Z_OFFSET = 0.06;
const float X_OFFSET = -0.10;


enum State {
  GROUNDED,
  TAKEOFF,
  EXPLORE,
  SPRAY,
  LAND
};

const std::vector<char *> state_names{"GROUNDED", "TAKEOFF", "EXPLORE", "SPRY", "LAND"};

template <typename T> T clamp(const T& value, const T& low, const T& high) {
  return value < low ? low : (value > high ? high : value);
}

float dist(geometry_msgs::Pose a, geometry_msgs::Pose b) {
  float diffs[3];
  diffs[0] = abs(a.position.x - b.position.x);
  diffs[1] = abs(a.position.y - b.position.y);
  diffs[2] = abs(a.position.z - b.position.z);
  return sqrt(pow(diffs[0], 2) + pow(diffs[1], 2) + pow(diffs[2], 2));
}

class Commander {
  private:
    State state, prev_state;
    geometry_msgs::PoseStamped curr_pose, curr_goal, local_curr_goal, final_goal, curr_request, takeoff_goal;
    geometry_msgs::PoseArray goals, traj;
    int goal_index = 0;

    mavros_msgs::State px4_state;
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::SetMode land_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::CommandBool disarm_cmd;

    ros::NodeHandle nh;
    ros::Rate rate;
    ros::Subscriber state_sub;
    ros::Publisher curr_request_pub;
    ros::Publisher mavros_pose_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::Subscriber goals_sub;
    ros::Subscriber traj_sub;
    ros::Time last_request;
    ros::Time explore_start;
    ros::Time start_time;
    std::vector<ros::Time> spray_times;
    tf2_ros::Buffer* tfBuffer;
    tf2_ros::TransformListener* tfListener;


    void goals_cb (const geometry_msgs::PoseArray::ConstPtr& msg) {
      goals = *msg;
    }
    void state_cb (const mavros_msgs::State::ConstPtr& msg) {
      px4_state = *msg;
    }
    void traj_cb (const geometry_msgs::PoseArray::ConstPtr& msg) {
      traj = *msg;
    }
    
  public:
    void connect_mavros () {
      while (ros::ok() && !px4_state.connected) {
        ros::spinOnce();
        rate.sleep();
      }
    }

    Commander() : rate(20.0){
      tfBuffer = new tf2_ros::Buffer;
      tfListener = new tf2_ros::TransformListener(*tfBuffer);

      state_sub = nh.subscribe<mavros_msgs::State> ("mavros/state", 10, &Commander::state_cb, this);
      curr_request_pub = nh.advertise<geometry_msgs::PoseStamped> ("commander/curr_request", 10);
      mavros_pose_pub = nh.advertise<geometry_msgs::PoseStamped> ("mavros/setpoint_position/local", 10);
      arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming");
      set_mode_client = nh.serviceClient<mavros_msgs::SetMode> ("mavros/set_mode");
      goals_sub = nh.subscribe("goal_gen/goals", 1, &Commander::goals_cb, this);
      traj_sub = nh.subscribe("waypoints", 1, &Commander::traj_cb, this);

      offb_set_mode.request.custom_mode = "OFFBOARD";
      land_set_mode.request.custom_mode = "AUTO.LAND";
      arm_cmd.request.value = true;
      disarm_cmd.request.value = false;

      takeoff_goal.header.frame_id = "map";
      takeoff_goal.pose.position.x = 0 + X_OFFSET;
      takeoff_goal.pose.position.y = 0;
      takeoff_goal.pose.position.z = TAKEOFF_ALTITUDE + Z_OFFSET;

      final_goal.header.frame_id = "map";
      final_goal.pose.position.x = FINAL_GOAL_DIST + X_OFFSET;
      final_goal.pose.position.y = 0;
      final_goal.pose.position.z = TAKEOFF_ALTITUDE + Z_OFFSET;

      takeoff_goal.header.frame_id = "map";

      connect_mavros();

      curr_request = final_goal; 
      curr_request_pub.publish(curr_request);
      ROS_INFO("Publishing Request");

    }


    void explore() {
      if (traj.poses.size() > 1) {
        curr_goal.pose = traj.poses[1];
      } else if (traj.poses.size() == 1) {
        ROS_INFO("unexpected size 1 traj");
      } else if (traj.poses.size() == 0 && (ros::Time::now() - explore_start).toSec() > 2.0) {
        state = LAND;
        ROS_WARN("No path, landing");
      }

      float yaw = 1.3 * sin((ros::Time::now() - explore_start).toSec() * 2 * M_PI / YAW_PERIOD) ;
      tf2::Quaternion q = tf2::Quaternion(0, 0, yaw);  
      curr_goal.pose.orientation.x = q.x();
      curr_goal.pose.orientation.y = q.y();
      curr_goal.pose.orientation.z = q.z();
      curr_goal.pose.orientation.w = q.w();
      /* // modified
      if (goals.poses.size() > goal_index) {
        curr_request.pose = goals.poses[goal_index];
        curr_request_pub.publish(curr_request);
        state = SPRAY;
      }
      */
      if (traj.poses.size() == 0 && (ros::Time::now() - explore_start).toSec() > 2.0) {
        state = LAND;
        ROS_WARN("No path, landing");
      }

      if(dist(curr_pose.pose, final_goal.pose) < DIST_THRESH && ros::Time::now() - explore_start > ros::Duration(40.0)) { // modified
        state = LAND;
      }
    }

    void spray() {
      curr_goal.pose.orientation.x = goals.poses[goal_index].orientation.x;
      curr_goal.pose.orientation.y = goals.poses[goal_index].orientation.y;
      curr_goal.pose.orientation.z = goals.poses[goal_index].orientation.z;
      curr_goal.pose.orientation.w = goals.poses[goal_index].orientation.w;
      if (traj.poses.size() == 0 && (ros::Time::now() - explore_start).toSec() > 2.0) {
        state = LAND;
        ROS_WARN("No path, landing");
      } 
       
      if (dist(goals.poses[goal_index], curr_pose.pose) > DIST_THRESH && spray_times.size() == goal_index) {
        if (traj.poses.size() > 1) {
          curr_goal.pose.position = traj.poses[1].position;
        } else if (traj.poses.size() == 1) {
          ROS_INFO("unexpected size 1 traj");
        }
      } else {
        if (spray_times.size() == goal_index) {
          spray_times.push_back(ros::Time::now());
          if (goals.poses.size() > goal_index + 1) {
            curr_request.pose = goals.poses[goal_index + 1];
          } else {
            curr_request.pose = final_goal.pose;
          }
          curr_request_pub.publish(curr_request);
        }
        static float prev_t;
        float t = (ros::Time::now() - spray_times[goal_index]).toSec();
        if (t < 1.0) {
          //wait to start
        } else if (t < 5.0) {
          if (prev_t < 1.0 ) {
            ROS_INFO("Turning on sprayer");
            //turn on sprayer
          }
          //spraying
        } else { 
          if (prev_t < 5.0) {
            ROS_INFO("Turning off sprayer");
            //turn off sprayer
          }
          state = EXPLORE;
          goal_index++;
        }
        prev_t = t;
      } 
    }

    void land() {
      static ros::Time land_start = ros::Time::now();
      if (px4_state.mode == "OFFBOARD") {
        if (ros::Time::now() - last_request > ros::Duration(1.0)) {
          ROS_INFO("px4 - switching to AUTO.LAND");
          set_mode_client.call(land_set_mode); 
          last_request = ros::Time::now();
        }
      } else if (curr_pose.pose.position.z > DIST_THRESH + Z_OFFSET || (ros::Time::now() - land_start).toSec() < 5.0) {
      } else if (px4_state.armed) {
        if (ros::Time::now() - last_request > ros::Duration(1.0)) {
          ROS_INFO("px4 - disarming");
          arming_client.call(disarm_cmd);
          last_request = ros::Time::now();
        }
      } else {
        state = GROUNDED;
      }
    }

    void takeoff() {
      curr_goal = takeoff_goal;
      if (px4_state.mode != "OFFBOARD") {
        if (ros::Time::now() - last_request > ros::Duration(1.0)) {
          ROS_INFO("px4 - switching to OFFBOARD");
          set_mode_client.call(offb_set_mode); 
          last_request = ros::Time::now();
        }
      } else if (!px4_state.armed) {
        if (ros::Time::now() - last_request > ros::Duration(1.0)) {
          ROS_INFO("px4 - arming");
          arming_client.call(arm_cmd);
          last_request = ros::Time::now();
        }
      } else if (curr_pose.pose.position.z < TAKEOFF_ALTITUDE + Z_OFFSET - DIST_THRESH) {
      } else {
        state = EXPLORE;
        explore_start = ros::Time::now();
      }
    }

    void wait() {
      // HAHA do nothing buddy, for now...
    }

    void sim_pub_goal() {
      static ros::Time prev_time = ros::Time::now();
      ros::Time curr_time = ros::Time::now();
      static geometry_msgs::PoseStamped prev_goal = local_curr_goal;

      float diffs[3];
      diffs[0] = abs(local_curr_goal.pose.position.x - prev_goal.pose.position.x);
      diffs[1] = abs(local_curr_goal.pose.position.y - prev_goal.pose.position.y);
      diffs[2] = abs(local_curr_goal.pose.position.z - prev_goal.pose.position.z);
      float requested_dist = sqrt(pow(diffs[0], 2) + pow(diffs[1], 2)); 
      float delta_t = (ros::Time::now() - prev_time).toSec();

      geometry_msgs::PoseStamped incremental_pose = local_curr_goal;
      incremental_pose.pose.position.x = clamp(
          local_curr_goal.pose.position.x,
          prev_goal.pose.position.x - delta_t * MAX_H_SPEED * diffs[0] / requested_dist,
          prev_goal.pose.position.x + delta_t * MAX_H_SPEED * diffs[0] / requested_dist);
      incremental_pose.pose.position.y = clamp(
          local_curr_goal.pose.position.y,
          prev_goal.pose.position.y - delta_t * MAX_H_SPEED * diffs[1] / requested_dist,
          prev_goal.pose.position.y + delta_t * MAX_H_SPEED * diffs[1] / requested_dist);
      incremental_pose.pose.position.z = clamp(
          local_curr_goal.pose.position.z,
          prev_goal.pose.position.z - delta_t * MAX_V_SPEED,
          prev_goal.pose.position.z + delta_t * MAX_V_SPEED);

      prev_goal = incremental_pose;
      prev_time = curr_time;
      mavros_pose_pub.publish(incremental_pose);
    }
      

    void run() {
      prev_state = TAKEOFF;
      state = TAKEOFF;
      start_time = ros::Time::now();
      while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();

        if (!tfBuffer->canTransform("map", "base_link", ros::Time(0), 0)) { 
            ROS_WARN("Transform not yet available");
            continue;
        }  

    
        geometry_msgs::TransformStamped transform_stamped = tfBuffer->lookupTransform("map", "base_link", ros::Time(0), ros::Duration(0));
        curr_pose.pose.orientation = transform_stamped.transform.rotation;
        curr_pose.pose.position.x = transform_stamped.transform.translation.x;
        curr_pose.pose.position.y = transform_stamped.transform.translation.y;
        curr_pose.pose.position.z = transform_stamped.transform.translation.z;
        if (state == TAKEOFF) {
          takeoff();
        } else if (state == EXPLORE) {
          explore();
        } else if (state == SPRAY) {
          spray();
        } else if (state == LAND) {
          land();
        } else if (state == GROUNDED) {
          wait();
        }
        if (state != prev_state) {
          ROS_INFO("Switching to state: %s", state_names[state]);
        }

        tfBuffer->transform(curr_goal, local_curr_goal, "t265_odom_frame");

        mavros_pose_pub.publish(local_curr_goal);
        //sim_pub_goal();
        prev_state = state;
         
      } 
      ros::spinOnce();
      rate.sleep();
    } 
};


int main (int argc, char** argv)
{
    

    ros::init (argc, argv, "commander");
    Commander commander;
    commander.run();
    /*
    ros::NodeHandle nh;
    mavros_msgs::ActuatorControl spray_on;
    mavros_msgs::ActuatorControl spray_off;
    ros::Rate rate(10.0);
    ros::Publisher spray_pub = nh.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 1);
    spray_on.group_mix = 3;
    spray_on.controls[5] = 1;
    spray_off.group_mix = 3;
    spray_off.controls[5] = -1;
    ros::Time start_time = ros::Time::now();
    while (ros::ok()) {
      if (int((ros::Time::now() - start_time).toSec()) % 2 == 0){
        ROS_INFO("Sprayer On");
        spray_pub.publish(spray_on);
      } else {
        ROS_INFO("Sprayer Off");
        spray_pub.publish(spray_off);
      }
      ros::spinOnce();
      rate.sleep();
    }
    */
}
