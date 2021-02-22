#include <ros/ros.h>
#include <math.h>
//#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <algorithm>
//#include <utility>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <list>
#include <cmath>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <tf2/LinearMath/Quaternion.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>


const float TAKEOFF_ALTITUDE = 1.0;
const float YAW_PERIOD = 12.0;
const float DIST_THRESH = 0.1;
const float Z_OFFSET = 0;//0.06;
const float X_OFFSET = 0;//-0.10;
const float MAX_H_SPEED = 0.1;
const float MAX_V_SPEED = 0.4;


enum State {
  GROUNDED,
  TAKEOFF,
  EXPLORE,
  SPRAY,
  LAND
};

template <typename T> T clamp(const T& value, const T& low, const T& high) {
  return value < low ? low : (value > high ? high : value);
}

class Commander {
  private:
    State state;
    geometry_msgs::PoseStamped current_pose, goal_pose, final_pose;
    visualization_msgs::Marker goals;

    mavros_msgs::State px4_state;
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::SetMode land_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::CommandBool disarm_cmd;
    trajectory_msgs::MultiDOFJointTrajectory traj;

    ros::NodeHandle nh;
    ros::Rate rate;
    ros::Subscriber state_sub;
    ros::Publisher goal_pose_pub;
    ros::Publisher mavros_pose_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::Subscriber goals_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber traj_sub;
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
    void traj_cb (const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg) {
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
      state_sub = nh.subscribe<mavros_msgs::State> ("mavros/state", 10, &Commander::state_cb, this);
      goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped> ("commander/goal_pose", 10);
      mavros_pose_pub = nh.advertise<geometry_msgs::PoseStamped> ("mavros/setpoint_position/local", 10);
      arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming");
      set_mode_client = nh.serviceClient<mavros_msgs::SetMode> ("mavros/set_mode");
      goals_sub = nh.subscribe("goal_gen/goals", 1, &Commander::goals_cb, this);
      pose_sub = nh.subscribe("mavros/local_position/pose", 1, &Commander::pose_cb, this);
      traj_sub = nh.subscribe("waypoints", 1, &Commander::traj_cb, this);


      offb_set_mode.request.custom_mode = "OFFBOARD";
      land_set_mode.request.custom_mode = "AUTO.LAND";
      arm_cmd.request.value = true;
      disarm_cmd.request.value = false;

      goal_pose.pose.position.x = 0 + X_OFFSET;
      goal_pose.pose.position.y = 0;
      goal_pose.pose.position.z = TAKEOFF_ALTITUDE + Z_OFFSET;

      final_pose.pose.position.x = 4 + X_OFFSET;
      final_pose.pose.position.y = 0;
      final_pose.pose.position.z = TAKEOFF_ALTITUDE + Z_OFFSET;

      start_time = ros::Time::now();
      connect_mavros();
    }


    void explore() {
      goal_pose = final_pose;
      static ros::Time explore_start = ros::Time::now();
      float yaw = 1.3 * sin((ros::Time::now() - explore_start).toSec() * 2 * M_PI / YAW_PERIOD) ;
      tf2::Quaternion q = tf2::Quaternion(0, 0, yaw);  
      //std::cout << "x: " << q.x() << " y: " <<  q.y() << " z: " << q.z() << " w: " <<  q.w() << std::endl;
      goal_pose.pose.orientation.x = q.x();
      goal_pose.pose.orientation.y = q.y();
      goal_pose.pose.orientation.z = q.z();
      goal_pose.pose.orientation.w = q.w();

      if(ros::Time::now() - explore_start > ros::Duration(40.0)) {
        state = LAND;
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
      } else if (current_pose.pose.position.z > DIST_THRESH + Z_OFFSET || (ros::Time::now() - land_start).toSec() < 5.0) {
        ROS_INFO("Lowering");
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
      } else if (current_pose.pose.position.z < TAKEOFF_ALTITUDE + Z_OFFSET - DIST_THRESH) {
        ROS_INFO("Rising");
      } else {
        state = EXPLORE;
      }
    }

    void spray() {
    }

    void wait() {
      // HAHA do nothing buddy, for now...
    }

    void sim_pub_goal() {
      static ros::Time prev_time = ros::Time::now();
      ros::Time curr_time = ros::Time::now();
      static geometry_msgs::PoseStamped prev_goal = goal_pose;

      float diffs[3];
      diffs[0] = abs(goal_pose.pose.position.x - prev_goal.pose.position.x);
      diffs[1] = abs(goal_pose.pose.position.y - prev_goal.pose.position.y);
      diffs[2] = abs(goal_pose.pose.position.z - prev_goal.pose.position.z);
      float requested_dist = sqrt(pow(diffs[0], 2) + pow(diffs[1], 2)); 
      float delta_t = (ros::Time::now() - prev_time).toSec();

      geometry_msgs::PoseStamped incremental_pose = goal_pose;
      incremental_pose.pose.position.x = clamp(
          goal_pose.pose.position.x,
          prev_goal.pose.position.x - delta_t * MAX_H_SPEED * diffs[0] / requested_dist,
          prev_goal.pose.position.x + delta_t * MAX_H_SPEED * diffs[0] / requested_dist);
      incremental_pose.pose.position.y = clamp(
          goal_pose.pose.position.y,
          prev_goal.pose.position.y - delta_t * MAX_H_SPEED * diffs[1] / requested_dist,
          prev_goal.pose.position.y + delta_t * MAX_H_SPEED * diffs[1] / requested_dist);
      incremental_pose.pose.position.z = clamp(
          goal_pose.pose.position.z,
          prev_goal.pose.position.z - delta_t * MAX_V_SPEED,
          prev_goal.pose.position.z + delta_t * MAX_V_SPEED);

      prev_goal = incremental_pose;
      prev_time = curr_time;
      mavros_pose_pub.publish(incremental_pose);
    }
      

    void run() {
      ROS_INFO("here");
      state = TAKEOFF;
      while(ros::ok()) {
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
        //mavros_pose_pub.publish(goal_pose);
        sim_pub_goal();

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
