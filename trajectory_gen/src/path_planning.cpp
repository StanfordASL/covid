#include <ros/ros.h>

#include <fcl/config.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/collision_geometry.h>
#include <fcl/narrowphase/collision.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <ompl/config.h>
/*
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
*/

#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/util/Console.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace om = ompl::msg;

ros::Publisher traj_pub;
ros::Publisher vis_pub;


class planner {

public:
    planner(void) {
        om::setLogLevel(om::LOG_NONE);
        drone_geom = std::shared_ptr<fcl::CollisionGeometryd>(new fcl::Boxd(0.7, 0.7, 0.5));
        tf_listener = new tf2_ros::TransformListener(tf_buffer);
        space = ob::StateSpacePtr(new ob::SE3StateSpace());
        ob::RealVectorBounds bounds(3);
        bounds.setLow(0, -1);
        bounds.setHigh(0, 10);
        bounds.setLow(1, -4);
        bounds.setHigh(1, 4);
        bounds.setLow(2, 0);
        bounds.setHigh(2, 1.2);
        space->as<ob::SE3StateSpace>()->setBounds(bounds);
        goal_pose.position.x = 0;
        goal_pose.position.y = 0;
        goal_pose.position.z = 0.7;

        si = std::make_shared<ob::SpaceInformation>(space);
        si->setStateValidityChecker(std::bind(&planner::isStateValid, this, std::placeholders::_1));
        pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));
        pdef->setOptimizationObjective(ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si)));

    }

    ~planner() {}

    void plan() {

        pdef->clearGoal();
        pdef->clearStartStates();

        setDronePose();
        ob::ScopedState<ob::SE3StateSpace> start(space);
        start->setXYZ(drone_pose.position.x,
                      drone_pose.position.y,
                      drone_pose.position.z);
		start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        ob::ScopedState<ob::SE3StateSpace> goal(space);
        goal->setXYZ(goal_pose.position.x,
                      goal_pose.position.y,
                      goal_pose.position.z);
		goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

        pdef->setStartAndGoalStates(start, goal);

        og::InformedRRTstar* solver = new og::InformedRRTstar(si);

        ob::PlannerPtr plan(new og::InformedRRTstar(si));
        plan->setProblemDefinition(pdef);
        plan->setup();

        ros::Time start_time = ros::Time::now();
        ob::PlannerStatus solved = plan->solve(2); 
        //std::cout << (ros::Time::now() - start_time).toSec() << std::endl;


        geometry_msgs::PoseArray msg;
        if (solved) {
            visualization_msgs::Marker clear;
            clear.action = visualization_msgs::Marker::DELETEALL;
            vis_pub.publish(clear);

            og::PathGeometric*  path = pdef->getSolutionPath()->as<og::PathGeometric>();
            geometry_msgs::Pose pose;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "map";

            visualization_msgs::Marker path_marker;
            path_marker.header.frame_id = "map";
            path_marker.header.stamp = ros::Time::now();
            path_marker.ns = "path";
            path_marker.action = visualization_msgs::Marker::ADD;
            path_marker.pose.orientation.w = 1.0;
            path_marker.id = 1;
            path_marker.type = visualization_msgs::Marker::LINE_STRIP;
            path_marker.scale.x = 0.05;
            path_marker.color.a = 1.0;
            path_marker.color.r = 1.0;


            for (std::size_t path_idx = 0; path_idx < path->getStateCount(); path_idx++) {
                const ob::SE3StateSpace::StateType *se3state = path->getState(path_idx)->as<ob::SE3StateSpace::StateType>();
                const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
                const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
                pose.position.x = pos->values[0];
                pose.position.y = pos->values[1];
                pose.position.z = pos->values[2];
                pose.orientation.x = rot->x;
                pose.orientation.y = rot->y;
				        pose.orientation.z = rot->z;
                pose.orientation.w = rot->w;
                msg.poses.push_back(pose);

                geometry_msgs::Point p;
                p.x = pos->values[0];
                p.y = pos->values[1];
                p.z = pos->values[2];
                path_marker.points.push_back(p);                

            }
            vis_pub.publish(path_marker);
            traj_pub.publish(msg);
        } else {
            traj_pub.publish(msg);
        }
        pdef->clearSolutionPaths();
    }

    void setMap(std::shared_ptr<fcl::CollisionGeometryd> new_map_geom) {
        map_geom = new_map_geom;
    }

    void setGoal(const geometry_msgs::PoseStamped &pose_stamped) {
        goal_pose.position.x = pose_stamped.pose.position.x;
        goal_pose.position.y = pose_stamped.pose.position.y;
        goal_pose.position.z = pose_stamped.pose.position.z;
    }

    void setDronePose(void) {
        geometry_msgs::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer.lookupTransform("map", "base_link", ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }
        drone_pose.orientation = transform_stamped.transform.rotation;
        drone_pose.position.x = transform_stamped.transform.translation.x;
        drone_pose.position.y = transform_stamped.transform.translation.y;
        drone_pose.position.z = transform_stamped.transform.translation.z;
    }

    bool isInCollision(void) { 
        setDronePose();
        if (map_geom.get() == NULL) {
            ROS_WARN("No map data");
            return true;
        }
        fcl::CollisionObjectd drone_obj(drone_geom);
        ROS_INFO("\nx:%f y:%f z:%f", drone_pose.position.x, drone_pose.position.y, drone_pose.position.z);
        drone_obj.setQuatRotation(fcl::Quaterniond(drone_pose.orientation.w,
                                                   drone_pose.orientation.x,
                                                  drone_pose.orientation.y,
                                                   drone_pose.orientation.z));
        drone_obj.setTranslation(fcl::Vector3d(drone_pose.position.x,
                                               drone_pose.position.y,
                                               drone_pose.position.z));
        fcl::CollisionObjectd map_obj(map_geom);
        fcl::CollisionRequestd request;
        fcl::CollisionResultd result;
        fcl::collide(&drone_obj, &map_obj, request, result);
        return result.isCollision();
    }


private:

    std::shared_ptr<fcl::CollisionGeometryd> map_geom;
    std::shared_ptr<fcl::CollisionGeometryd> drone_geom;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener* tf_listener;
    geometry_msgs::Pose drone_pose;
    geometry_msgs::Pose goal_pose;

    ob::StateSpacePtr space;
    ob::SpaceInformationPtr si;
    ob::ProblemDefinitionPtr pdef;


    bool isStateValid(const ob::State *state) {
        if (map_geom.get() == NULL) {
            ROS_WARN("No map data");
            return false;
        }
        const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();
        const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0); 
        const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
        fcl::CollisionObjectd drone_obj(drone_geom);
        drone_obj.setQuatRotation(fcl::Quaterniond(rot->w, rot->x, rot->y, rot->z));        
        drone_obj.setTranslation(fcl::Vector3d(pos->values[0], pos->values[1], pos->values[2]));  
        fcl::CollisionObjectd map_obj(map_geom);
        fcl::CollisionRequestd request;
        fcl::CollisionResultd result;
        fcl::collide(&drone_obj, &map_obj, request, result);
        return(!result.isCollision());
    }
};

void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg, planner* planner_ptr)
{
	  octomap::ColorOcTree* octomap_tree = dynamic_cast<octomap::ColorOcTree*>(octomap_msgs::msgToMap(*msg));
	  fcl::OcTreed* fcl_tree = new fcl::OcTreed(std::shared_ptr<const octomap::ColorOcTree>(octomap_tree));
    planner_ptr->setMap(std::shared_ptr<fcl::CollisionGeometryd>(fcl_tree));
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_stamped, planner* planner_ptr)
{
    planner_ptr->setGoal(*pose_stamped);
}

int main(int argc, char **argv) {

    //auto si(std::make_shared<ob::SpaceInformation>(space));

    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;
    planner planner_object;
	  ros::Subscriber octree_sub = nh.subscribe<octomap_msgs::Octomap>("/rtabmap/octomap_binary", 1, boost::bind(&octomapCallback, _1, &planner_object)); 
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/commander/curr_request", 1, boost::bind(&goalCallback, _1, &planner_object));

    vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    traj_pub = nh.advertise<geometry_msgs::PoseArray>("waypoints",1);
    
    ros::Rate rate(2.0);
    while (ros::ok()) {
        planner_object.plan();
        rate.sleep();
        ros::spinOnce();
    }
    return(0);
}
           
