#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from trajectory_msgs.msg import MultiDOFJointTrajectory
from threading import Thread



class OffboardPosctl():
    def __init__(self):
        self.local_pos = PoseStamped()
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.local_position_callback)
        self.trajectory = MultiDOFJointTrajectory()
        self.trajectory_sub = rospy.Subscriber('waypoints', MultiDOFJointTrajectory, self.trajectory_callback)
        self.curr_goal = PoseStamped()
        self.curr_goal_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.curr_goal_thread = Thread(target=self.mainLoop, args=())
        self.curr_goal_thread.daemon = True
        self.curr_goal_thread.start()

        self.sub_topics_ready = {
            key: False
            for key in [
                'local_pos', 'trajectory'
            ]
        }
    
    def local_position_callback(self, data):
        self.local_position = data
        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True

    def trajectory_callback(self, data):
        self.trajectory = data
        if not self.sub_topics_ready['trajectory']:
            self.sub_topics_ready['trajectory'] = True
        self.curr_goal.pose.position.x = self.trajectory.points[1].transforms[0].translation.x
        self.curr_goal.pose.position.y = self.trajectory.points[1].transforms[0].translation.y
        self.curr_goal.pose.position.z = self.trajectory.points[1].transforms[0].translation.z


    def mainLoop(self):
        rate = rospy.Rate(10)
        self.curr_goal.header = Header()
        self.curr_goal.header.frame_id = "base_link"
        while not rospy.is_shutdown():
            self.curr_goal.header.stamp = rospy.Time.now()
            self.curr_goal_pub.publish(self.curr_goal)
        
if __name__ == '__main__':
    rospy.init_node('test_node', anonymous=True)
    posctl = OffboardPosctl();

    rospy.spin();
