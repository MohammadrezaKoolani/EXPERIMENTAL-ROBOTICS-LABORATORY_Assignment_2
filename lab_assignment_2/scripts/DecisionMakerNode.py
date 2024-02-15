#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class LogicNode:
    def __init__(self):
        rospy.init_node('logic_node', anonymous=True)
        
        # Subscribers
        self.marker_sub = rospy.Subscriber('/rosbot/marker_found', Int32, self.marker_callback)
        
        # Publishers
        self.cmd_pub = rospy.Publisher('/rosbot/command', String, queue_size=10)
        
        # Action Clients
        self.waypoint_client = SimpleActionClient('go_to_waypoint', MoveBaseAction)
        self.waypoint_client.wait_for_server()

        # Internal State
        self.current_state = "searching"
        self.target_marker_id = 1

    def marker_callback(self, msg):
        if msg.data == self.target_marker_id and self.current_state == "searching":
            rospy.loginfo("Target marker found. Initiating navigation to marker...")
            self.navigate_to_marker()

    def navigate_to_marker(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = PoseStamped()  # Set your goal pose here
        self.waypoint_client.send_goal(goal)
        wait = self.waypoint_client.wait_for_result()
        if not wait:
            rospy.logerr("Navigation action failed.")
        else:
            result = self.waypoint_client.get_result()
            if result:
                rospy.loginfo("Successfully navigated to the marker.")
                self.execute_post_navigation_logic()

    def execute_post_navigation_logic(self):
        # Implement any post-navigation logic here
        self.cmd_pub.publish("Post-navigation action executed.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    logic_node = LogicNode()
    try:
        logic_node.run()
    except rospy.ROSInterruptException:
        pass


