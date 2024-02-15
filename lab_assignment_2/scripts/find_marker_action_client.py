#!/usr/bin/env python
import rospy
import actionlib
from your_action_package.msg import FindMarkerAction, FindMarkerGoal

class FindMarkerActionClient(object):
    def __init__(self):
        rospy.init_node('find_marker_action_client_py', anonymous=True)
        self.client = actionlib.SimpleActionClient('find_marker_action', FindMarkerAction)
        rospy.loginfo("Waiting for action server to start.")
        self.client.wait_for_server()
        rospy.loginfo("Action server started, sending goal.")

    def send_goal_and_wait(self, marker_id):
        goal = FindMarkerGoal(marker_id=marker_id)
        self.client.send_goal(goal)
        rospy.loginfo("Goal sent to find marker with ID: {}".format(marker_id))
        self.client.wait_for_result()
        return self.client.get_result()

if __name__ == '__main__':
    try:
        marker_action_client = FindMarkerActionClient()
        marker_id = 11  # Example: Change this to the desired marker ID
        result = marker_action_client.send_goal_and_wait(marker_id)
        if result:
            rospy.loginfo("Result received: Success status is {}".format(result.success))
        else:
            rospy.loginfo("Action did not complete successfully.")
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted before completion.")

