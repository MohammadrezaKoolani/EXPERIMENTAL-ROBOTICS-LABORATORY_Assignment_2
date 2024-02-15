#!/usr/bin/env python
import rospy
import actionlib
from lab_assignment_2.msg import FindMarkerAction, FindMarkerGoal, Marker
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class FindMarkerActionServer:
    def __init__(self):
        self._as = actionlib.SimpleActionServer('find_marker', FindMarkerAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self.marker_pub = rospy.Publisher('/rosbot/search_id', Int32, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/rosbot/marker_found', Marker, self.marker_found_cb)
        self.marker_found = False
        self.id = Int32()

    def rotate_rosbot(self, ang_z):
        msg = Twist()
        msg.angular.z = ang_z
        self.cmd_vel_pub.publish(msg)

    def execute_cb(self, goal):
        rospy.loginfo('Finding marker ID %d', goal.marker_id)
        self.id.data = goal.marker_id
        self.marker_pub.publish(self.id)
        self.marker_found = False
        self.rotate_rosbot(0.8)
        while not self.marker_found:
            rospy.sleep(0.1)
        self.rotate_rosbot(0)
        rospy.loginfo('Found marker ID %d', self.id.data)
        self._as.set_succeeded()

    def marker_found_cb(self, msg):
        if msg.id == self.id.data:
            self.marker_found = True

if __name__ == '__main__':
    rospy.init_node('find_marker_action_server')
    server = FindMarkerActionServer()
    rospy.spin()

