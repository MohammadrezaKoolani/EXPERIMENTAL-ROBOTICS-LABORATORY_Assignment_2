import rospy
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from lab_assignment_2.msg import Marker

class ArucoMarkerPublisher:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.search_sub = rospy.Subscriber("/rosbot/search_id", Int32, self.search_callback)
        self.marker_pub = rospy.Publisher("/rosbot/marker_found", Marker, queue_size=1)
        self.image_pub = rospy.Publisher("/result_image", Image, queue_size=1)
        
        # ArUco Marker Detection setup
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()
        
    def search_callback(self, target_msg):
        rospy.loginfo(f"Looking for marker ID: {target_msg.data}")
        self.target = target_msg.data

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        corners, ids, _ = aruco.detectMarkers(cv_image, self.dictionary, parameters=self.parameters)
        
        if ids is not None:
            for i, id in enumerate(ids):
                if id[0] == self.target:
                    # Assuming Marker message has fields: id, center.x, center.y
                    marker_msg = Marker()
                    marker_msg.id = id[0]
                    centerX = int((corners[i][0][0][0] + corners[i][0][2][0]) / 2)
                    centerY = int((corners[i][0][0][1] + corners[i][0][2][1]) / 2)
                    marker_msg.center.x = centerX
                    marker_msg.center.y = centerY
                    self.marker_pub.publish(marker_msg)
                    
                    # Draw detected markers
                    aruco.drawDetectedMarkers(cv_image, corners, ids)
                    
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)

def main():
    rospy.init_node('aruco_marker_publisher', anonymous=True)
    ArucoMarkerPublisher()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()


