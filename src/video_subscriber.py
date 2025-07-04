#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage

bridge = CvBridge()

def image_callback(msg):
    global bridge
    try:
        cv_image = bridge.imgmsg_to_cv2(msg,"bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    
    cv2.imshow("Camera Feed", cv_image)

def main():
    rospy.init_node("VideoSubscriber", anonymous=True)
    rospy.Subscriber("/camera/Image/compressed", CompressedImage, image_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down video subscriber node.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass