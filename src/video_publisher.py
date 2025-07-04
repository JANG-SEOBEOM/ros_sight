#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage


def main():
    rospy.init_node("video_subscriber", anonymous=True)
    pub = rospy.Publisher("/camera/image/compressed", CompressedImage, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    bridge = CvBridge()

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        rospy.logerr("Could not open video device")
        return
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Failed to read frame from video device")
            break
        msg = bridge.cv2_to_compressed_imgmsg(frame, dst_format="jpeg")
        pub.publish(msg)
        rate.sleep() 
    cap.release()     

    rospy.spin()


if __name__ == "__main__":
    try: 
        main()
    except rospy.ROSInterruptException:
        pass