#!/usr/bin/env python3

# from https://answers.ros.org/question/316829/how-to-publish-a-pose-in-quaternion/?answer=316844#post-id-316844
import rospy
from geometry_msgs.msg import PoseStamped
import std_msgs.msg

def publisher():
    pub = rospy.Publisher('pose', PoseStamped, queue_size=1)
    rospy.init_node('pose_publisher', anonymous=True)
    rate = rospy.Rate(2) # Hz
    h = std_msgs.msg.Header()
    h.frame_id = 'map'
    while not rospy.is_shutdown():
        p = PoseStamped()

        h.stamp = rospy.Time.now()
        p.header = h
        p.pose.position.x = 0.5
        p.pose.position.y = -0.1
        p.pose.position.z = 1.0
        # Make sure the quaternion is valid and normalized
        p.pose.orientation.x = 0.0
        p.pose.orientation.y = 0.0
        p.pose.orientation.z = 0.0
        p.pose.orientation.w = 1.0
        pub.publish(p)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy:
        pass
