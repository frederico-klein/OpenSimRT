#!/usr/bin/env python3

# -*- coding: utf-8 -*-
"""
Created on Tue Jan 25 15:02:12 2022

@author: frekle
"""

import socket

# maybe make this a class�t

localIP     = "0.0.0.0"

localPort   = 8080

bufferSize  = 1024



msgFromServer       = "Hello UDP Client"

bytesToSend         = str.encode(msgFromServer)



# Create a datagram socket

UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)



# Bind to address and ip

UDPServerSocket.bind((localIP, localPort))



print("UDP server up and listening")



# Listen for incoming datagrams

#while(True):






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


        # read from the simple bridge

        bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)

        message = bytesAddressPair[0]

        address = bytesAddressPair[1]

        clientMsg = "Message from Client:{}".format(message)
        clientIP  = "Client IP Address:{}".format(address)

        print(clientMsg)
        print(clientIP)

        # Sending a reply to client
        UDPServerSocket.sendto(bytesToSend, address)


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
