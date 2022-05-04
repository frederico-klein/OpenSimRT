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

import re

def publisher():
    pub = []
    for i in range(3):
        pub.append(rospy.Publisher('pose'+str(i), PoseStamped, queue_size=1))
    pub_s1 = rospy.Publisher('reply', std_msgs.msg.String, queue_size=1)
    pub_s2 = rospy.Publisher('other', std_msgs.msg.String, queue_size=1)
    rospy.init_node('pose_publisher', anonymous=True)
    rate = rospy.Rate(2) # Hz
    h = std_msgs.msg.Header()
    h.frame_id = 'map'
    while not rospy.is_shutdown():
        p = []
        h.stamp = rospy.Time.now()
        for i in range(3):
            p.append(PoseStamped())

        for i in range(3):
            p[i].header = h
            p[i].pose.position.x = 0.5
            p[i].pose.position.y = -0.1
            p[i].pose.position.z = 1.0 + float(i)/10

        # read from the simple bridge

        bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)

        message = bytesAddressPair[0]

        address = bytesAddressPair[1]

        clientMsg = "Message from Client:{}".format(message)
        clientIP  = "Client IP Address:{}".format(address)

        pub_s1.publish(clientMsg)
        pub_s2.publish(clientIP)

        # Sending a reply to client
        UDPServerSocket.sendto(bytesToSend, address)


        (timestring, rest) = message.decode("utf-8").split(maxsplit=1)

        n = 18 # ? idk
        shouldbe3 = re.findall(" ".join(["[^\s]+"] * n), rest)
        if(len(shouldbe3) != 3 ):
            print(shouldbe3)

        for i in range(3):
            (q0,q1,q2,q3,rest) = shouldbe3[i].split(maxsplit=4)

            # Make sure the quaternion is valid and normalized

            p[i].pose.orientation.x = 0.0
            p[i].pose.orientation.y = 0.0
            p[i].pose.orientation.z = 0.0
            p[i].pose.orientation.w = 1.0
            pub[i].publish(p[i])
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy:
        pass
