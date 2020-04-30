#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist



def move():
    # Starts a new node
    rospy.init_node('move', anonymous=True)
    velocity_publisher = rospy.Publisher('/quadrotor_1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    #Receiveing the user's input
    # print("Let's move your robot")
    # speed = input("Input your speed:")
    # distance = input("Type your distance:")
    # isForward = input("Foward?: ")#True or False


    #Checking if the movement is forward or backwards
    # if(isForward):
    #     vel_msg.linear.x = abs(speed)
    # else:
    #     vel_msg.linear.x = -abs(speed)
    # #Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0


    while not rospy.is_shutdown():

        if it<100000:
            vel_msg.linear.x = 0.5
            it = it+1
        else:
            vel_msg.linear.x = -0.5
            it = it+1
            if it ==200000: it=0

        print(it)
        #Force the robot to stop
        velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        #Testing our function
        it = 0
        move()
    except rospy.ROSInterruptException: pass
