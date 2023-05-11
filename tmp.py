#!/usr/bin/env python
'''
Bruno Sanchez Garcia				A01378960
Carlos Antonio Pazos Reyes 			A01378262
Manuel Agustin Diaz Vivanco			A01379673

Nodo reactive navigation:
Probar follow wall left y right
'''

# import libraries
import rospy
from geometry_msgs.msg import PoseStamped, Twist, Point
from tf.transformations import euler_from_quaternion
import numpy as np

class ReactiveNavigation():
    def __init__(self):
        rospy.init_node('tmp')
        self.rate = rospy.Rate(60)
        self.vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        rospy.Subscriber('/pose_sim',PoseStamped,self.pose_callback)
        self.pose = PoseStamped()
        self.goal = Point(-5.0,-5.0,0.0)

    def pose_callback(self,msg):
        self.pose = msg

    def reactive_navigation(self):
        delta_y = self.goal.y - self.pose.pose.position.y
        delta_x = self.goal.x - self.pose.pose.position.x
        robot_th = self.get_th(self.pose.pose.orientation)
        goal_th = np.arctan2(delta_y,delta_x)
        d_error = np.sqrt( (self.goal.x-self.pose.pose.position.x)**2 + (self.goal.y-self.pose.pose.position.y)**2 )
        ang_error = goal_th - robot_th
        print('d_error:',d_error,'ang_error:',ang_error)
        
        msg = Twist()
        if d_error <= 0.2:
            msg.angular.z = 0.0
            msg.linear.x = 0.0
        else:
            msg.angular.z =  ang_error
            msg.linear.x = 0.2
        self.vel_pub.publish(msg)
    
    def get_th(self,q):
        # get the yaw angle from quaternion
        Q = [0.0,0.0,0.0,0.0]
        Q[0] = q.x
        Q[1] = q.y
        Q[2] = q.z
        Q[3] = q.w
        (roll,pitch,yaw) = euler_from_quaternion(Q)
        return yaw

    def main(self):
        while not rospy.is_shutdown():
            try:
                self.reactive_navigation() 
            except Exception as e:
                print(e)
        self.rate.sleep() 
					

if __name__ == '__main__':
	try:
		rn = ReactiveNavigation()
		rn.main()
	except (rospy.ROSInterruptException, rospy.ROSException):
		print("topic was closed during publish")
