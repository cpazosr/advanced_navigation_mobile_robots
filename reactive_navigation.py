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
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
import follow_wall

class ReactiveNavigation():
    def __init__(self):
        rospy.init_node('reative_navigation')
        self.vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        rospy.Subscriber('/scan',LaserScan,self.lidar_callback)
        self.fs = 60.0
        self.rate = rospy.Rate(self.fs)
        self.scan = LaserScan()

    def lidar_callback(self,msg):
        self.scan = msg

    def reactive_navigation(self):
        # print(follow_wall.range_index(self.scan,0.0))
        z,x = follow_wall.follow_wall('right',self.scan)
        msg = Twist()
        msg.angular.z =  z
        msg.linear.x = x
        self.vel_pub.publish(msg)

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
