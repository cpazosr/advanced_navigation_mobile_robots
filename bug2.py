#!/usr/bin/env python
'''
Bruno Sanchez Garcia				A01378960
Carlos Antonio Pazos Reyes 			A01378262
Manuel Agustin Diaz Vivanco			A01379673

Nodo navegacion reactiva por bug2
'''

# import libraries
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import numpy as np
import follow_wall

class Bug2():
    def __init__(self):
        rospy.init_node('bug2')
        self.vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)   # control robot
        rospy.Subscriber('/scan',LaserScan,self.lidar_callback)         # lidar info
        # rospy.Subscriber('pose_sim',PoseStamped,self.pose_callback)   # localization node
        rospy.Subscriber('/true_odometry',Odometry,self.odom_callback)  # gazebo localization
        self.fs = 60.0
        self.rate = rospy.Rate(self.fs)     # ros rate
        self.scan = LaserScan()             # lidar scan variable
        self.robot_pose = Odometry()        # robot odometry from gazebo
        self.robot_pose.pose.pose.position = Point(0.0,0.0,0.0)
        self.goal = Point(-9.0,9.0,0.0)      # goal point
        self.wall_dist = 0.5    # follow wall distance
        self.w_max = 1.5        # follow wall max angular speed
        self.v_max = 0.3        # follow wall max linear speed
        self.ang_thresh = 0.05  # angular threshold for checking direction alignment
        self.dist_thresh = 0.1  # distance threshold for checking goal reahced
        self.line_thresh = 0.01 # distance threshold for checking if close to line
        self.state = 1          # state machine
        self.side = 'right'     # default follow wall side
        self.m = (self.goal.y - self.robot_pose.pose.pose.position.y) / (self.goal.x - self.robot_pose.pose.pose.position.x) # line slope
        self.b = self.robot_pose.pose.pose.position.y - self.m*self.robot_pose.pose.pose.position.x # inital to origin

    def lidar_callback(self,msg):
        self.scan = msg     # lidar scan message

    def odom_callback(self,msg):
        self.robot_pose = msg   # robot odometry from gazebo message

    def reactive_navigation(self):
        # bug 2 algorithm
        distance_ahead_left = follow_wall.get_distance_in_sector(self.scan, # scan from front to 15deg to the left
                                                                 -np.pi,
                                                                 -np.pi + (15*np.pi/180)) # 15 deg left
        distance_ahead_right = follow_wall.get_distance_in_sector(self.scan,    # scan from front to 15 deg to the right
                                                                  np.pi - (15*np.pi/180),
                                                                  np.pi) # 15 deg right
        distance_ahead = (distance_ahead_left + distance_ahead_right) / 2.0 # 30 deg front span

        delta_y = self.goal.y - self.robot_pose.pose.pose.position.y    # x component of distance to goal
        delta_x = self.goal.x - self.robot_pose.pose.pose.position.x    # y component of distance to goal
        robot_th = self.get_th(self.robot_pose.pose.pose.orientation)   # theta angle of robot
        goal_th = np.arctan2(delta_y,delta_x)                           # theta desired to goal
        d_error = np.sqrt( (self.goal.x-self.robot_pose.pose.pose.position.x)**2 + (self.goal.y-self.robot_pose.pose.pose.position.y)**2 )  # distance to goal
        ang_error = goal_th - robot_th      # angular difference to goal orientation
        dist_to_line = self.distance_to_line(self.robot_pose.pose.pose.position.x,self.robot_pose.pose.pose.position.y,self.m,self.b) # distance to line
        # debug prints
        # print('d_error: %.2f'%d_error,'ang_error:%.2f'%ang_error,'dist_ahead:%.2f'%distance_ahead,'state:',self.state)
        # print('state:%d'%self.state,'goal_th:%.2f'%goal_th,'robot_th:%.2f'%robot_th,'ang_err:%.2f'%ang_error,'d_ahead:%.2f'%distance_ahead)
        print('state:%d'%self.state,'d_left:%.3f'%distance_ahead_left,'d_right:%.3f'%distance_ahead_right,'ang_err:%.4f'%ang_error,'d_ahead:%.2f'%distance_ahead)
        # print('state:%d'%self.state,'d_left:%.3f'%distance_ahead_left,'d_right:%.3f'%distance_ahead_right,'ang_err:%.4f'%ang_error,'side:%s'%self.side)

        # build cmd_vel message
        msg = Twist()
        msg.linear.x = self.v_max   # constante linear speed
        if self.state == 1:             # go to point
            if d_error <= self.dist_thresh:     # goal found
                self.state = 3      # go to final state
                msg.angular.z = 0.0 # stop movement
                msg.linear.x = 0.0
            elif distance_ahead <= self.wall_dist*2:   # hit
                self.state = 2      # go to follow wall state
                if distance_ahead_left < distance_ahead_right:  # git left
                    self.side= 'left'
                else:                   # hit right
                    self.side = 'right'
            else:
                msg.angular.z = ang_error   # keep going to point
                # self.state = 1
        elif self.state == 2:       # follow wall
            if dist_to_line <= self.line_thresh: # leave
                self.state = 4
            else:                   # keep following wall
                z,x = follow_wall.follow_wall(self.side,self.scan,self.wall_dist,self.w_max,self.v_max)
                msg.angular.z = z
                # self.state = 2
        elif self.state == 3:       # on goal
            msg.angular.z = 0.0     # stop robot
            msg.linear.x = 0.0
        elif self.state == 4:       # substate to align robot angle to goal
            msg.linear.x = 0
            msg.angular.z = ang_error       # rotate over self axis
            if abs(ang_error) <= self.ang_thresh:   # if aligned
                self.state = 1
        self.vel_pub.publish(msg)   # send cmd_vel
    
    def get_th(self,q):
        # get the yaw angle from quaternion
        Q = [0.0,0.0,0.0,0.0]
        Q[0] = q.x
        Q[1] = q.y
        Q[2] = q.z
        Q[3] = q.w
        (roll,pitch,yaw) = euler_from_quaternion(Q)
        return yaw

    def distance_to_line(self,x,y,m,b):
        "-mx + y - b = 0"
        A = - m * 10
        B = 10
        C = - B * b
        "d = |Ax+By+C|/sqrt(A**2+B**2)"
        d = abs(A*x + B*y + C) / (np.sqrt(A**2 + B**2))
        return d

    def main(self):
        # main execution ROS node
        while not rospy.is_shutdown():
            try:
                self.reactive_navigation() 
            except Exception as e:
                print(e)
        self.rate.sleep() 
					

if __name__ == '__main__':
    # call bug 2
	try:
		b2 = Bug2()
		b2.main()
	except (rospy.ROSInterruptException, rospy.ROSException):
		print("topic was closed during publish")
