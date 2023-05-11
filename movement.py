#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
from geometry_msgs.msg import Quaternion
from std_srvs.srv import Empty, EmptyResponse
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import norm
class Movement_nonholonomic_robot():

    def __init__(self):
        rospy.init_node("challenge_movement")
        rospy.wait_for_service("/reset")
        self.res = rospy.ServiceProxy("reset", Empty)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist,queue_size=1)
        self.pose_sub = rospy.Subscriber("/pose_sim", PoseStamped, self.pose_callback)
        self.contador = 0
        self.vel = Twist()
        self.timer = rospy.Timer(rospy.Duration(7), self.Time_callback)
        self.time_passed = False
        self.rate = rospy.Rate(120)
        self.list_x = []
        self.list_y = []
        self.list_th = []
        self.P = PoseStamped()
        rospy.sleep(2)
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1)

    def pose_callback(self,msg):
        self.P = msg


    def Time_callback(self,msg):
        self.time_passed = True
        print("5 seconds passed, terminating movement " + str(self.contador))
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.vel_pub.publish(self.vel)
        self.list_x.append(self.P.pose.position.x)
        self.list_y.append(self.P.pose.position.y)
        x = self.P.pose.orientation.x
        y = self.P.pose.orientation.y
        z = self.P.pose.orientation.z
        w = self.P.pose.orientation.w
        euler = euler_from_quaternion([x, y, z, w])
        self.list_th.append(euler[2])
        print("x:  " + str(self.list_x))
        print("y:  " + str(self.list_y))
        print("th:   " + str(self.list_th))

        rospy.sleep(2)
        service = self.res()
        if service == EmptyResponse():
            self.contador += 1
            self.time_passed = False
        else:
            service = self.res()





    def Movement(self):

        if self.contador == 15:
            self.list_th.sort()
            hmean = np.mean(self.list_th)
            hstd = np.std(self.list_th)
            pdf = norm.pdf(self.list_th, hmean, hstd)
            #self.ax2.plot(self.list_th, pdf)
            #self.ax1.scatter(self.list_x, self.list_y, c ="blue")

            cov = np.cov(self.list_x, self.list_y)
            val, rot = np.linalg.eig(cov)
            val = np.sqrt(val)
            center = np.mean([self.list_x, self.list_y], axis=1)[:, None]

            t = np.linspace(0, 2.0 * np.pi, 1000)
            xy = np.stack((np.cos(t), np.sin(t)), axis=-1)
            '''self.ax1.set_xlim(-0.25,0.25)
            self.ax1.set_ylim(-0.25,0.25)
            self.ax2.set_xlim(-0.25,0.25)
            self.ax2.set_ylim(-0.25,0.25)'''
            self.ax1.scatter(self.list_x, self.list_y)
            self.ax1.plot(*(2*np.dot(rot, (val * xy).T) + center))


            plt.show(block=False)

        if self.time_passed == False and self.contador < 15:
            self.vel.linear.x = 0.1
            self.vel.angular.z = 0.0
            self.vel_pub.publish(self.vel)




    def main(self):
        print("runing main")
        while not rospy.is_shutdown():
            try:
                self.Movement()
            except Exception as e :
                print(e)
            self.rate.sleep()

if __name__ == '__main__':

    try:
        challenge = Movement_nonholonomic_robot()
        print("entering main")
        challenge.main()
        print("exiting main")

    except (rospy.ROSInterruptException, rospy.ROSException("topic was closed during publish()")):
        pass
