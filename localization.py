#!/usr/bin/env python
'''
Bruno Sanchez Garcia				A01378960
Carlos Antonio Pazos Reyes 			A01378262
Manuel Agustin Diaz Vivanco		    A01379673

Nodo para calcular y publicar la posicion ideal del robot
es decir sin ruido.
'''
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion, Point
import tf

class Odometry_nonholonomic_robot():

    def __init__(self):
        rospy.init_node("localization")
        self.t0 = rospy.Time.now() # node initial time
        self.x_k = 0.0  # -0.955           # initial gazebo state to sync in rviz
        self.y_k = 0.0  # 0.753
        self.th_k = 0.0 # -1.57
        self.V = 0.0                # variables to receive cmd_vel
        self.w = 0.0
        self.tb = tf.TransformBroadcaster() # to move/update robot in rviz
        self.F = 60.0                      # frequency
        self.rate = rospy.Rate(self.F)
        self.T = float(1/self.F)            # sample time
        self.odom = rospy.Publisher('/pose_sim', PoseStamped, queue_size=10)
        self.Wl = rospy.Publisher('/rviz/wl', Float32, queue_size=10)
        self.Wr = rospy.Publisher('/rviz/wr', Float32, queue_size=10)
        self.pJS = rospy.Publisher('/joint_states', JointState, queue_size=10)
        rospy.Subscriber('/cmd_vel',Twist,self.cmd_vel_callback)
        self.r = 0.05           # robot radius
        self.l = 0.188          # distance between wheels
        self.pose_stamped = PoseStamped()   # msg of ideal robot state
        self.frame_id = 'base_link'         # frame robot base
        self.js = JointState()              # publish to update wheel movement
        self.js.name = ["right_wheel_joint", "left_wheel_joint"] # wheel joints

    def cmd_vel_callback(self,msg):
        self.V = msg.linear.x       # receive cmd_vel velocities
        self.w = msg.angular.z

    def Odometry(self):
        alpha_k = 0.0               # robot with no noise
        # alpha_k = np.random.normal(0.0, 0.5)
        x_k1 = self.x_k + (self.T *(self.V + alpha_k))* np.cos(self.th_k)   # update x
        y_k1 = self.y_k + (self.T *(self.V + alpha_k))* np.sin(self.th_k)   # update y
        th_k1 = self.th_k + (self.T *(self.w + alpha_k))                    # update yaw
        Q = tf.transformations.quaternion_from_euler(0, 0, th_k1)   # get orientation quaternion
        time_stamp = rospy.Time.now()
        self.pose_stamped.header.stamp = time_stamp     # time stamp
        self.pose_stamped.header.frame_id = self.frame_id   # same frame base_link
        self.pose_stamped.pose.position = Point(x_k1, y_k1, 0.0)    # build position
        self.pose_stamped.pose.orientation = Quaternion(Q[0],Q[1],Q[2],Q[3])    # build orientation
        self.odom.publish(self.pose_stamped)            # publish state
        wl = (self.V - 0.5 * self.l * self. w)/self.r   # get right wheel angular speed
        self.Wl.publish(wl)
        wr = (self.V + 0.5 * self.l * self.w)/self.r    # get left wheel angular speed
        self.Wr.publish(wr)
        self.tb.sendTransform([x_k1,      # send transform tu update rviz
                               y_k1,0],
                               Q,
                               time_stamp, "base_link", "map")
        
        t = time_stamp - self.t0        # time to update wheel movement
        self.js.position = [wr * t.to_sec(), wl * t.to_sec()]   # calculation to move wheels
        self.js.header.stamp = time_stamp
        self.pJS.publish(self.js)       # update wheel movement in rviz

        self.x_k = x_k1         # update state for next iteration
        self.y_k = y_k1
        self.th_k = th_k1

        print(self.pose_stamped)


    def main(self):
        while not rospy.is_shutdown():
            try:
                self.Odometry()
            except Exception as e :
                print(e)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        challenge = Odometry_nonholonomic_robot()
        challenge.main()
        print("exiting main")

    except (rospy.ROSInterruptException, rospy.ROSException):
        print("topic was closed during publish")
