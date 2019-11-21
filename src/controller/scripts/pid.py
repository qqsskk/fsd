#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from fs_msgs.msg import ControllerOutput
from fs_msgs.msg import PIDControlled
from controller import controller
from tf.transformations import euler_from_quaternion
import numpy as np

class pid():

    def __init__(self, kp=0.0, ki=0.0, kd=0.0):
        """
        """
        self.x = 0.0
        self.y = 0.0
        self.t = rospy.Time.now()

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.v_error = 0.0
        self.v_error_sum = 0.0

        self.state_available = False
        self.setpoint_available = False

        #Publishers/Subscribers
        self.pub_tau_delta = rospy.Publisher('/pid_controller', PIDControlled, queue_size=1)

        self.pub_lin_vel = rospy.Publisher('/LinearVelocity',Float32,queue_size=1)
        self.sub_pose = rospy.Subscriber('/simulator/odom', Odometry, self.parse_state)
        self.sub_steer_control = rospy.Subscriber('/control_out', ControllerOutput, self.parse_setpoint)

    def parse_state(self, odom_msg):
        """
        parse_state saves the state variables from odom message.
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y
        orientation_q = odom_msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.yaw = euler_from_quaternion(orientation_list)[2]
        self.state_available = True
        """
        #print "STATE RECEIVED!"
        self.x_prev = self.x
        self.y_prev = self.y
        self.t_prev = self.t

        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y
        self.t = odom_msg.header.stamp
        # dt = self.t-self.t_prev

        self.dt = (self.t-self.t_prev).to_sec()
        self.v_x = odom_msg.twist.twist.linear.x
        self.v_y = odom_msg.twist.twist.linear.y
        self.v = (self.v_x**2 + self.v_y**2)**0.5

        # self.v = 15.

        orientation_q = odom_msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.yaw = euler_from_quaternion(orientation_list)[2]
        # Linear Velocity Message
        vel_msg = Float32()
        vel_msg.data = self.v
        self.pub_lin_vel.publish(vel_msg)

        # self.yaw = 0.

        #print('v: ', self.v)
        #print('yaw: ', self.yaw)

        self.state_available = True

    def parse_setpoint(self, setpoint_msg):
        """
        """
        print "SETPOINT RECEIVED!"
        self.v_setpoint = setpoint_msg.velocity
        print('v_setpoint: ',self.v_setpoint)
        self.delta_setpoint = setpoint_msg.steering
        print('delta_setpoint: ',self.delta_setpoint)
        self.setpoint_available = True

    def pid_update(self):

        INTEGRAL_SATURATION = 100.

        self.v_error_prev = self.v_error

        self.v_error = self.v_setpoint - self.v

        print('v_error: ',self.v_error)

        self.v_error_sum = self.v_error_sum + self.v_error
        self.v_error_int = self.v_error_sum*self.dt

        np.clip(self.v_error_int,-INTEGRAL_SATURATION,INTEGRAL_SATURATION)

        self.v_error_diff = self.v_error - self.v_error_prev
        self.v_error_deriv = self.v_error_diff/self.dt

        self.tau = self.kp*self.v_error + self.ki*self.v_error_int + self.kd*self.v_error_deriv
        self.delta = self.delta_setpoint

    def pedalcontrol(self):
        BRAKE_THRESHOLD = -5.
        MAX_BRAKE = -100.
        TAU_TO_BRAKE = 100./(BRAKE_THRESHOLD-MAX_BRAKE)

        MAX_TORQUE = 2500

        if self.tau < 0:
            self.torque = 0.
            if self.tau < MAX_BRAKE:
                self.brake = 100.
            elif self.tau < BRAKE_THRESHOLD:
                self.brake = TAU_TO_BRAKE*(BRAKE_THRESHOLD-self.tau)

                # The Curve-fitting is done to obtain a 3rd order polynomial
                points = np.array([(10,BRAKE_THRESHOLD),(20,-10),(70,-80),(100,MAX_BRAKE)])
                # Extracting the abscissa and ordinate
                y = points[:,0]
                x = points[:,1]
                # Calculating the polynomial
                z = np.polyfit(x,y,3)
                # Converting it into a function
                p = np.poly1d(z)
                # Calculating the torque request from the brake_signal
                self.brake = p(self.tau)
                # Limitting the maximum and minimum torque
                #return np.clip(self.brake,0,100)

            else:
                self.brake = 0.
        else:
            self.brake = 0.
            if self.tau > MAX_TORQUE:
                self.torque = MAX_TORQUE
            else:
                self.torque = self.tau


    def publish_tau_delta(self):
        self.pid_update()
        # self.state_available = False
        # self.setpoint_available = False

        self.pedalcontrol()
        torque_brake_steer_msg = PIDControlled()
        print('torque: ',self.torque)
        print('brake: ',self.brake)
        print('delta: ',self.delta)
        torque_brake_steer_msg.torque = self.torque
        torque_brake_steer_msg.brake = self.brake
        torque_brake_steer_msg.steering = self.delta
        self.pub_tau_delta.publish(torque_brake_steer_msg)



if __name__ == "__main__":
    rospy.init_node('PID_controller')
    rate = rospy.Rate(50)
    my_controller = pid(30.0,5.5,2.0) # 25.0,2.0,0.5 - Working set
    while not rospy.is_shutdown():
        if my_controller.state_available and my_controller.setpoint_available:
            my_controller.publish_tau_delta()
            rate.sleep()
