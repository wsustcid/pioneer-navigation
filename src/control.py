#!/usr/bin/env python
'''
@Author: Shuai Wang
@Github: https://github.com/wsustcid
@Version: 1.0.0
@Date: 2020-11-26 14:58:57
@LastEditTime: 2020-12-15 11:10:28
@Description:  
'''
""" The robot controlling node 
1. Control the Pioneer-3DX mobile robot by the joystic in the driving fashion: 
  The brake, throttle, steering and gear created by the joystick are converted to the linear and angular velocity which can be directly used for the p3dx robot.
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class Control():
    def __init__(self):
        ## Init node
        rospy.init_node('robo_control')

        rospy.on_shutdown(self.shutdown)

        ## Params for the remote control 
        # joy axes and buttons index
        self.left_joy_lr = rospy.get_param("~left_joy_lr", 0) # steering
        self.A = rospy.get_param("A", 2) # v1 gear
        self.B = rospy.get_param("B", 1) # v2 gear
        self.X = rospy.get_param("X", 3) # w1 gear
        self.Y = rospy.get_param("Y", 0) # w2 gear
        self.LT = rospy.get_param("LT", 6) # brake
        self.RT = rospy.get_param("RT", 7) # throttle

        self.v1_min = rospy.get_param("~gear_v1_min", 0.0)
        self.v1_max = rospy.get_param("~gear_v1_max", 0.5)
        self.v2_min = rospy.get_param("~gear_v2_min", 0.3)
        self.v2_max = rospy.get_param("~gear_v2_max", 0.8)
        self.w1_scale = rospy.get_param("~gear_w1", 0.5)
        self.w2_scale = rospy.get_param("~gear_w2", 1.0)
        self.throttle_delta = rospy.get_param("~throttle_increment", 0.01)
        self.brake_delta = rospy.get_param("~brake_decrement", 0.05)


    def teleop(self):
        # initialize
        self.steering = 0.0
        self.throttle_pressed = False
        self.brake_pressed    = False

        self.gear_v1_pressed  = False
        self.gear_v2_pressed  = False
        self.gear_w1_pressed  = False
        self.gear_w2_pressed  = False
        
        # vel_cmd
        self.vel = Twist()
        self.last_v = 0.0

        # Joy msg subscriber (global topic)
        joy_sub = rospy.Subscriber("/joy", Joy, self.callback_joy, queue_size=1)

        # Control msg publisher (private topic )
        self.vel_pub   = rospy.Publisher("/RosAria/cmd_vel", Twist, queue_size=1)

        while not rospy.is_shutdown():
            # convert joy cmd to liner and angular velocity
            v, w = self.joy_to_cmd(self.last_v)
            # pub v, w
            self.vel.linear.x = v
            self.vel.angular.z = w
            self.vel_pub.publish(self.vel)

            self.last_v = v
            print("cmd v: %s, w: %s" % (v, w))

            rospy.sleep(0.1)



    # detect joy commands
    def callback_joy(self, joy_data):
        """ This func only will be called when receiving new joy data!
        # To ensure there is only one gear is valid at one time
        # the other gear will set to false once one gear button is pressed. 
        # The pressed gear will not be deteced again
        # only when the state of it is false(the other gear is pressed)
        """
        # detect buttoms
        if not self.gear_v1_pressed:
            self.gear_v1_pressed  = joy_data.buttons[self.A]
            if self.gear_v1_pressed:
                self.gear_v2_pressed = False

        if not self.gear_v2_pressed:
            self.gear_v2_pressed  = joy_data.buttons[self.B]
            if self.gear_v2_pressed:
                self.gear_v1_pressed = False
        
        if not self.gear_w1_pressed:
            self.gear_w1_pressed  = joy_data.buttons[self.X]
            if self.gear_w1_pressed:
                self.gear_w2_pressed = False
        
        if not self.gear_w2_pressed:
            self.gear_w2_pressed  = joy_data.buttons[self.Y]
            if self.gear_w2_pressed:
                self.gear_w1_pressed = False

        self.steering = joy_data.axes[self.left_joy_lr]
        self.brake_pressed    = joy_data.buttons[self.LT]
        self.throttle_pressed = joy_data.buttons[self.RT]

        
    def joy_to_cmd(self, last_v):
        """covert the joy commands to the partical control commands
        """
        # set v interval according to the gear
        if self.gear_v1_pressed:
            vmin = self.v1_min
            vmax = self.v1_max
        elif self.gear_v2_pressed:
            vmin = self.v2_min
            vmax = self.v2_max
        else:
            vmin = 0.0
            vmax = 0.0 # the car won't move in gear 0 (no gear is choosed) 
        
        if self.gear_w1_pressed:
            w_scale = self.w1_scale
        elif self.gear_w2_pressed:
            w_scale = self.w2_scale
        else:
            w_scale = 0.0
        
        v_delta = 0.0 # reset v delta for each update
        if self.brake_pressed:
            v_delta -= self.brake_delta    
        if self.throttle_pressed:
            v_delta += self.throttle_delta
        
        v =  min(vmax, max(vmin, last_v + v_delta))
        w =  w_scale * self.steering # the range of self.steering is [-1,1]

        return v, w

    def shutdown(self):
        # stop the car
        self.vel_pub.publish(Twist())
        rospy.loginfo("Stopped the car!")

if __name__ == "__main__":
    try:
        control = Control()
        control.teleop()
    except:
        rospy.loginfo("The node is terminated!")