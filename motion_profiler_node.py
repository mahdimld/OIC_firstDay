#!/usr/bin/env python
# license removed for brevity
import roslib
import numpy as np
import rospy
import re
from parsian_msgs.msg import parsian_robot_command
from parsian_msgs.msg import parsian_robot_task
from parsian_msgs.msg import parsian_world_model
from sensor_msgs.msg import Joy

# Import required Python code.
import motion_profiler
from point import Point
from dynamic_reconfigure.server import Server
from parsian_agent.cfg import motion_proConfig

speed = 10.0
RobotSpeed = 1
minSpeed = 0.3
maxSpeed = 1.5
Shooting = 2


class MotionProfilerNode:
    def __init__(self):
        rospy.init_node('Robo', anonymous=True)
        self.wm_sub = rospy.Subscriber('joy', Joy, self.MovingRobot, queue_size=1, buff_size=2 ** 24)
        self.p = rospy.Publisher('/agent_2/command', parsian_robot_command, queue_size=1, latch=True)
        self.m = rospy.Publisher('/world_model', parsian_world_model, queue_size=1, latch=True)
        rospy.Timer(rospy.Duration(1.0/60.0), self.TimerCallback)
        self.msg = parsian_robot_command()
        self.msg.robot_id = 1
        self.v1 = 0
        self.v2 = 0
        self.v3 = 0
        self.v4 = 0
        self.lastVel_N=0;
        self.lastVel_F=0;
        self.lastVel_W=0;

        rospy.spin()


    def velTuning(self):

        if self.msg.vel_F - self.lastVel_F>0.5 :
            self.msg.vel_F=0.5+self.lastVel_F;
        elif self.msg.vel_F - self.lastVel_F<-0.5 :
            self.msg.vel_F=-0.5+self.lastVel_F;

            if self.msg.vel_N - self.lastVel_N>0.5 :
                self.msg.vel_N=0.5+self.lastVel_N;
        elif self.msg.vel_N - self.lastVel_F<-0.5 :
            self.msg.vel_N=-0.5+self.lastVel_N;

            if self.msg.vel_w - self.lastVel_W>0.5 :
                self.msg.vel_w=0.5+self.lastVel_W;
        elif self.msg.vel_w - self.lastVel_W<-0.5 :
            self.msg.vel_w=-0.5+self.lastVel_W;



            self.lastVel_W=self.msg.vel_w;
            self.lastVel_F=self.msg.vel_F;
            self.lastVel_N=self.msg.vel_N;



    def jacobian(self, vx, vy, w):
        robotRadius = 0.0795
        wheelRadius = 0.0275
        _DEG2RAD = np.pi/180
        _BIT_RESOLUTION = 127
        motorAlpha = [60.0 * _DEG2RAD, 135.0 * _DEG2RAD, 225.0 * _DEG2RAD, 300 * _DEG2RAD]
        dw1 = (1.0 / wheelRadius) * ((robotRadius * w) - (vx * np.sin(motorAlpha[0])) + (vy * np.cos(motorAlpha[0])))
        dw2 = (1.0 / wheelRadius) * ((robotRadius * w) - (vx * np.sin(motorAlpha[1])) + (vy * np.cos(motorAlpha[1])))
        dw3 = (1.0 / wheelRadius) * ((robotRadius * w) - (vx * np.sin(motorAlpha[2])) + (vy * np.cos(motorAlpha[2])))
        dw4 = (1.0 / wheelRadius) * ((robotRadius * w) - (vx * np.sin(motorAlpha[3])) + (vy * np.cos(motorAlpha[3])))
        motorMaxRadPerSec = 1000 * 2 * np.pi / float(60.0)
        dw1 = (dw1 / (motorMaxRadPerSec)) * _BIT_RESOLUTION
        dw2 = (dw2 / (motorMaxRadPerSec)) * _BIT_RESOLUTION
        dw3 = (dw3 / (motorMaxRadPerSec)) * _BIT_RESOLUTION
        dw4 = (dw4 / (motorMaxRadPerSec)) * _BIT_RESOLUTION
        self.v1 = (round(dw1))
        self.v2 = (round(dw2))
        self.v3 = (round(dw3))
        self.v4 = (round(dw4))

    def TimerCallback(self, event):
        pwm = parsian_world_model()
        self.m.publish(pwm)
        self.p.publish(self.msg)

    def MovingRobot(self, data):
        #rospy.loginfo(data.buttons)
        #rospy.loginfo(data.axes)

        # if data.buttons[0] == 1:
        #     rospy.loginfo("1 is pressed")
        #     self.jacobian(0.0, 0.0, 0.0)                 ############# STOP #############
        # elif data.buttons[1] == 1:
        #     rospy.loginfo("2 is pressed")
        # elif data.buttons[2] == 1:
        #     rospy.loginfo("3 is pressed")
        # elif data.buttons[3] == 1:
        #     rospy.loginfo("4 is pressed")
        #     #self.msg.kickSpeed = 250

        if self.msg.vel_F - self.lastVel_F>0 :
            self.msg.vel_F=0.8+self.lastVel_F;
        elif self.msg.vel_F - self.lastVel_F<-0.8 :
            self.msg.vel_F=-0.8+self.lastVel_F;

            if self.msg.vel_N - self.lastVel_N>0.8 :
                self.msg.vel_N=0.8+self.lastVel_N;
        elif self.msg.vel_N - self.lastVel_F<-0.8 :
            self.msg.vel_N=-0.8+self.lastVel_N;

            if self.msg.vel_w - self.lastVel_W>100 :
                self.msg.vel_w=0.8+self.lastVel_W;
        elif self.msg.vel_w - self.lastVel_W<-100 :
            self.msg.vel_w=-0.8+self.lastVel_W;


        if data.buttons[4] == 1:
            rospy.loginfo("L1 is pressed")
            self.msg.vel_w = 210
            self.jacobian(0.0, 0.0, speed / 20.0)         ############ CCW ##############
        elif data.buttons[5] == 1:
            rospy.loginfo("R1 is pressed")
            self.msg.vel_w = -210
            self.jacobian(0.0, 0.0, -speed / 20.0)        ############ CW ###############
        elif data.buttons[6] == 1:
            rospy.loginfo("L2 is pressed")
            self.msg.vel_w = 100
        elif data.buttons[7] == 1:
            rospy.loginfo("R2 is pressed")
            self.msg.vel_w = -100
        elif data.axes[0] == 1.0 and -1 < data.axes[1] < 1 and data.axes[2] == 1.0:
            rospy.loginfo("Left Dual Shock")
            if data.buttons[1] == 1:
                self.msg.vel_N = RobotSpeed * minSpeed
            elif data.buttons[2] == 1:
                self.msg.vel_N = RobotSpeed * maxSpeed
            elif data.buttons[3] == 1:
                self.msg.vel_N = RobotSpeed * Shooting
            else:
                self.msg.vel_N = RobotSpeed
                self.jacobian(0.0, speed * 4.2 / 127.0, 0.0)
        elif data.axes[0] == 1.0 and data.axes[1] == 1 and 0.7 < data.axes[2] <= 1:
            rospy.loginfo("Left Up Dual Shock")
            if data.buttons[1] == 1:
                self.msg.vel_N = RobotSpeed * minSpeed
                self.msg.vel_F = RobotSpeed * minSpeed
            elif data.buttons[2] == 1:
                self.msg.vel_N = RobotSpeed * maxSpeed
                self.msg.vel_F = RobotSpeed * maxSpeed
            elif data.buttons[3] == 1:
                self.msg.vel_N = RobotSpeed * Shooting
                self.msg.vel_F = RobotSpeed * Shooting
            else:
                self.msg.vel_N = RobotSpeed
                self.msg.vel_F = RobotSpeed
                self.jacobian(speed * 4.2 / 127.0, speed * 4.2 / 127.0, 0.0)
        elif -1 < data.axes[0] < 1 and data.axes[1] == 1 and -1 < data.axes[2] < 1:
            rospy.loginfo("Up Dual Shock")
            if data.buttons[1] == 1:
                self.msg.vel_F = RobotSpeed * minSpeed
            elif data.buttons[2] == 1:
                self.msg.vel_F = RobotSpeed * maxSpeed
            elif data.buttons[3] == 1:
                self.msg.vel_F = RobotSpeed * Shooting
            else:
                self.msg.vel_F = RobotSpeed
                self.jacobian(speed * 4.2 / 127.0, 0.0, 0.0)
        elif data.axes[0] == -1.0 and data.axes[1] == 1 and -1 <= data.axes[2] < -0.5:
            rospy.loginfo("Right Up Dual Shock")
            if data.buttons[1] == 1:
                self.msg.vel_F = RobotSpeed * minSpeed
                self.msg.vel_N = -RobotSpeed * minSpeed
            elif data.buttons[2] == 1:
                self.msg.vel_F = RobotSpeed * maxSpeed
                self.msg.vel_N = -RobotSpeed * maxSpeed
            elif data.buttons[3] == 1:
                self.msg.vel_F = RobotSpeed * Shooting
                self.msg.vel_N = -RobotSpeed * Shooting
            else:
                self.msg.vel_F = RobotSpeed
                self.msg.vel_N = -RobotSpeed
                self.jacobian(speed * 4.2 / 127.0, -speed * 4.2 / 127.0, 0.0)
        elif data.axes[0] == -1.0 and -0.5 < data.axes[1] < 1 and data.axes[2] == -1.0:
            rospy.loginfo("Right Dual Shock")
            if data.buttons[1] == 1:
                self.msg.vel_N = -RobotSpeed * minSpeed
            elif data.buttons[2] == 1:
                self.msg.vel_N = -RobotSpeed * maxSpeed
            elif data.buttons[3] == 1:
                self.msg.vel_N = -RobotSpeed * Shooting
            else:
                self.msg.vel_N = -RobotSpeed
                self.jacobian(0.0, -speed * 4.2 / 127.0, 0.0)
        elif data.axes[0] == -1.0 and -1 <= data.axes[1] < -0.5 and -1 <= data.axes[2] < -0.5:
            rospy.loginfo("Right Down Dual Shock")
            if data.buttons[1] == 1:
                self.msg.vel_N = -RobotSpeed * minSpeed
                self.msg.vel_F = -RobotSpeed * minSpeed
            elif data.buttons[2] == 1:
                self.msg.vel_N = -RobotSpeed * maxSpeed
                self.msg.vel_F = -RobotSpeed * maxSpeed
            elif data.buttons[3] == 1:
                self.msg.vel_N = -RobotSpeed * Shooting
                self.msg.vel_F = -RobotSpeed * Shooting
            else:
                self.msg.vel_N = -RobotSpeed
                self.msg.vel_F = -RobotSpeed
                self.jacobian(-speed * 4.2 / 127.0, -speed * 4.2 / 127.0, 0.0)
        elif -1 < data.axes[0] < 1 and data.axes[1] == -1 and -1 < data.axes[2] < 1:
            rospy.loginfo("Down Dual Shock")
            if data.buttons[1] == 1:
                self.msg.vel_F = -RobotSpeed * minSpeed
            elif data.buttons[2] == 1:
                self.msg.vel_F = -RobotSpeed * maxSpeed
            elif data.buttons[3] == 1:
                self.msg.vel_F = -RobotSpeed * Shooting
            else:
                self.msg.vel_F = -RobotSpeed
                self.jacobian(-speed * 4.2 / 127.0, 0.0, 0.0)
        elif data.axes[0] == 1 and data.axes[1] == -1 and data.axes[2] == 1.0:
            rospy.loginfo("Left Down Dual Shock")
            if data.buttons[1] == 1:
                self.msg.vel_F = -RobotSpeed * minSpeed
                self.msg.vel_N = RobotSpeed * minSpeed
            elif data.buttons[2] == 1:
                self.msg.vel_F = -RobotSpeed * maxSpeed
                self.msg.vel_N = RobotSpeed * maxSpeed
            elif data.buttons[3] == 1:
                self.msg.vel_F = -RobotSpeed * Shooting
                self.msg.vel_N = RobotSpeed * Shooting
            else:
                self.msg.vel_F = -RobotSpeed
                self.msg.vel_N = RobotSpeed
                self.jacobian(-speed * 4.2 / 127.0, speed * 4.2 / 127.0, 0.0)

        elif data.axes[6] == 1.0:
            rospy.loginfo("up is pressed")
            if data.buttons[1] == 1:
                self.msg.vel_F = RobotSpeed * minSpeed
            elif data.buttons[2] == 1:
                self.msg.vel_F = RobotSpeed * maxSpeed
            elif data.buttons[3] == 1:
                self.msg.vel_F = RobotSpeed * Shooting
            else:
                self.msg.vel_F = RobotSpeed
                self.jacobian(speed * 4.2 / 127.0, 0.0, 0.0)
        elif data.axes[5] == -1.0:
            rospy.loginfo("Right is pressed")
            if data.buttons[1] == 1:
                self.msg.vel_N = -RobotSpeed * minSpeed
            elif data.buttons[2] == 1:
                self.msg.vel_N = -RobotSpeed * maxSpeed
            elif data.buttons[3] == 1:
                self.msg.vel_N = -RobotSpeed * Shooting
            else:
                self.msg.vel_N = -RobotSpeed
                self.jacobian(0.0, -speed * 4.2 / 127.0, 0.0)
        elif data.axes[6] == -1.0:
            rospy.loginfo("Down is pressed")
            if data.buttons[1] == 1:
                self.msg.vel_F = -RobotSpeed * minSpeed
            elif data.buttons[2] == 1:
                self.msg.vel_F = -RobotSpeed * maxSpeed
            elif data.buttons[3] == 1:
                self.msg.vel_F = -RobotSpeed * Shooting
            else:
                self.msg.vel_F = -RobotSpeed
                self.jacobian(-speed * 4.2 / 127.0, 0.0, 0.0)
        elif data.axes[5] == 1.0:
            rospy.loginfo("Left is pressed")
            if data.buttons[1] == 1:
                self.msg.vel_N = RobotSpeed * minSpeed
            elif data.buttons[2] == 1:
                self.msg.vel_N = RobotSpeed * maxSpeed
            elif data.buttons[3] == 1:
                self.msg.vel_N = RobotSpeed * Shooting
            else:
                self.msg.vel_N = RobotSpeed
                self.jacobian(0.0, speed * 4.2 / 127.0, 0.0)
        else:
            self.msg.vel_F = 0
            self.msg.vel_N = 0
            self.msg.vel_w = 0
            self.msg.kickSpeed = 0
            self.jacobian(0.0, 0.0, 0.0)                       ##### FOR GRSIM #########

        button = data.buttons
        self.msg.wheel1 = self.v1
        self.msg.wheel2 = self.v2
        self.msg.wheel3 = self.v3
        self.msg.wheel4 = self.v4

        rospy.loginfo("Velocity of wheel 1 is:{}".format(self.v1))
        rospy.loginfo("Velocity of wheel 2 is:{}".format(self.v2))
        rospy.loginfo("Velocity of wheel 3 is:{}".format(self.v3))
        rospy.loginfo("Velocity of wheel 4 is:{}".format(self.v4))
        rospy.loginfo("Shoot Velocity is : {}".format(self.msg.kickSpeed))
        rospy.loginfo("--------------------------")
        self.lastVel_F=self.msg.vel_F;
        self.lastVel_N=self.msg.vel_N;
        self.lastVel_W=self.msg.vel_w;



if __name__ == '__main__':
    try:
        MotionProfilerNode()
    except rospy.ROSInterruptException:
        pass




