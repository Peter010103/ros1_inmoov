#!/usr/bin/env python
import yaml
import os
import rospy
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge, CvBridgeError
from adafruit_servokit import ServoKit


class Joints(Enum):
    R_WRIST = 0
    R_THUMB = 1
    R_INDEX = 2
    R_MIDDLE = 3
    R_RINGFINGER = 4
    R_PINKY = 5
    R_BICEP = 6
    R_ARM_ROTATE = 7
    R_SHOULDER = 8
    R_OMOPLATE = 9
    L_THUMB = 10
    L_INDEX = 11
    L_MIDDLE = 12
    L_RINGFINGER = 13
    L_PINKY = 14
    L_WRIST = 15
    L_BICEP = 16
    L_ARM_ROTATE = 17
    L_SHOULDER = 18
    L_OMOPLATE = 19
    NECK = 20
    ROTHEAD = 21
    ROLLNECK = 22
    EYELID = 23
    JAW = 24
    EYEX = 25
    EYEY = 26
    TOPSTOM = 27
    MIDSTOM = 28
    LOWSTOM = 29


kit = ServoKit(channels=16)

servoNamesToIndices = {}
with open(r'jointNamesToServoNumber.yml') as inputFile:
    servoNamesToIndices = yaml.full_load(inputFile)


def moveToAngle(joint, angle):
    kit.servo[servoNamesToIndices[joint]].angle = angle


def callbackData(data):
    moveToAngle('R_WRIST', data.data[Joints.R_WRIST])
    moveToAngle('R_ARM_ROTATE', data.data[Joints.R_ARM_ROTATE])
    moveToAngle('R_BICEP', data.data[Joints.R_BICEP])
    moveToAngle('R_OMOPLATE', data.data[Joints.R_OMOPLATE])
    moveToAngle('R_SHOULDER', data.data[Joints.R_SHOULDER])
    moveToAngle('L_WRIST', data.data[Joints.L_WRIST])
    moveToAngle('L_ARM_ROTATE', data.data[Joints.L_ARM_ROTATE])
    moveToAngle('L_BICEP', data.data[Joints.L_BICEP])
    moveToAngle('L_OMOPLATE', data.data[Joints.L_OMOPLATE])
    moveToAngle('L_SHOULDER', data.data[Joints.L_SHOULDER])


if __name__ == "__main__":
    rospy.init_node('i2c_serial', anonymous=True)
    image_sub = rospy.Subscriber(
        '/servo_angles', Int16MultiArray, callbackData)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
