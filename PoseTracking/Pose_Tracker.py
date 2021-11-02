#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from mpl_toolkits import mplot3d
import mediapipe as mp
import numpy as np
from enum import Enum
import cv2 as cv2
import math
import matplotlib.pyplot as plt

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
fig = plt.figure()
bridge = CvBridge()

showGraph = True

left_arm_publisher = rospy.Publisher('/joints/arm/left', Int16MultiArray, queue_size=10 )
right_arm_publisher = rospy.Publisher('/joints/arm/right', Int16MultiArray, queue_size=10 )

rate = rospy.Rate(10) #This keeps it running at 10Hz, this will probably want changing

if showGraph:
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    plt.show()

class Display(Enum):
    ClearAll = 0
    IndexOnly = 1
    ViewAll = 2

currentDisplay = (
    Display.ViewAll
)  # Press 't' to toggle which angle values to draw on camera image

def calc_angle(u, v):
    """Function that calculates the angle between two vectors"""
    return round(np.arccos(u.dot(v) / (np.linalg.norm(u) * np.linalg.norm(v))), 2)

def lin2interp(a, b, ratio):
    """
    Function that interpolates between two inputs (a < b) to return x
        ratio = (x-a)/(b-a)
    """
    return int((b - a) * ratio + a)

def normalise(vector):
    magnitude = (vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2) ** 0.5
    return np.array(
        [vector[0] / magnitude, vector[1] / magnitude, vector[2] / magnitude]
    )

def projectToPlane(normal, vector):
    # I think I can do the vector minus the bit of the vector in the direction of the normal
    return vector - (normalise(normal) * np.dot(normalise(normal), vector))

def frameCallback(data):
    with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
        # Flip the image horizontally for a later selfie-view display, and convert
        # the BGR image to RGB.
        image = bridge.imgmsg_to_cv2(data, "bgr8")
        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        results = pose.process(image)
        landmarks = results.pose_world_landmarks.landmark
        pictureLandmarks = results.pose_landmarks.landmark
        if showGraph:
            xData = []
            yData = []
            zData = []

            for landmark in landmarks:
                xData.append(landmark.x)
                yData.append(landmark.y)
                zData.append(landmark.z)
            ax.cla()
            ax.scatter3D(
                xData,
                zData,
                yData,
            )
            ax.set_xlim(-1, 1)
            ax.set_ylim(-2, 2)
            ax.set_zlim(-1, 1)
            ax.set_xlabel("x")
            ax.set_zlabel("y")
            ax.set_ylabel("z")
            plt.pause(0.02)

        leftElbowPosition = np.array([landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].x, landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].y, landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].z])
        rightElbowPosition = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].x, landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].y, landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].z])
        leftWristPosition =  np.array([landmarks[mp_pose.PoseLandmark.LEFT_WRIST].x, landmarks[mp_pose.PoseLandmark.LEFT_WRIST].y, landmarks[mp_pose.PoseLandmark.LEFT_WRIST].z])
        rightWristPosition = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].x, landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].y, landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].z])
        leftShoulder = np.array([landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].x, landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].y, landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].z])
        rightShoulder = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].x, landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].y, landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].z])
        leftHip = np.array([landmarks[mp_pose.PoseLandmark.LEFT_HIP].x, landmarks[mp_pose.PoseLandmark.LEFT_HIP].y, landmarks[mp_pose.PoseLandmark.LEFT_HIP].z])
        rightHip = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_HIP].x, landmarks[mp_pose.PoseLandmark.RIGHT_HIP].y, landmarks[mp_pose.PoseLandmark.RIGHT_HIP].z])

        topLeftArmVector = leftElbowPosition - leftShoulder
        topRightArmVector = rightElbowPosition - rightShoulder
        lowLeftArmVector = leftWristPosition - leftElbowPosition
        lowRightArmVector = rightWristPosition - rightElbowPosition

        leftElbowAngle = calc_angle(topLeftArmVector, lowLeftArmVector)
        rightElbowAngle = calc_angle(topRightArmVector, lowRightArmVector)
        # Calculate the plane of the body
        bodyNormal = np.cross((rightShoulder - leftShoulder), (leftHip - leftShoulder))

        leftBodySideVector = leftShoulder - leftHip
        rightBodySideVector = rightShoulder - rightHip
        angleAtLeftArm = calc_angle(bodyNormal, topLeftArmVector)
        angleAtRightArm = calc_angle(bodyNormal, topRightArmVector)

        leftArmVectorProjectedToPlane = projectToPlane(bodyNormal, topLeftArmVector)
        angleLeftArmOmoPlate = calc_angle(leftArmVectorProjectedToPlane, leftBodySideVector)

        rightArmVectorProjectedToPlane = projectToPlane(
            bodyNormal, topRightArmVector)
        angleRightArmOmoPlate = calc_angle(
            rightArmVectorProjectedToPlane, rightBodySideVector
        )

        topLeftArmAndBodyNormal = np.cross(topLeftArmVector, leftBodySideVector)
        leftArmRotationAngle = calc_angle(normalise(topLeftArmAndBodyNormal), lowLeftArmVector)

        topRightArmAndBodyNormal = np.cross(topRightArmVector, rightBodySideVector)
        rightArmRotationAngle = calc_angle(normalise(topRightArmAndBodyNormal), lowRightArmVector)
        #It will be left, before right.  Then and it is elbow, shoulder, omo, rotation
        outputArray = np.array(list(map(math.trunc, map(math.degrees,[leftElbowAngle,leftArmRotationAngle, angleAtLeftArm, angleLeftArmOmoPlate,
                                rightElbowAngle,rightArmRotationAngle, angleAtRightArm, angleRightArmOmoPlate]))))
        rospy.loginfo(outputArray)
        left_arm_publisher.publish(outputArray[0:4])
        right_arm_publisher.publish(outputArray[4:])
        # Draw the pose annotation on the image.
        image.flags.writeable = True

        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        mp_drawing.draw_landmarks(
            image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS
        )
        cv2.imshow("MediaPipe Pose", image)


def listener():
    rospy.init_node('/shadow_arm_controller', anonymous=True)
    image_sub = rospy.Subscriber('/camera/image_raw', Image, frameCallback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
    cv.destroyAllWindows()   

if __name__ == "__main__":
    listener()