#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from mpl_toolkits import mplot3d
import mediapipe as mp
import numpy as np
from enum import Enum
import cv2 as cv2
import math
import numpy as np
import matplotlib.pyplot as plt

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
fig = plt.figure()

showGraph = True

rospy.init_node('pose_tracker', anonymous=True)

left_elbow_publisher = rospy.Publisher('left_elbow', Float64, queue_size=10)
left_shoulder_publisher = rospy.Publisher('left_shoulder', Float64, queue_size=10)
left_omoplate_publisher = rospy.Publisher('left_omoplate', Float64, queue_size=10)
left_rotation_publisher = rospy.Publisher('left_rotation', Float64, queue_size=10)

right_elbow_publisher = rospy.Publisher('right_elbow', Float64, queue_size=10)
right_shoulder_publisher = rospy.Publisher('right_shoulder', Float64, queue_size=10)
right_omoplate_publisher = rospy.Publisher('right_omoplate', Float64, queue_size=10)
right_rotation_publisher = rospy.Publisher('right_rotation', Float64, queue_size=10)

rate = rospy.Rate(10) #This keeps it running at 10Hz, this will probably want changing

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


def calculateAngle(landmarks, landmark1, landmark2, landmark3):
    pos1 = np.array(
        [landmarks[landmark1].x, landmarks[landmark1].y, landmarks[landmark1].z]
    )
    pos2 = np.array(
        [landmarks[landmark2].x, landmarks[landmark2].y, landmarks[landmark2].z]
    )
    pos3 = np.array(
        [landmarks[landmark3].x, landmarks[landmark3].y, landmarks[landmark3].z]
    )
    OneToTwo = pos2 - pos1
    TwoToThree = pos3 - pos2
    return calc_angle(TwoToThree, OneToTwo)


def projectToPlane(normal, vector):
    # I think I can do the vector minus the bit of the vector in the direction of the normal
    return vector - (normalise(normal) * np.dot(normalise(normal), vector))

if showGraph:
    fig = plt.figure()
    ax = plt.axes(projection="3d")

# # For webcam input:
cap = cv2.VideoCapture(0)
with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            # If loading a video, use 'break' instead of 'continue'.
            continue
        try:
            # Flip the image horizontally for a later selfie-view display, and convert
            # the BGR image to RGB.
            image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
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

            leftElbowAngle = calc_angle(topLeftArmVector, leftWristPosition - leftElbowPosition)
            rightElbowAngle = calc_angle(topRightArmVector, leftWristPosition - leftElbowPosition)
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
            leftArmRotationAngle = calc_angle(normalise(topLeftArmAndBodyNormal), leftWristPosition - leftElbowPosition)

            topRightArmAndBodyNormal = np.cross(topRightArmVector, rightBodySideVector)
            rightArmRotationAngle = calc_angle(normalise(topRightArmAndBodyNormal),rightWristPosition - rightElbowPosition)
            #It will be left, before right.  Then and it is elbow, shoulder, omo, rotation
            outputArray = np.array(list(map(math.trunc, map(math.degrees,[leftElbowAngle, angleAtLeftArm, angleLeftArmOmoPlate, leftArmRotationAngle,
                                    rightElbowAngle, angleAtRightArm, angleRightArmOmoPlate, rightArmRotationAngle]))))
            rospy.loginfo(outputArray)
            left_elbow_publisher.publish(outputArray[0])
            left_shoulder_publisher.publish(outputArray[1])
            left_omoplate_publisher.publish(outputArray[2])
            left_rotation_publisher.publish(outputArray[3])
            right_elbow_publisher.publish(outputArray[4])
            right_shoulder_publisher.publish(outputArray[5])
            right_omoplate_publisher.publish(outputArray[6])
            right_rotation_publisher.publish(outputArray[7])
            # Draw the pose annotation on the image.
            image.flags.writeable = True

            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            mp_drawing.draw_landmarks(
                image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS
            )
            cv2.imshow("MediaPipe Pose", image)
            if cv2.waitKey(5) & 0xFF == 27:
                break
        except:
            continue
        rate.sleep()

cap.release()
plt.show()