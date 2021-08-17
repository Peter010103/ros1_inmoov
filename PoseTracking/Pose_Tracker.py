from mpl_toolkits import mplot3d
import mediapipe as mp
import numpy as np
from enum import Enum
import serial
import sys
import time
import cv2 as cv2
import math
import numpy as np
import matplotlib.pyplot as plt

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
fig = plt.figure()

communication = False  # Press 'c' to toggle communication with arduino
showGraph = True

if communication:
    # port = "/dev/ttyACM0"
    port = "COM4"
    arduino = serial.Serial(
        port, 9600, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE
    )  # initialise serial object

joint_array = [
    [11, 13, 15],  # Left arm bicep joint
    [12, 14, 16],  # Right arm bicep joint
]
# Initialise joint angles array (2x10)
joint_angles = np.zeros((2, len(joint_array)))


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

def send2arduino(servo_output):
    """Function that converts the servo outputs to a bytearray, which is sent to the arduino"""
    servo_data = bytearray(np.uint8(servo_output.ravel()))
    servo_data.append(255)
    # print(servo_data)
    arduino.write(servo_data)

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
            leftElbowAngle = calculateAngle(landmarks, 11, 13, 15)
            rightElbowAngle = calculateAngle(landmarks, 12, 14, 16)
            #  print(f"L: {leftElbowAngle} R: {rightElbowAngle}")
            # Calculate the plane of the body
            rightShoulder = np.array([landmarks[12].x, landmarks[12].y, landmarks[12].z])
            leftShoulder = np.array([landmarks[11].x, landmarks[11].y, landmarks[11].z])
            leftHip = np.array([landmarks[23].x, landmarks[23].y, landmarks[23].z])

            bodyNormal = np.cross((rightShoulder - leftShoulder), (leftHip - leftShoulder))

            topLeftArmVector = np.array(
                [landmarks[13].x, landmarks[13].y, landmarks[13].z]) - leftShoulder
            topRightArmVector = (
                np.array([landmarks[14].x, landmarks[14].y, landmarks[14].z]) - rightShoulder
            )

            leftBodySideVector = leftShoulder - leftHip
            rightBodySideVector = rightShoulder - np.array(
                [landmarks[24].x, landmarks[24].y, landmarks[24].z]
            )
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
            leftArmRotationAngle = calc_angle(normalise(topLeftArmAndBodyNormal), np.array([
                landmarks[15].x, landmarks[15].y, landmarks[15].z]) - np.array(
                [landmarks[13].x, landmarks[13].y, landmarks[13].z])
            )

            topRightArmAndBodyNormal = np.cross(
                topRightArmVector, rightBodySideVector)
            rightArmRotationAngle = calc_angle(
                normalise(topRightArmAndBodyNormal),
                np.array([landmarks[16].x, landmarks[16].y, landmarks[16].z])
                - np.array([landmarks[14].x, landmarks[14].y, landmarks[14].z]),
            )
            #It will be left, before right.  Then and it is elbow, shoulder, omo, rotation
            outputArray = np.array(list(map(math.trunc, map(math.degrees,[leftElbowAngle, angleAtLeftArm, angleLeftArmOmoPlate, leftArmRotationAngle,
                                    rightElbowAngle, angleAtRightArm, angleRightArmOmoPlate, rightArmRotationAngle]))))
            print(outputArray)
            # Draw the pose annotation on the image.
            image.flags.writeable = True

            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            mp_drawing.draw_landmarks(
                image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS
            )
            if communication:
                send2arduino(outputArray)
            cv2.imshow("MediaPipe Pose", image)
            if cv2.waitKey(5) & 0xFF == 27:
                break
        except:
            continue

cap.release()
plt.show()
