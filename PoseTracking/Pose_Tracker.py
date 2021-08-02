from mpl_toolkits import mplot3d
import mediapipe as mp
import numpy as np
from enum import Enum
import serial
import sys
import time
import cv2 as cv2

import numpy as np
import matplotlib.pyplot as plt

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
fig = plt.figure()

# # port = "/dev/ttyACM0"
# port = "COM4"
# arduino = serial.Serial(
#     port, 9600, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE
# )  # initialise serial object

joint_array = [
    [11, 13, 15],  # Left arm bicep joint
    [12, 14, 16],  # Right arm bicep joint
]
joint_angles = np.zeros((2, len(joint_array)))  # Initialise joint angles array (2x10)


class Display(Enum):
    ClearAll = 0
    IndexOnly = 1
    ViewAll = 2


currentDisplay = (
    Display.ViewAll
)  # Press 't' to toggle which angle values to draw on camera image
communication = False  # Press 'c' to toggle communication with arduino


def calc_angle(u, v):
    """Function that calculates the angle between two vectors"""
    return round(np.arccos(u.dot(v) / (np.linalg.norm(u) * np.linalg.norm(v))), 2)


def lin2interp(a, b, ratio):
    """
    Function that interpolates between two inputs (a < b) to return x
        ratio = (x-a)/(b-a)
    """
    return int((b - a) * ratio + a)


def calculateAngle(landmarks, landmark1, landmark2, landmark3):
    # [[0, 0, 0], [0, 0, 1], [0, 1, 1]]
    pos1 = np.array(
        [landmarks[landmark1].x, landmarks[landmark1].y, landmarks[landmark1].z]
    )
    pos2 = np.array(
        [landmarks[landmark2].x, landmarks[landmark2].y, landmarks[landmark2].z]
    )
    pos3 = np.array(
        [landmarks[landmark3].x, landmarks[landmark3].y, landmarks[landmark3].z]
    )
    print(f"Pos1: {pos1}")
    print(f"Pos2: {pos2}")
    print(f"Pos2: {pos3}")
    # pos1 = np.array(landmarks[0])
    # pos2 = np.array(landmarks[1])
    # pos3 = np.array(landmarks[2])
    OneToTwo = pos2 - pos1
    TwoToThree = pos3 - pos2
    print(f"Vector 1 to 2: {OneToTwo}")
    print(f"Vector 2 to 3: {TwoToThree}")
    return np.rad2deg(calc_angle(TwoToThree, OneToTwo))


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

        # Flip the image horizontally for a later selfie-view display, and convert
        # the BGR image to RGB.
        image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        results = pose.process(image)
        landmarks = results.pose_world_landmarks.landmark
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
        plt.pause(0.05)
        print(mp_pose.PoseLandmark.LEFT_SHOULDER.value)
        print("Left Positions")
        leftElbowAngle = calculateAngle(landmarks, 11, 13, 15)
        cv2.putText(
            image,
            str(leftElbowAngle),
            tuple(
                np.multiply(
                    np.array([landmarks[13].x, landmarks[13].y]), [640, 480]
                ).astype(int)
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        print("Right Positions")
        rightElbowAngle = calculateAngle(landmarks, 12, 14, 16)
        cv2.putText(
            image,
            str(rightElbowAngle),
            tuple(
                np.multiply(
                    np.array([landmarks[14].x, landmarks[14].y]), [640, 480]
                ).astype(int)
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        print(f"L: {leftElbowAngle} R: {rightElbowAngle}")
        # Calculate the plane of the body
        pos1 = np.array([landmarks[12].x, landmarks[12].y, landmarks[12].z])
        pos2 = np.array([landmarks[11].x, landmarks[11].y, landmarks[11].z])
        pos3 = np.array([landmarks[23].x, landmarks[23].y, landmarks[23].z])

        normal = np.cross((pos1 - pos2), (pos3 - pos2))

        topRightArmVector = (
            np.array([landmarks[14].x, landmarks[14].y, landmarks[14].z]) - pos1
        )
        angleAtRightArm = calc_angle(normal, topRightArmVector)
        cv2.putText(
            image,
            str(angleAtRightArm),
            tuple(
                np.multiply(
                    np.array([landmarks[12].x, landmarks[12].y]), [640, 480]
                ).astype(int)
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        # Draw the pose annotation on the image.
        image.flags.writeable = True

        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        mp_drawing.draw_landmarks(
            image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS
        )
        cv2.imshow("MediaPipe Pose", image)
        if cv2.waitKey(5) & 0xFF == 27:
            break
cap.release()
plt.show()
