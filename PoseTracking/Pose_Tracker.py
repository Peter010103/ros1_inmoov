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

left_arm_publisher = rospy.Publisher(
    '/joints/arm/left', Int16MultiArray, queue_size=10)
right_arm_publisher = rospy.Publisher(
    '/joints/arm/right', Int16MultiArray, queue_size=10)

# This keeps it running at 10Hz, this will probably want changing
rate = rospy.Rate(10)

if showGraph:
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    plt.show()


def calc_angle(u, v):
    """Function that calculates the angle between two vectors"""
    return round(np.arccos(u.dot(v) / (np.linalg.norm(u) * np.linalg.norm(v))), 2)


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
        #Converts the image from a ROS image to one suitable for processing
        image = bridge.imgmsg_to_cv2(data, "bgr8")
        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        results = pose.process(image)
        #This extracts the landmarks we are interested in
        landmarks = results.pose_world_landmarks.landmark
        # pictureLandmarks = results.pose_landmarks.landmark
        #Displays the data on a 3d graph to allow manual inspection to see what is being picked up
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

        # Extracts all of the positions we are interested in from the landmarks array for further calculation
        leftElbowPosition = np.array([landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].x,
                                     landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].y, landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].z])
        rightElbowPosition = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].x,
                                      landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].y, landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].z])
        leftWristPosition = np.array([landmarks[mp_pose.PoseLandmark.LEFT_WRIST].x,
                                     landmarks[mp_pose.PoseLandmark.LEFT_WRIST].y, landmarks[mp_pose.PoseLandmark.LEFT_WRIST].z])
        rightWristPosition = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].x,
                                      landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].y, landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].z])
        leftShoulder = np.array([landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].x,
                                landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].y, landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].z])
        rightShoulder = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].x,
                                 landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].y, landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].z])
        leftHip = np.array([landmarks[mp_pose.PoseLandmark.LEFT_HIP].x,
                           landmarks[mp_pose.PoseLandmark.LEFT_HIP].y, landmarks[mp_pose.PoseLandmark.LEFT_HIP].z])
        rightHip = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_HIP].x,
                            landmarks[mp_pose.PoseLandmark.RIGHT_HIP].y, landmarks[mp_pose.PoseLandmark.RIGHT_HIP].z])

        # This calculates the vectors which go along your arm
        topLeftArmVector = leftElbowPosition - leftShoulder
        topRightArmVector = rightElbowPosition - rightShoulder
        lowLeftArmVector = leftWristPosition - leftElbowPosition
        lowRightArmVector = rightWristPosition - rightElbowPosition

        # Calculates the angle at your elbow
        leftElbowAngle = calc_angle(topLeftArmVector, lowLeftArmVector)
        rightElbowAngle = calc_angle(topRightArmVector, lowRightArmVector)

        # Calculate the plane of the body, because we know it has the following normal vector
        bodyNormal = np.cross(
            (rightShoulder - leftShoulder), (leftHip - leftShoulder))

        # Calculate the angle at your shoulder
        angleAtLeftArm = calc_angle(bodyNormal, topLeftArmVector)
        angleAtRightArm = calc_angle(bodyNormal, topRightArmVector)

        # Calculate the vectors which are going down your body
        leftBodySideVector = leftShoulder - leftHip
        rightBodySideVector = rightShoulder - rightHip

        # This puts your arms into the same plane as the body so that it can work out how out to the side your arms are
        leftArmVectorProjectedToPlane = projectToPlane(
            bodyNormal, topLeftArmVector)
        angleLeftArmOmoPlate = calc_angle(
            leftArmVectorProjectedToPlane, leftBodySideVector)

        rightArmVectorProjectedToPlane = projectToPlane(
            bodyNormal, topRightArmVector)
        angleRightArmOmoPlate = calc_angle(
            rightArmVectorProjectedToPlane, rightBodySideVector
        )

        #This attempts to calculate how rotated your arm is by taking a plane between your body 
        #side and the top of your arm and looking at the angle this makes with your lower arm
        topLeftArmAndBodyNormal = np.cross(
            topLeftArmVector, leftBodySideVector)
        leftArmRotationAngle = calc_angle(
            normalise(topLeftArmAndBodyNormal), lowLeftArmVector)

        topRightArmAndBodyNormal = np.cross(
            topRightArmVector, rightBodySideVector)
        rightArmRotationAngle = calc_angle(
            normalise(topRightArmAndBodyNormal), lowRightArmVector)
        # It will be left, before right.  Then and it is elbow, shoulder, omo, rotation
        outputArray = np.array(list(map(math.trunc, map(math.degrees, [leftElbowAngle, leftArmRotationAngle, angleAtLeftArm, angleLeftArmOmoPlate,
                                                                       rightElbowAngle, rightArmRotationAngle, angleAtRightArm, angleRightArmOmoPlate]))))
        #Logs the info to the terminal and publishes it to the topics so that other nodes can receive it
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

#Standard setup for ROS so that we can listen for an incoming message on a topic, and call our function to deal with the input
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
