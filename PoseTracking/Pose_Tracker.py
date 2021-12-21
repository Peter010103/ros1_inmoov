#!/usr/bin/env python
inRos = False
if inRos:
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
π = np.pi
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

showGraph = True
if showGraph:
    fig = plt.figure()
    ax = plt.axes(projection="3d")

if not inRos:
    class Display(Enum):
        ClearAll = 0
        IndexOnly = 1
        ViewAll = 2
    # This keeps it running at 10Hz, this will probably want changing
    currentDisplay = (
        Display.ViewAll
    )  # Press 't' to toggle which angle values to draw on camera image
else:
    bridge = CvBridge()
    left_arm_publisher = rospy.Publisher(
        '/joints/arm/left', Int16MultiArray, queue_size=10)
    right_arm_publisher = rospy.Publisher(
        '/joints/arm/right', Int16MultiArray, queue_size=10)
    rate = rospy.Rate(10)

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
        # Flip (do we really want to be doing this because I think it leads to the confusion with left and right arms) the image horizontally for a later selfie-view display, and convert
        # the BGR image to RGB.
        # Converts the image from a ROS image to one suitable for processing
        if inRos:
            image = bridge.imgmsg_to_cv2(data, "bgr8")
        else:
            image = data
        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        results = pose.process(image)
        # This extracts the landmarks we are interested in
        landmarks = results.pose_world_landmarks.landmark
        # pictureLandmarks = results.pose_landmarks.landmark
        # Displays the data on a 3d graph to allow manual inspection to see what is being picked up
        if showGraph:
            xData = []
            yData = []
            zData = []

            for landmark in landmarks:
                xData.append(landmark.x)
                yData.append(-landmark.y)
                zData.append(landmark.z)
            ax.cla()
            ax.scatter3D(
                xData,
                zData,
                yData,
            )
            ax.set_xlim(-1, 1)
            ax.set_ylim(-1, 1)
            ax.set_zlim(-1, 1)
            ax.set_xlabel("x")
            ax.set_zlabel("y")
            ax.set_ylabel("z")

        # Extracts all of the positions we are interested in from the landmarks array for further calculation
        leftElbowPosition = np.array([landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].x,
                                    -landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].y, landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].z])
        rightElbowPosition = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].x,
                                    -landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].y, landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].z])
        leftWristPosition = np.array([landmarks[mp_pose.PoseLandmark.LEFT_WRIST].x,
                                    -landmarks[mp_pose.PoseLandmark.LEFT_WRIST].y, landmarks[mp_pose.PoseLandmark.LEFT_WRIST].z])
        rightWristPosition = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].x,
                                    -landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].y, landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].z])
        leftShoulder = np.array([landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].x,
                                -landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].y, landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].z])
        rightShoulder = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].x,
                                -landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].y, landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].z])
        leftHip = np.array([landmarks[mp_pose.PoseLandmark.LEFT_HIP].x,
                            -landmarks[mp_pose.PoseLandmark.LEFT_HIP].y, landmarks[mp_pose.PoseLandmark.LEFT_HIP].z])
        rightHip = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_HIP].x,
                            -landmarks[mp_pose.PoseLandmark.RIGHT_HIP].y, landmarks[mp_pose.PoseLandmark.RIGHT_HIP].z])

        if showGraph:
            # This is where we want to display the points that we have picked out in colour
            X = np.column_stack((leftElbowPosition, rightElbowPosition, leftWristPosition,
                                rightWristPosition, leftShoulder, rightShoulder, leftHip, rightHip))
            # This is an array 3 long which has x,y,z data in that order
            ax.scatter3D(
                X[0],
                X[2],
                X[1],
                c='red'
            )

        # This calculates the vectors which go along your arm
        topLeftArmVector = leftElbowPosition - leftShoulder
        topRightArmVector = rightElbowPosition - rightShoulder
        lowLeftArmVector = leftWristPosition - leftElbowPosition
        lowRightArmVector = rightWristPosition - rightElbowPosition
        if showGraph:
            ax.quiver(leftShoulder[0], leftShoulder[2], leftShoulder[1],
                    topLeftArmVector[0], topLeftArmVector[2], topLeftArmVector[1], color='green')
            ax.quiver(rightShoulder[0], rightShoulder[2], rightShoulder[1],
                    topRightArmVector[0], topRightArmVector[2], topRightArmVector[1], color='red')
            ax.quiver(leftElbowPosition[0], leftElbowPosition[2], leftElbowPosition[1],
                    lowLeftArmVector[0], lowLeftArmVector[2], lowLeftArmVector[1], color='green')
            ax.quiver(rightElbowPosition[0], rightElbowPosition[2], rightElbowPosition[1],
                    lowRightArmVector[0], lowRightArmVector[2], lowRightArmVector[1], color='red')

        # Calculates the angle at your elbow
        # Default 0 (full extension), extended 0, flexed 90
        leftElbowAngle = calc_angle(topLeftArmVector, lowLeftArmVector)
        rightElbowAngle = calc_angle(topRightArmVector, lowRightArmVector)

        if showGraph:
            ax.text(leftElbowPosition[0],leftElbowPosition[2],leftElbowPosition[1], str(math.degrees(leftElbowAngle))[:4], color = 'green')
            ax.text(rightElbowPosition[0],rightElbowPosition[2],rightElbowPosition[1], str(math.degrees(rightElbowAngle))[:4], color = 'red')

        # Calculate the vectors which are going down your body
        leftBodySideVector = leftHip - leftShoulder
        rightBodySideVector = rightHip - rightShoulder
        if showGraph:
            ax.quiver(leftShoulder[0], leftShoulder[2], leftShoulder[1], leftBodySideVector[0],
                    leftBodySideVector[2], leftBodySideVector[1], color='green')
            ax.quiver(rightShoulder[0], rightShoulder[2], rightShoulder[1], rightBodySideVector[0],
                    rightBodySideVector[2], rightBodySideVector[1], color='red')

        # Calculate the plane of the body, because we know it has the following normal vector
        # Normal vector points forward
        frontalPlaneNormal = normalise(np.cross(
            leftBodySideVector, (rightShoulder - leftShoulder)))
        if showGraph:
            ax.quiver(leftShoulder[0], leftShoulder[2], leftShoulder[1], frontalPlaneNormal[0],
                    frontalPlaneNormal[2], frontalPlaneNormal[1], color='black')

        # Normal vector points left
        longitudinalPlaneNormal = normalise(np.cross(
            leftBodySideVector, frontalPlaneNormal)) #Wonder if we would be better using vector between the two shoulders
        transversePlaneNormal = np.cross(longitudinalPlaneNormal, frontalPlaneNormal) #Vector points up
        if showGraph:
            ax.quiver(leftShoulder[0], leftShoulder[2], leftShoulder[1], longitudinalPlaneNormal[0],
                    longitudinalPlaneNormal[2], longitudinalPlaneNormal[1], color='orange')
            ax.quiver(leftShoulder[0], leftShoulder[2], leftShoulder[1], transversePlaneNormal[0],
                    transversePlaneNormal[2], transversePlaneNormal[1], color='blue')

        # Omoplate movement (Coronal / frontal plane) (abduction / adduction)
        # Default 10, low 10, high 80
        leftArmVectorProjectedToFrontalPlane = projectToPlane(
            frontalPlaneNormal, topLeftArmVector)
        angleLeftArmOmoPlate = calc_angle(
            leftArmVectorProjectedToFrontalPlane, leftBodySideVector)

        rightArmVectorProjectedToFrontalPlane = projectToPlane(
            frontalPlaneNormal, topRightArmVector)
        angleRightArmOmoPlate = calc_angle(
            rightArmVectorProjectedToFrontalPlane, rightBodySideVector)

        if showGraph:
            ax.quiver(leftShoulder[0], leftShoulder[2], leftShoulder[1], leftArmVectorProjectedToFrontalPlane[0],
                    leftArmVectorProjectedToFrontalPlane[2], leftArmVectorProjectedToFrontalPlane[1], color='green', alpha=0.4)
            ax.quiver(rightShoulder[0], rightShoulder[2], rightShoulder[1], rightArmVectorProjectedToFrontalPlane[0],
                    rightArmVectorProjectedToFrontalPlane[2], rightArmVectorProjectedToFrontalPlane[1], color='red', alpha=0.4)

        # Front and Back movement (Sagittal / longitudinal plane) (flexion / extension)
        # Default 30, forward 180, backward 0
        leftArmVectorProjectedToLongitudinalPlane = projectToPlane(
            longitudinalPlaneNormal, topLeftArmVector)
        # if np.dot(leftArmVectorProjectedToLongitudinalPlane, frontalPlaneNormal) < 0:
        #     # if the arm points backwards, dot product with vector pointing forward will be -ve
        #     angleLeftArmShoulderPlane = π/6 - calc_angle(
        #         leftArmVectorProjectedToLongitudinalPlane, -transversePlaneNormal)
        # else:
        #     angleLeftArmShoulderPlane = π/6 + calc_angle(
        #         leftArmVectorProjectedToLongitudinalPlane, -transversePlaneNormal)
        angleLeftArmShoulderPlane = π/6 + calc_angle(leftArmVectorProjectedToLongitudinalPlane, -transversePlaneNormal)

        rightArmVectorProjectedToLongitudinalPlane = projectToPlane(
            longitudinalPlaneNormal, topRightArmVector)
        if np.dot(rightArmVectorProjectedToLongitudinalPlane, frontalPlaneNormal) < 0:
            # same as above
            angleRightArmShoulderPlane = π/6 - calc_angle(
                rightArmVectorProjectedToLongitudinalPlane, -transversePlaneNormal)
        else:
            angleRightArmShoulderPlane = π/6 + calc_angle(
                rightArmVectorProjectedToLongitudinalPlane, -transversePlaneNormal)

        if showGraph:
            ax.quiver(leftShoulder[0], leftShoulder[2], leftShoulder[1], leftArmVectorProjectedToLongitudinalPlane[0],
                    leftArmVectorProjectedToLongitudinalPlane[2], leftArmVectorProjectedToLongitudinalPlane[1], color='green', alpha=0.4)
            ax.quiver(rightShoulder[0], rightShoulder[2], rightShoulder[1], rightArmVectorProjectedToLongitudinalPlane[0],
                    rightArmVectorProjectedToLongitudinalPlane[2], rightArmVectorProjectedToLongitudinalPlane[1], color='green', alpha=0.4)

        # Rotation of shoulder joint
        # Default 90, inward 40, outward 180
        planeOfLeftElbowJoint = np.cross(
            lowLeftArmVector, topLeftArmVector)  # points to left (could well be wrong we need to check, because some of the others have been the wrong way round)
        leftArmRotationAngle = calc_angle(
            planeOfLeftElbowJoint, frontalPlaneNormal)

        planeOfRightElbowJoint = np.cross(
            topRightArmVector, lowRightArmVector)  # points to right, same as above
        rightArmRotationAngle = calc_angle(
            planeOfRightElbowJoint, frontalPlaneNormal)

        # This attempts to calculate how rotated your arm is by taking a plane between your body
        # side and the top of your arm and looking at the angle this makes with your lower arm
        # topLeftArmAndBodyNormal = np.cross(
        #     topLeftArmVector, leftBodySideVector)
        # leftArmRotationAngle = calc_angle(
        #     topLeftArmAndBodyNormal, lowLeftArmVector)

        # topRightArmAndBodyNormal = np.cross(
        #     topRightArmVector, rightBodySideVector)
        # rightArmRotationAngle = calc_angle(
        #     topRightArmAndBodyNormal, lowRightArmVector)

        # It will be left, before right.  Then and it is elbow, rotation, shoulder (sagittal/longitudinal), omo (coronal/frontal)
        outputArray = np.array(list(map(math.trunc, map(math.degrees, [leftElbowAngle, leftArmRotationAngle, angleLeftArmShoulderPlane, angleLeftArmOmoPlate,
                                                                    rightElbowAngle, rightArmRotationAngle, angleRightArmShoulderPlane, angleRightArmOmoPlate]))))

        # print(f"LElbow: {outputArray[0]:3} LRot: {outputArray[1]:3} LShoulder: {outputArray[2]:3} LOmo: {outputArray[3]:3} RElbow: {outputArray[4]:3} RRot: {outputArray[5]:3} RShoulder: {outputArray[6]:3} ROmo: {outputArray[7]:3}")
        # Logs the info to the terminal and publishes it to the topics so that other nodes can receive it
        if inRos:
            rospy.loginfo(outputArray)
            left_arm_publisher.publish(outputArray[0:4])
            right_arm_publisher.publish(outputArray[4:])
        else:
            print(f"LElbow: {outputArray[0]:3} LRot: {outputArray[1]:3} LShoulder: {outputArray[2]:3} LOmo: {outputArray[3]:3} RElbow: {outputArray[4]:3} RRot: {outputArray[5]:3} RShoulder: {outputArray[6]:3} ROmo: {outputArray[7]:3}")
        # print(outputArray)
        # Draw the pose annotation on the image.
        image.flags.writeable = True
        if not inRos:
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            mp_drawing.draw_landmarks(
                image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS
            )
            cv2.imshow("MediaPipe Pose", image)
        if showGraph:
            plt.pause(0.02)

# Standard setup for ROS so that we can listen for an incoming message on a topic, and call our function to deal with the input


def listener():
    rospy.init_node('/shadow_arm_controller', anonymous=True)
    image_sub = rospy.Subscriber('/camera/image_raw', Image, frameCallback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")


if __name__ == "__main__":
    if inRos:
        listener()
    else:
        cap = cv2.VideoCapture(0)
        while cap.isOpened():
            success, image = cap.read()
            if not success:
                print("Ignoring empty camera frame.")
                # If loading a video, use 'break' instead of 'continue'.
                continue
            try:
                frameCallback(image)
            except:
                continue
        cap.release()
    plt.show()
