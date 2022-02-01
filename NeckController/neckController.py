#!/usr/bin/env python
import matplotlib.pyplot as plt
import math
import cv2 as cv2
from enum import Enum
import numpy as np
import mediapipe as mp
from mpl_toolkits import mplot3d
inRos = False
if inRos:
    import rospy
    from std_msgs.msg import Int16MultiArray
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge, CvBridgeError
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
    spinal_column_publisher = rospy.Publisher(
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
        leftEye = np.array([landmarks[mp_pose.PoseLandmark.LEFT_EYE].x,
                            landmarks[mp_pose.PoseLandmark.LEFT_EYE].y, landmarks[mp_pose.PoseLandmark.LEFT_EYE].z])
        rightEye = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_EYE].x,
                             landmarks[mp_pose.PoseLandmark.RIGHT_EYE].y, landmarks[mp_pose.PoseLandmark.RIGHT_EYE].z])
        mouthLeft = np.array([landmarks[mp_pose.PoseLandmark.MOUTH_LEFT].x,
                             landmarks[mp_pose.PoseLandmark.MOUTH_LEFT].y, landmarks[mp_pose.PoseLandmark.MOUTH_LEFT].z])
        leftShoulder = np.array([landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].x,
                                 landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].y, landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].z])
        rightShoulder = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].x,
                                  landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].y, landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].z])
        leftHip = np.array([landmarks[mp_pose.PoseLandmark.LEFT_HIP].x,
                            landmarks[mp_pose.PoseLandmark.LEFT_HIP].y, landmarks[mp_pose.PoseLandmark.LEFT_HIP].z])

        if showGraph:
            # This is where we want to display the points that we have picked out in colour
            X = np.column_stack(
                (leftEye, rightEye, leftShoulder, rightShoulder, leftHip))
            # This is an array 3 long which has x,y,z data in that order
            ax.scatter3D(
                X[0],
                X[2],
                X[1],
                c='red'
            )

        # This calculates the vectors
        eyeVector = leftEye - rightEye
        shoulderVector = rightShoulder - leftShoulder
        leftBodySideVector = leftHip - leftShoulder
        if showGraph:
            ax.quiver(rightEye[0], rightEye[2], rightEye[1],
                      eyeVector[0], eyeVector[2], eyeVector[1], color='green')

        # Calculate the plane of the body, because we know it has the following normal vector
        # Normal vector points forward
        frontalPlaneNormal = normalise(np.cross(
            leftBodySideVector, shoulderVector))
        if showGraph:
            ax.quiver(leftShoulder[0], leftShoulder[2], leftShoulder[1], frontalPlaneNormal[0],
                      frontalPlaneNormal[2], frontalPlaneNormal[1], color='black')

        # Normal vector points left
        longitudinalPlaneNormal = normalise(np.cross(
            leftBodySideVector, frontalPlaneNormal))  # Wonder if we would be better using vector between the two shoulders

        # Normal vector points up
        transversePlaneNormal = np.cross(
            longitudinalPlaneNormal, frontalPlaneNormal)
        if showGraph:
            ax.quiver(leftShoulder[0], leftShoulder[2], leftShoulder[1], longitudinalPlaneNormal[0],
                      longitudinalPlaneNormal[2], longitudinalPlaneNormal[1], color='orange')
            ax.quiver(leftShoulder[0], leftShoulder[2], leftShoulder[1], transversePlaneNormal[0],
                      transversePlaneNormal[2], transversePlaneNormal[1], color='blue')

        # LR rotation of head, movement by STERNOCLEIDOMASTOID (SCM)
        # Guess is rothead?
        # Default 90, low 30, high 150
        eyeVectorProjectedToTransversePlane = projectToPlane(
            transversePlaneNormal, eyeVector)
        sternocleidomastoid = π - \
            calc_angle(eyeVectorProjectedToTransversePlane, frontalPlaneNormal)

        if showGraph:
            ax.text(mouthLeft[0], mouthLeft[2], mouthLeft[1], str(
                math.degrees(sternocleidomastoid))[:4], color='green')

        # LR slanting of head movement, movement by SCALENE
        # Rollneck movement?
        # Default 90, low 60, high 130
        eyeVectorProjectedToFrontalPlane = projectToPlane(
            frontalPlaneNormal, eyeVector)
        rollneckAngle = calc_angle(
            eyeVectorProjectedToFrontalPlane, longitudinalPlaneNormal)

        # FB nodding of head, movement by Splenius Capitis
        # Could be neck?
        # Default 90, low 20, high 160

        leftEyeMouthVector = mouthLeft - leftEye
        facePlaneNormalVector = normalise(
            np.cross(leftEyeMouthVector, eyeVector))
        facePlaneNormalProjectedToLongitudinal = projectToPlane(
            longitudinalPlaneNormal, facePlaneNormalVector)
        neckAngle = π - \
            calc_angle(facePlaneNormalProjectedToLongitudinal,
                       transversePlaneNormal)

        # It will be left, before right.  Then and it is neck (nodding), rotation (neck shaking), roll neck (slanting)
        outputArray = np.array(list(map(math.trunc, map(math.degrees, [neckAngle, sternocleidomastoid, rollneckAngle]))))

        # print(f"LElbow: {outputArray[0]:3} LRot: {outputArray[1]:3} LShoulder: {outputArray[2]:3} LOmo: {outputArray[3]:3} RElbow: {outputArray[4]:3} RRot: {outputArray[5]:3} RShoulder: {outputArray[6]:3} ROmo: {outputArray[7]:3}")
        # Logs the info to the terminal and publishes it to the topics so that other nodes can receive it
        if inRos:
            rospy.loginfo(outputArray)
            spinal_column_publisher.publish(outputArray[0:4])
        else:
            print(
                f"Neck: {outputArray[0]:3} Neck Rotation: {outputArray[1]:3} NeckRoll: {outputArray[2]:3}")
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
