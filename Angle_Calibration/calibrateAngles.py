Debug = True

import yaml
import msvcrt as m
if not Debug:
    from adafruit_servokit import ServoKit
    kit = ServoKit(channels=48)

# W to increase the angle, S to decrease the angle, O to accept the angle
joints = ['r_thumb', 'r_index', 'r_middle', 'r_ringFinger', 'r_pinky', 'r_wrist', 'r_bicep', 'r_arm_rotate', 'r_shoulder', 'r_omoplate',
          'l_thumb', 'l_index', 'l_middle', 'l_ringFinger', 'l_pinky', 'l_wrist', 'l_bicep', 'l_arm_rotate', 'l_shoulder', 'l_omoplate',
          'neck', 'rothead', 'rollneck', 'eyelid', 'jaw', 'eyeX', 'eyeY', 'topstom', 'midstom', 'lowstom']

servoNamesToIndices = {}
with open(r'jointNamesToServoNumber.yml') as inputFile:
    servoNamesToIndices = yaml.full_load(inputFile)

outputData = {}


def moveToAngle(joint, angle):
    if Debug:
        print(f"Angle: {angle}")
    else:
        kit.servo[servoNamesToIndices[joint]].angle = angle


for joint in joints:
    currentAngle = 0
    skipJoint = False
    # First set the closed position of the joint
    print(f"{joint}")
    print("Setting Closed Position")
    dataEntered = ''
    while dataEntered != 'O' and not skipJoint:
        moveToAngle(joint, currentAngle)
        dataEntered = input("Enter angle: ")
        if dataEntered.isnumeric():
            currentAngle = int(dataEntered)
        elif dataEntered == '':
            skipJoint = True
    if not skipJoint:
        closedAngle = currentAngle
        print(f"Closed angle set to {closedAngle}")
        dataEntered = ''
        print("Setting Rest Position")
        dataEntered = ''
        while dataEntered != 'O':
            moveToAngle(joint, currentAngle)
            dataEntered = input("Enter angle: ")
            if dataEntered.isnumeric():
                currentAngle = int(dataEntered)
        restAngle = currentAngle
        print(f"Rest angle set to {restAngle}")
        dataEntered = ''
        print("Setting Open Position")
        dataEntered = ''
        while dataEntered != 'O':
            moveToAngle(joint, currentAngle)
            dataEntered = input("Enter angle: ")
            if dataEntered.isnumeric():
                currentAngle = int(dataEntered)
        openAngle = currentAngle
        print(f"Open angle set to {openAngle}")
        outputData[joint] = {'closed': closedAngle,
                            'rest': restAngle, 'open': openAngle}
        moveToAngle(joint,restAngle)


with open('calibrationPoints.yml', 'w') as outfile:
    yaml.dump(outputData, outfile)
