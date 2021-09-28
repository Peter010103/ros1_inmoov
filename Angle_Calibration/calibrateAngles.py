import yaml
import msvcrt as m
# W to increase the angle, S to decrease the angle, O to accept the angle
joints = ['r_thumb', 'r_index', 'r_middle', 'r_ringFinger', 'r_pinky', 'r_wrist', 'r_bicep', 'r_arm_rotate', 'r_shoulder', 'r_omoplate',
          'l_thumb', 'l_index', 'l_middle', 'l_ringFinger', 'l_pinky', 'l_wrist', 'l_bicep', 'l_arm_rotate', 'l_shoulder', 'l_omoplate',
          'neck', 'rothead', 'rollneck', 'eyelid', 'jaw', 'eyeX', 'eyeY', 'topstom', 'midstom', 'lowstom']

outputData = {}


def moveToAngle(joint, angle):
    print(f"Angle: {angle}")


for joint in joints:
    currentAngle = 0
    # First set the closed position of the joint
    print(f"{joint}")
    print("Setting Closed Position")
    dataEntered = ''
    while dataEntered != 'O':
        moveToAngle(joint, currentAngle)
        dataEntered = input("Enter angle: ")
        if dataEntered.isnumeric():
            currentAngle = int(dataEntered)
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
