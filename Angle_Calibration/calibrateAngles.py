# A to accept the angle
import yaml
import os
Debug = False

if not Debug:
    from adafruit_servokit import ServoKit
    kit = ServoKit(channels=16)

servoNamesToIndices = {}
with open(r'jointNamesToServoNumber.yml') as inputFile:
    servoNamesToIndices = yaml.full_load(inputFile)

outputData = {}
if os.path.isfile('./calibrationPoints.yml'):
    with open(r'calibrationPoints.yml') as previousValues:
        outputData = yaml.full_load(previousValues)

def moveToAngle(joint, angle):
    if Debug:
        print(f"Angle: {angle}")
    else:
        kit.servo[servoNamesToIndices[joint]].angle = angle


for joint in servoNamesToIndices:
    if (servoNamesToIndices[joint] >= 16):
        continue
    currentAngle = 0
    skipJoint = False
    # First set the closed position of the joint
    print(f"{joint}")
    print("Setting Closed Position")
    dataEntered = ''

    while dataEntered != 'A' and not skipJoint:
        moveToAngle(joint, currentAngle)
        dataEntered = input("Enter angle: ").upper()

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

        while dataEntered != 'A':
            moveToAngle(joint, currentAngle)
            dataEntered = input("Enter angle: ").upper()

            if dataEntered.isnumeric():
                currentAngle = int(dataEntered)

        restAngle = currentAngle
        print(f"Rest angle set to {restAngle}")
        dataEntered = ''
        print("Setting Open Position")
        dataEntered = ''

        while dataEntered != 'A':
            moveToAngle(joint, currentAngle)
            dataEntered = input("Enter angle: ").upper()

            if dataEntered.isnumeric():
                currentAngle = int(dataEntered)

        openAngle = currentAngle
        print(f"Open angle set to {openAngle}")
        outputData[joint] = {'closed': closedAngle,
                             'rest': restAngle, 'open': openAngle}
        moveToAngle(joint, restAngle)


with open('calibrationPoints.yml', 'w') as outfile:
    yaml.dump(outputData, outfile)
