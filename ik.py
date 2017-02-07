"""
Implements inverse and forward kinematics functions, in addition to other movement calculation functions for the Dobot Arm
First Author: Mike Ferguson www.mikeahferguson.com 3/26/2016
Additional Authors (Add your name below):
1.
License: MIT


"""
import struct
import time
import serial

import math

#ser = serial.Serial(
#      port='COM3',
#      baudrate=115200,
#      )

#ser.isOpen()


"""
polar coordinates conversion: https://www.youtube.com/watch?v=L4v98ZZft68
IK 2D, 2DOF, revolute revolute: https://www.youtube.com/watch?v=cvzv3YxuoQE

algorithm

xy plane is paralell to surface dobot is on. z is perpendicular
1. first get distance, in xy plane, to current point from origin using forward kinematics, known angles, and pythagoreas thm. This is your radius. Your angle can be defined to be theta original. You now have your starting point in polar coordinates.
2. Ignore the desired z position data for now. Get the polar coordinates for the desired point. The radius is not important now. The angle is though. Subtracting the desired angle from the current angle gives you the number of degrees and direction to rotate the base.
3. The radius from step 1 (starting radius) gives you your current horizontal position (imagine it as x or y, doesn't matter). You also already know your current z position (potentially from step 1).
4. The radius from step 2 (desired radius) gives you your desired horizontal position (imagine it as x or y, doesn't matter). Of course, the user has already input the desired z position. This is now a 2D problem with two arms that each rotate (2 degrees of freedom)
5: use IK, see ik 2d video, to find number of degrees and direction to rotate upper and lower arms. Note that there are often two solutions. One (elbow down) is not possible.
6. Check that move is valid (e.g. not out of range, etc...)
7. move


default working angle units are radians
"""
#length and height dimensions in mm
lengthUpperArm = 479.0
lengthLowerArm = 350.0
heightFromBase = 335.0

#input:
#cartesian (x,y,z) coordinate
#robot dimensions
#output:
#angles for robot arm base, upper, and lower arms in degree
def convert_cartesian_coordinate_to_arm_angles(x, y, z, upperArmLength, lowerArmLength, baseHeight):

    #do a general check to see if even a maximally stretched arm could reach the point
    # if it can't, return some dummy angle numbers of -999
    #note the base height correction in z
    distanceFromOriginToEndPoint = get_distance_from_origin_to_cartesian_point_3D(x,y,z-heightFromBase)
    print(str(distanceFromOriginToEndPoint))
    if (distanceFromOriginToEndPoint > (upperArmLength + lowerArmLength)):
        return [-999,-999,-999]

    baseAngle = get_polar_coordinate_angle_from_cartesian_x_y_coordinate(x,y)
    radiusToEndPoint = get_polar_coordinate_radius_from_cartesian_x_y_coordinate(x,y)
    #note the correction for base height
    armAngles = get_arm_angles_from_radius_z_coordinate_using_2d_revolute_revolute_inverse_kinematics(radiusToEndPoint, z-heightFromBase, upperArmLength, lowerArmLength)
    upperArmAngle = armAngles[0]
    lowerArmAngle = armAngles[1]

    #convert the angles to degrees when you return them
    return [baseAngle * (180/math.pi), upperArmAngle * (180/math.pi), lowerArmAngle * (180/math.pi)]



def get_arm_angles_from_radius_z_coordinate_using_2d_revolute_revolute_inverse_kinematics(r, z, upperArmLength, lowerArmLength):

    #akar r dan z = diameter
    diam = math.sqrt(pow(r,2)+pow(z,2))

    #sudut dari seberang diam
    dd = math.acos((pow(upperArmLength,2)+pow(lowerArmLength,2)-pow(r,2)-pow(z,2))/(2*float(upperArmLength)*float(lowerArmLength)))
    print "dd:"+str(dd)
    print "ang_dd:"+str(math.degrees(dd))

    #seberang dari sudut dd , tegak lurus
    x=math.sin(dd)*upperArmLength
    print "x:"+str(x)

    y1=abs(math.cos(dd)*upperArmLength)
    print "y1:"+str(y1)

    if (int(math.degrees(dd))>=int(90.0)):
    	y2=abs(lowerArmLength+y1)
    else:
    	y2=abs(lowerArmLength-y1)
    print "y2:"+str(y2)

    ang1 = math.degrees(math.atan2(x,y2))
    print "ang1:"+str(ang1)
    
    print "z:"+str(z)
    if (int(heightFromBase)>int(z+heightFromBase)):
    	ang2 = 90-math.degrees(math.atan2(abs(z),abs(r)))
    else:
        ang2 = 90+math.degrees(math.atan2(abs(z),abs(r)))
        print "up"

    print "ang2:"+str(ang2)

    lowerArmAngle = ang2+ang1-90.0
    print "lower_ang:"+str(lowerArmAngle)
     
    sudut_bantu = 180.0-math.degrees(dd)-lowerArmAngle
    print "sudut_bantu:"+str(sudut_bantu)+"\r\n"

    upperArmAngle = 180.0 - math.degrees(dd) 
    print "upper_ang:"+str(upperArmAngle)+"\r\n"

    upperArmAngle = math.radians(upperArmAngle)
    lowerArmAngle = math.radians(lowerArmAngle)

    return [upperArmAngle, lowerArmAngle]



#input:
#x,y coordinate
#output
#polar coordinate angle to x,y point in radians
def get_polar_coordinate_angle_from_cartesian_x_y_coordinate(x,y):
    #use cartesian to polar conversion equations to get the angle
    #alternatively, this is just using the toa rule
    angle = math.atan2(y,x)
    return angle

#input:
#x,y coordinate
#output
#length of polar coordinate radius to x,y point
def get_polar_coordinate_radius_from_cartesian_x_y_coordinate(x,y):
    #use cartesian to polar conversion equations to get the angle
    #alternatively, this is just using the circle equation rule
    radius = math.sqrt(pow(x,2) + pow(y,2))
    return radius

def get_distance_from_origin_to_cartesian_point_3D(x,y,z):
    #get distance from origin (0,0,0) to end point in 3D using pythagorean thm in 3D; distance = sqrt(x^2+y^2+z^2)
    distanceToEndPoint = math.sqrt( pow(x,2) + pow(y,2) + pow(z,2) )
    print(distanceToEndPoint)
    return distanceToEndPoint


def get_upper_arm_angle():
    #return the angle in degrees or radians for the upper arm from the accelerometer data and/or known theoretical angle
    return 45#or radians!

def get_lower_arm_angle():
    #return the angle in degrees or radians for the lower arm from the accelerometer data and/or known theoretical angle
    return 45#or radians!

def get_base_angle():
    #return the angle in degrees or radians for the base from the accelerometer data and/or known theoretical angle
    return 45#or radians!


"""

    #get polar coordinates for the current point
    #radius is simply given by the base angle
    # can read the angles from the IMU and empirically determine the radius and angle - I'm using this approach since it should account for any build up in error, assuming accelerometers
    #are accurate!!!!
    #alternatively can just use pythagorean thm on theoretical current x,y data

    startUpperArmAngle = get_upper_arm_angle
    startLowerArmAngle = get_lower_arm_angle
    startBaseAngle = get_base_angle

    #could abstract this next bit into a 2D forward kinematics function and then just use the horizontal data returned
    #only care about the radius, so


    currentPosPolarCoordRadius = ???
    currentPosPolarCoordAngle = currentBaseAngle


    #end get polar coordinates


def get_radius_in_horizontal_plane_to_cartesian_end_effector_position_using_2d_revolute_revolute_forward_kinematics(upperArmAngle, lowerArmAngle, upperArmLength, lowerArmLength):
    #the equation for radius is determined using forward kinematics, just uses socahtoa rules, namely coa here.
    radius = ( math.cos(upperArmAngle) * upperArmLength ) + ( math.cos(upperArmAngle + lowerArmAngle) * lowerArmLength )
    return radius


"""


if __name__ == '__main__':



    # length and height dimensions in mm
    lengthUpperArm = 479.0
    lengthLowerArm = 350.0
    heightFromBase = 335.0

    commandFlag = True
    while(commandFlag):
        x = input('Enter x: ')
        y = input('Enter y: ')
        z = input('Enter z: ')
        angles = convert_cartesian_coordinate_to_arm_angles(float(x),float(y),float(z),lengthUpperArm,lengthLowerArm,heightFromBase)

        if(angles[0] != -999):
            print('\nFor the point (' + str(x) + ' , ' + str(y) + ' , ' + str(z) + ') , the angles are:' )
            print('Base Angle: ' + str(angles[0]))
            print('Upper Arm Angle: ' + str(angles[1]))
            print('Lower Arm Angle: ' + str(angles[2]))
            print('\n')

#            ser.write(struct.pack('f',float(angles[0])))
#            ser.write(struct.pack('f',float(angles[1])))
#            ser.write(struct.pack('f',float(angles[2])))


        else:
            print('Invalid coordinate: Out of arm\'s range.')
            print('\n')

        s = raw_input('quit?')
        if (s == 'y'):
            commandFlag = False


