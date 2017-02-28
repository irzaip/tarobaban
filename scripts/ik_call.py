#!/usr/bin/env python
import robot as rb
import sys
import rospy
from tarobaban.srv import *
import struct
import time
import serial
import math


ret0 = 0.0
ret1 = 0.0
ret2 = 0.0


lengthUpperArm = 0.479
lengthLowerArm = 0.350
heightFromBase = 0.335

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
    return [baseAngle * (180/math.pi), lowerArmAngle * (180/math.pi), upperArmAngle * (180/math.pi)]



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

    lowerArmAngle = 90-ang2+ang1
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


def give_angles(x, y, z):
    rospy.wait_for_service('get_angle')
    try:
        get_angles = rospy.ServiceProxy('get_angle', get_angle)
        resp1 = get_angles(x, y, z)
        return 
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def trns(x,y,z):
    #define here the transform translation and rotation
    a = rb.transl(0, 0.750,0)
    b = rb.trotz(rb.deg2rad(180))

    #asumsi titik + [1]
    pt = [[x],[y],[z],[1]]
    cc = a + (b * pt)
    result = rb.transl(cc)
    x,y,z=result

    #kembalikan kebentuk kord
    x=-x[0,0]
    y=y[0,0]
    z=z[0,0]
    return x,y,z

if __name__ == "__main__":


    commandFlag = True
    while(commandFlag):
        x = input('Enter x: ')
        y = input('Enter y: ')
        z = input('Enter z: ')

        #convert to user cordinate
        x,y,z = trns(x,y,z)

        angles = convert_cartesian_coordinate_to_arm_angles(float(x),float(y),float(z),lengthUpperArm,lengthLowerArm,heightFromBase)

        if(angles[0] != -999):
            print('\nFor the point (' + str(x) + ' , ' + str(y) + ' , ' + str(z) + ') , the angles are:' )
            print('Base Angle: ' + str(angles[0]))
            print('Lower Arm Angle: ' + str(angles[1]))
            print('Upper Arm Angle: ' + str(angles[2]))
            print('\n')

#            ser.write(struct.pack('f',float(angles[0])))
#            ser.write(struct.pack('f',float(angles[1])))
#            ser.write(struct.pack('f',float(angles[2])))
            give_angles(math.radians(-angles[0]), math.radians((-angles[1]+90)), math.radians((angles[2])))

        else:
            print('Invalid coordinate: Out of arm\'s range.')
            print('\n')

        s = raw_input('quit?')
        if (s == 'y'):
            commandFlag = False

    