#!/usr/bin/env python
import robot as rb
import sys
import rospy
from tarobaban.srv import *
import struct
import time
import serial
import math
import os, struct, array
from fcntl import ioctl
from visualization_msgs.msg import *


ret0 = 0.0
ret1 = 0.0
ret2 = 0.0


lengthUpperArm = 0.479
lengthLowerArm = 0.350
heightFromBase = 0.335



# Iterate over the joystick devices.
print('Available devices:')

for fn in os.listdir('/dev/input'):
    if fn.startswith('js'):
        print('  /dev/input/%s' % (fn))

# We'll store the states here.
axis_states = {}
button_states = {}

# These constants were borrowed from linux/input.h
axis_names = {
    0x00 : 'x',
    0x01 : 'y',
    0x02 : 'z',
    0x03 : 'rx',
    0x04 : 'ry',
    0x05 : 'rz',
    0x06 : 'trottle',
    0x07 : 'rudder',
    0x08 : 'wheel',
    0x09 : 'gas',
    0x0a : 'brake',
    0x10 : 'hat0x',
    0x11 : 'hat0y',
    0x12 : 'hat1x',
    0x13 : 'hat1y',
    0x14 : 'hat2x',
    0x15 : 'hat2y',
    0x16 : 'hat3x',
    0x17 : 'hat3y',
    0x18 : 'pressure',
    0x19 : 'distance',
    0x1a : 'tilt_x',
    0x1b : 'tilt_y',
    0x1c : 'tool_width',
    0x20 : 'volume',
    0x28 : 'misc',
}

button_names = {
    0x120 : 'trigger',
    0x121 : 'thumb',
    0x122 : 'thumb2',
    0x123 : 'top',
    0x124 : 'top2',
    0x125 : 'pinkie',
    0x126 : 'base',
    0x127 : 'base2',
    0x128 : 'base3',
    0x129 : 'base4',
    0x12a : 'base5',
    0x12b : 'base6',
    0x12f : 'dead',
    0x130 : 'a',
    0x131 : 'b',
    0x132 : 'c',
    0x133 : 'x',
    0x134 : 'y',
    0x135 : 'z',
    0x136 : 'tl',
    0x137 : 'tr',
    0x138 : 'tl2',
    0x139 : 'tr2',
    0x13a : 'select',
    0x13b : 'start',
    0x13c : 'mode',
    0x13d : 'thumbl',
    0x13e : 'thumbr',

    0x220 : 'dpad_up',
    0x221 : 'dpad_down',
    0x222 : 'dpad_left',
    0x223 : 'dpad_right',

    # XBox 360 controller uses these codes.
    0x2c0 : 'dpad_left',
    0x2c1 : 'dpad_right',
    0x2c2 : 'dpad_up',
    0x2c3 : 'dpad_down',
}

axis_map = []
button_map = []

# Open the joystick device.
fn = '/dev/input/js0'
print('Opening %s...' % fn)
jsdev = open(fn, 'rb')

# Get the device name.
#buf = bytearray(63)
buf = array.array('c', ['\0'] * 64)
ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
js_name = buf.tostring()
print('Device name: %s' % js_name)

# Get number of axes and buttons.
buf = array.array('B', [0])
ioctl(jsdev, 0x80016a11, buf) # JSIOCGAXES
num_axes = buf[0]

buf = array.array('B', [0])
ioctl(jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
num_buttons = buf[0]

# Get the axis map.
buf = array.array('B', [0] * 0x40)
ioctl(jsdev, 0x80406a32, buf) # JSIOCGAXMAP

for axis in buf[:num_axes]:
    axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
    axis_map.append(axis_name)
    axis_states[axis_name] = 0.0

# Get the button map.
buf = array.array('H', [0] * 200)
ioctl(jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

for btn in buf[:num_buttons]:
    btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
    button_map.append(btn_name)
    button_states[btn_name] = 0

print '%d axes found: %s' % (num_axes, ', '.join(axis_map))
print '%d buttons found: %s' % (num_buttons, ', '.join(button_map))



# Main event loop

x_val = 0.0
y_val = 0.0

rx_val = 0.0
ry_val = 0.0

x_cord = 0.0
y_cord = 0.0
z_cord = 0.0

debug = False

def update_IM(x,y,z):
    command = InteractiveMarkerFeedback()
    command.event_type = 1
    command.pose.position.x = x
    command.pose.position.y = y
    command.pose.position.z = z
    command.marker_name = "simple_6dof_MOVE_3D"
    command.control_name = "wow"
    rospy.loginfo('marker to x: %s, y: %s, z: %s' % (str(x),str(y), str(z)))
    pub.publish(command)
    #rate.sleep()


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
    if debug: print(str(distanceFromOriginToEndPoint))
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
    if debug: print "dd:"+str(dd)
    if debug: print "ang_dd:"+str(math.degrees(dd))

    #seberang dari sudut dd , tegak lurus
    x=math.sin(dd)*upperArmLength
    if debug: print "x:"+str(x)

    y1=abs(math.cos(dd)*upperArmLength)
    if debug: print "y1:"+str(y1)

    if (int(math.degrees(dd))>=int(90.0)):
        y2=abs(lowerArmLength+y1)
    else:
        y2=abs(lowerArmLength-y1)
    if debug: print "y2:"+str(y2)

    ang1 = math.degrees(math.atan2(x,y2))
    if debug: print "ang1:"+str(ang1)
    
    if debug: print "z:"+str(z)
    if (int(heightFromBase)>int(z+heightFromBase)):
        ang2 = 90-math.degrees(math.atan2(abs(z),abs(r)))
    else:
        ang2 = 90+math.degrees(math.atan2(abs(z),abs(r)))
        if debug: print "up"

    if debug: print "ang2:"+str(ang2)

    lowerArmAngle = 90-ang2+ang1
    if debug: print "lower_ang:"+str(lowerArmAngle)
     
    sudut_bantu = 180.0-math.degrees(dd)-lowerArmAngle
    if debug: print "sudut_bantu:"+str(sudut_bantu)+"\r\n"

    upperArmAngle = 180.0 - math.degrees(dd) 
    if debug: print "upper_ang:"+str(upperArmAngle)+"\r\n"

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
    a = rb.transl(0,0 ,0)
    b = rb.trotz(rb.deg2rad(-90))

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


def go_cord(x,y,z):
    x,y,z = trns(x,y,z)
    angles = convert_cartesian_coordinate_to_arm_angles(float(x),float(y),float(z),lengthUpperArm,lengthLowerArm,heightFromBase)
    give_angles(math.radians(-angles[0]), math.radians((-angles[1]+90)), math.radians((angles[2])))
    return



if __name__ == "__main__":

    pub = rospy.Publisher('/basic_controls/feedback', InteractiveMarkerFeedback, queue_size=10)
    rospy.init_node('IM_joystick', anonymous=True)
    rate = rospy.Rate(10) # 10hz


    while True:

        #make X movement
        if x_val > 0.1:
            x_cord=x_cord+(x_val*0.01)
            print "x_cord::",x_cord
            update_IM(x_cord,y_cord,z_cord)

        if x_val < -0.1:
            x_cord=x_cord+(x_val*0.01)
            print "x_cord::",x_cord
            update_IM(x_cord,y_cord,z_cord)


        #make Y movement
        #y substraction to reverse AXIS
        if y_val > 0.1:
            y_cord=y_cord-(y_val*0.01)
            print "y_cord::",y_cord
            update_IM(x_cord,y_cord,z_cord)

        if y_val < -0.1:
            y_cord=y_cord-(y_val*0.01)
            print "y_cord::",y_cord
            update_IM(x_cord,y_cord,z_cord)

        #make Z movement with ry
        #inverse axis too
        if ry_val > 0.1:
            z_cord=z_cord-(ry_val*0.01)
            print "z_cord::",z_cord
            update_IM(x_cord,y_cord,z_cord)

        if ry_val < -0.1:
            z_cord=z_cord-(ry_val*0.01)
            print "z_cord::",z_cord
            update_IM(x_cord,y_cord,z_cord)


        
        evbuf = jsdev.read(8)
        if evbuf:
            time, value, type, number = struct.unpack('IhBB', evbuf)

            if type & 0x80:
                 print "(initial)",

            if type & 0x01:
                button = button_map[number]
                if button:
                    button_states[button] = value
                    if value:
                        print "%s pressed" % (button)
                        if button=='x': 
                            print "MOVE",x_cord,y_cord,z_cord
                            go_cord(x_cord,y_cord,z_cord)
                        if button=='tr':
                            sys.exit()

                    else:
                        print "%s released" % (button)

            if type & 0x02:
                axis = axis_map[number]
                if axis:
                    fvalue = value / 32767.0
                    axis_states[axis] = fvalue
                    print "%s: %.3f" % (axis, fvalue)

                    #additional move to cord
                    if axis == 'x':
                        if fvalue > 0.1 or fvalue < -0.1:
                            x_val = fvalue
                        else:
                            x_val = 0
        
                    if axis == 'y':
                        if fvalue > 0.1 or fvalue < -0.1:
                            y_val = fvalue
                        else:
                            y_val = 0

                    if axis == 'ry':
                        if fvalue > 0.1 or fvalue < -0.1:
                            ry_val = fvalue
                        else:
                            ry_val = 0
