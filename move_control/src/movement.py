# coding=utf-8
import rospy
import time
import tty
import termios
import math
from datetime import datetime
from std_msgs.msg import Int32
from move_control.msg import *

location_x=0
location_y=0
location_th=0

def callback_actionfinish(msg):
    global location_x
    global location_y
    global location_th
    print 'action done'
    location_x= float(msg.x)
    location_y = float(msg.y)
    location_th = float(msg.th)


rospy.init_node('move_control')
pub = rospy.Publisher('wheel',wheel,queue_size=1)
#ac_done = rospy.Subscriber('actionfinish',actionfinish,callback_actionfinish)
wheel_radius=0.021
axle_length=0.09
encoder_num=350
unit_length=wheel_radius*2*math.pi/encoder_num
def _forward():

    speed = wheel()
    speed.rightspeed = 25
    speed.leftspeed = 25
    speed.r_count=0
    speed.l_count=0
    pub.publish(speed)    #发布secs

def _left():
    speed = wheel()
    speed.rightspeed = 15
    speed.leftspeed = -15
    speed.r_count=0
    speed.l_count=0
    pub.publish(speed)    #发布secs

def _right():
    speed = wheel()
    speed.rightspeed=-15
    speed.leftspeed = 15
    speed.r_count=0
    speed.l_count=0
    pub.publish(speed)    #发布secs

def _back():
    speed = wheel()
    speed.rightspeed = -25
    speed.leftspeed = -25
    speed.r_count=0
    speed.l_count=0
    pub.publish(speed)    #发布secs

def _stop():
    speed = wheel()
    speed.rightspeed = 0
    speed.leftspeed = 0
    speed.r_count=0
    speed.l_count=0
    pub.publish(speed)    #发布secs

def MoveLine(dist,spd):
    speed = wheel()
    if dist>0:
        speed.rightspeed = spd
        speed.leftspeed = spd
        speed.r_count = int(dist/unit_length)
        speed.l_count = int(dist/unit_length)
    else:
        speed.rightspeed = -spd
        speed.leftspeed = -spd
        speed.r_count = int(dist / unit_length)
        speed.l_count = int(dist / unit_length)
    if speed.r_count==0 or speed.l_count ==0:
        return
    pub.publish(speed)  # 发布secs
    msg=rospy.wait_for_message('actionfinish',actionfinish)
    global location_x
    global location_y
    global location_th
    print 'action done'
    location_x = float(msg.x)
    location_y = float(msg.y)
    location_th = float(msg.th)
    print location_x, location_y, location_th

def MoveRotate(angle,spd):

    speed = wheel()
    if angle>0:
        speed.rightspeed = -spd
        speed.leftspeed = spd
        speed.r_count = int(-0.5*angle*axle_length/unit_length)
        speed.l_count = int(0.5*angle*axle_length/unit_length)
    else:
        speed.rightspeed = spd
        speed.leftspeed = -spd
        speed.r_count = int(0.5*angle*axle_length/unit_length)
        speed.l_count = int(-0.5*angle*axle_length/unit_length)
    if speed.r_count==0 or speed.l_count ==0:
        return
    pub.publish(speed)  # 发布secs
    msg=rospy.wait_for_message('actionfinish',actionfinish)
    global location_x
    global location_y
    global location_th
    print 'action done'
    location_x = float(msg.x)
    location_y = float(msg.y)
    location_th = float(msg.th)
    print location_x,location_y,location_th

def MoveTo(x,y,spd):
    th=math.atan2(x-location_x,y-location_y)
    print th
    th=th-location_th
    print th
    if th<-math.pi:
        th+=math.pi*2
    if th>math.pi:
        th-=math.pi*2
    MoveRotate(th,spd)
    MoveLine(((x-location_x)**2+(y-location_y)**2)**0.5,spd)

def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def readkey(getchar_fn=None):
    getchar = getchar_fn or readchar
    c1 = getchar()
    if ord(c1) != 0x1b:
        return c1
    c2 = getchar()
    if ord(c2) != 0x5b:
        return c1
    c3 = getchar()
    return chr(0x10 + ord(c3) - 65)



while True:
    #print "input: f,r,t,q"
    key=readkey()
    # if key=='f':
    #     print "input: dist"
    #     dist=input()
    #     print "input: speed(1-30)"
    #     speed =input()
    #     MoveLine(dist,speed)
    # if key=='r':
    #     print "input: angle"
    #     angle=input()
    #     print "input: speed(1-30)"
    #     speed =input()
    #     MoveRotate(angle,speed)
    # if key=='t':
    #     print "input: x"
    #     x=input()
    #     print "input: y"
    #     y = input()
    #     print "input: speed(1-30)"
    #     speed =input()
    #     MoveTo(x,y,speed)
    if key=='w':
        #go_forward()
        _forward()

        print 'w'
    if key=='a':
        _left()
        #go_back()
        print 'a'
    if key=='s':
        _back()
        #go_left()
        print 's'
    if key=='d':
        _right()
        print 'd'
    	#go_right()

    if key=='f':
        MoveRotate(-math.pi,10)
        print 'f'
    if key=='h':
        MoveRotate(math.pi,10)
        print 'h'
    if key=='t':
        MoveLine(0.5,15)
        print 't'
    if key=='g':
        MoveLine(-0.5,15)
        print 'g'
    if key == 'x':
        _stop()
    if key=='q':

    	break
