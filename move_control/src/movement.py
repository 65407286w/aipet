# coding=utf-8
import rospy
import time
import tty
import termios
from datetime import datetime
from std_msgs.msg import Int32
from move_control.msg import *
rospy.init_node('rospub')

pub = rospy.Publisher('wheel',wheel,queue_size=1)    #实例化publisher,向话题‘topic’发布Int32类型的数据

def _forward():
    speed = wheel()
    speed.rightspeed=1
    speed.leftspeed = 1
    pub.publish(speed)    #发布secs

def _left():
    speed = wheel()
    speed.rightspeed=1
    speed.leftspeed = -1
    pub.publish(speed)    #发布secs

def _right():
    speed = wheel()
    speed.rightspeed=-1
    speed.leftspeed = 1
    pub.publish(speed)    #发布secs

def _back():
    speed = wheel()
    speed.rightspeed = -1
    speed.leftspeed = -1
    pub.publish(speed)    #发布secs

def _stop():
    speed = wheel()
    speed.rightspeed = 0
    speed.leftspeed = 0
    pub.publish(speed)    #发布secs

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
    key=readkey()
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
    if key == 'x':
     _stop()
    if key=='q':

    	break
