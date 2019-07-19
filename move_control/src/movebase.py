# coding=utf-8
import rospy
import time
from datetime import datetime
from std_msgs.msg import Int32
from move_control.msg import *
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
#left side
ENA = 7
#right side
ENB = 8
IN1 = 11
IN2 = 12
IN3 = 13
IN4 = 15

MH1 = 38
MH2 = 40

left_speed = 0
right_speed = 0
require_left_speed = 0
require_right_speed = 0

left_ori =1
right_ori =1
left_speed = 0
right_speed = 0
left_dutycycle=0
right_dutycycle=0

rospy.init_node('subscriber')


def init():
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)
    GPIO.setup(IN3, GPIO.OUT)
    GPIO.setup(IN4, GPIO.OUT)
    GPIO.setup(ENA, GPIO.OUT)
    GPIO.setup(ENB, GPIO.OUT)
    GPIO.setup(MH1, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # 通过16号引脚读取左轮脉冲数据
    GPIO.setup(MH2, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # 通过18号引脚读取右轮脉冲数据

def callback_wheel(msg):    #定义回调函数
    global right_ori
    global require_right_speed
    global left_ori
    global require_left_speed

    if msg.leftspeed > 0:
        right_ori = 1
    else:
        right_ori = -1
    require_right_speed = abs(msg.rightspeed)

    if msg.rightspeed >0:
        left_ori = 1
    else:
        left_ori = -1
    require_left_speed = abs(msg.leftspeed)
    print require_right_speed
    print require_right_speed
    print "*****"






def my_callback_l(channel_l):            #这里的channel和channel1无须赋确定值，但笔者测试过，不能不写
    global left_speed
    if GPIO.event_detected(MH1):
        left_speed=left_speed+1
        #print left_speed


def my_callback_r(channel_r):          #边缘检测回调函数，详情在参见链接中
    global right_speed                 #设置为全局变量
    if GPIO.event_detected(MH2):        #检测到一个脉冲则脉冲数加1
        right_speed=right_speed+1
        #print right_speed

def change_speed(l,r):
    global left_dutycycle
    global right_dutycycle
    if right_ori == 1:
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
    if right_ori == -1:
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
    if left_ori == 1:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
    if left_ori == -1:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)

    left_dutycycle += l
    right_dutycycle += r
    # print left_dutycycle
    # print right_dutycycle
    if (left_dutycycle>100):
        left_dutycycle =100
    if (left_dutycycle<50 and l<0):
        left_dutycycle =0
    if (left_dutycycle<50 and l>0):
        left_dutycycle =50
    if (right_dutycycle>100):
        right_dutycycle =100
    if (right_dutycycle < 50 and r<0):
        right_dutycycle = 0
    if (right_dutycycle < 50 and r>0):
        right_dutycycle = 50

    pa.ChangeDutyCycle(left_dutycycle)
    pb.ChangeDutyCycle(right_dutycycle)

def timer_callback(self):
    global left_speed
    global right_speed
    global require_left_speed
    global require_right_speed

    # print "-----"
    # print left_speed
    # print require_left_speed
    # #
    # print right_speed
    # print require_right_speed
    if require_left_speed < left_speed and require_right_speed < right_speed:
        change_speed(-1,-1)
    if require_left_speed < left_speed and require_right_speed > right_speed:
        change_speed(-1,1)
    if require_left_speed > left_speed and require_right_speed < right_speed:
        change_speed(1,-1)
    if require_left_speed > left_speed and require_right_speed > right_speed:
        change_speed(1,1)


    if require_left_speed > left_speed and require_right_speed == right_speed:
        change_speed(1,0)
    if require_left_speed < left_speed and require_right_speed == right_speed:
        change_speed(-1,0)
    if require_left_speed == left_speed and require_right_speed > right_speed:
        change_speed(0,1)
    if require_left_speed == left_speed and require_right_speed < right_speed:
        change_speed(0,-1)

    if require_left_speed == left_speed and require_right_speed == right_speed:
        change_speed(0,0)
    left_speed = 0
    right_speed = 0



init()
pub = rospy.Subscriber('wheel',wheel,callback_wheel)



GPIO.add_event_detect(MH1,GPIO.RISING,callback=my_callback_l) #在引脚上添加上升临界值检测再回调

GPIO.add_event_detect(MH2,GPIO.RISING,callback=my_callback_r)

timer_period = rospy.Duration(0.1)

tmr = rospy.Timer(timer_period, timer_callback)
pa = GPIO.PWM(ENA, 2000.0)
pb = GPIO.PWM(ENB, 2000.0)
pa.start(0.0)
pb.start(0.0)
rospy.spin()
