# coding=utf-8
import rospy
import time
from datetime import datetime
from std_msgs.msg import Int32
from move_control.msg import *
from pid import PID
import RPi.GPIO as GPIO
import math
#设置GPIO模式
GPIO.setmode(GPIO.BOARD)

#设置in1到in4接口

#left side
ENA = 8
#right side
ENB = 7
IN3 = 12
IN4 = 16
IN1 = 18
IN2 = 22

# 接线 左边a 31 b 33 右边 a35 b 37
MH1 = 31
MH2 = 33
MH3 = 35
MH4 = 37
counter_r=0
counter_l=0
cr=0
cl=0
rate_encoder = 2 * math.pi * 21.0 / 360
d=90
location_x=0
location_y=0
location_th=0
direction_l = 0
direction_r = 0
r_a_last =-1
r_b_last =-1
l_a_last =-1
l_b_last =-1
l_d=0
r_d=0
l_time_last=time.time()
r_time_last=time.time()
l_k=0
r_k=0

rospy.init_node('subscriber')
def init():
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)
    GPIO.setup(IN3, GPIO.OUT)
    GPIO.setup(IN4, GPIO.OUT)
    GPIO.setup(ENA, GPIO.OUT)
    GPIO.setup(ENB, GPIO.OUT)
    GPIO.setup(MH1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(MH2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(MH3, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(MH4, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def callback_wheel(msg):    #定义回调函数
    global direction_l
    global direction_r
    global cl
    global cr
    direction_l = int(msg.leftspeed)
    direction_r = int(msg.rightspeed)
    if direction_l==0 and direction_r ==0:
        print (location_x,location_y,location_th)
        print cl,cr

    #print "*****"


def my_callback_l(channel_l):
    global counter_l
    global l_d
    global l_k
    global l_time_last
    global l_a_last
    global l_b_last

    if GPIO.event_detected(MH1):
        l_a=GPIO.input(MH1)
        l_b=GPIO.input(MH2)
        l_time=time.time()
        if l_time-l_time_last>0.01:
            l_d=0

        if (l_d!=0 and (l_a - l_b != l_a_last - l_b_last)):
            l_k+=1
        if (l_k>2):
            l_d=0
        if l_d==0:
            l_k=0
            if l_a == l_b:
                l_d=1
            else:
                l_d=-1
            l_a_last=l_a
            l_b_last=l_b

        counter_l+=l_d
        l_time_last=l_time


def my_callback_r(channel_r):
    global counter_r
    global r_d
    global r_k
    global r_time_last
    global r_a_last
    global r_b_last

    if GPIO.event_detected(MH3):
        r_a = GPIO.input(MH3)
        r_b = GPIO.input(MH4)
        r_time = time.time()
        if r_time - r_time_last > 0.01:

            r_d = 0

        if (r_d != 0 and (r_a - r_b != r_a_last - r_b_last)):
            r_k += 1
        if (r_k > 2):
            r_d = 0
        if r_d == 0:
            r_k = 0
            if r_a == r_b:
                r_d = 1
            else:
                r_d = -1
            r_a_last = r_a
            r_b_last = r_b

        counter_r += r_d
        r_time_last = r_time

k=0

def timer_callback(self):
    global counter_l
    global counter_r
    global cl
    global cr
    global k
    global location_x
    global location_y
    global location_th
    global direction_l
    global direction_r
    global r_a_last
    global r_b_last
    global l_a_last
    global l_b_last


    print counter_l,   counter_r
    if direction_l == 1:
        GPIO.output(IN1, GPIO.HIGH)

        GPIO.output(IN2, GPIO.LOW)
    if direction_l == 0:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
    if direction_l == -1:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)


    if direction_r == 1:
        GPIO.output(IN3, GPIO.HIGH)

        GPIO.output(IN4, GPIO.LOW)
    if direction_r == 0:
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)
    if direction_r == -1:
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)


    inc_l = counter_l * rate_encoder
    inc_r = counter_r * rate_encoder
    th = (inc_r-inc_l) / d
    location_th += th
    if location_th>2*math.pi:
        location_th -=2 * math.pi
    if location_th<0:
        location_th += 2 * math.pi
    location_x += math.cos(location_th) * (inc_l + inc_r) / 2

    location_y += math.sin(location_th) * (inc_l + inc_r) / 2
    #print (counter_l,counter_r)
    counter_l = 0
    counter_r = 0


init()
pub = rospy.Subscriber('wheel',wheel,callback_wheel)

GPIO.add_event_detect(MH1,GPIO.FALLING,callback=my_callback_l)
#GPIO.add_event_detect(MH2,GPIO.RISING,callback=my_callback_l)
GPIO.add_event_detect(MH3,GPIO.FALLING,callback=my_callback_r) # ,bouncetime=1)
#GPIO.add_event_detect(MH4,GPIO.RISING,callback=my_callback_r)



timer_period = rospy.Duration(0.1)

tmr = rospy.Timer(timer_period, timer_callback)
pa = GPIO.PWM(ENA, 2000.0)
pb = GPIO.PWM(ENB, 2000.0)
pa.start(45.0)
pb.start(45.0)
rospy.spin()