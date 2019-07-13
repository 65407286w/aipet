# coding=utf-8
import rospy
import time
from datetime import datetime
from std_msgs.msg import Int32
from move_control.msg import *
import pid
import RPi.GPIO as GPIO
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
#设置GPIO模式
GPIO.setmode(GPIO.BOARD)

#设置in1到in4接口

#left side
ENA = 8
#right side
ENB = 7
IN1 = 12
IN2 = 16
IN3 = 18
IN4 = 22

# 接线 左边a 31 b 33 右边 a35 b 37
MH1 = 31
MH2 = 33
MH3 = 35
MH4 = 37
encoder_num=360
wheel_radius=0.021
axle_length=0.09
counter_r=0
counter_l=0
cr=0
cl=0
unit_length = 2 * math.pi * wheel_radius / encoder_num

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
l_dc=50.0
r_dc=50.0
l_count=0
r_count=0
current_time = rospy.Time.now()
last_time = rospy.Time.now()
rospy.init_node('move_base')
action_finish = rospy.Publisher('actionfinish',actionfinish,queue_size=1)
odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()
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
    global cl, cr
    global l_count,r_count
    global l_pid, r_pid
    global l_p, r_p, l_dc, r_dc
    #l_pid.clear()
    #r_pid.clear()
    direction_l = int(msg.leftspeed)
    direction_r = int(msg.rightspeed)
    l_count= int(msg.l_count)
    r_count= int(msg.r_count)
    #print direction_l,direction_r
    l_pid.SetPoint = abs(direction_l)
    r_pid.SetPoint = abs(direction_r)

    if direction_l==0 and direction_r ==0:
        print (location_x,location_y,location_th)
        #print cl,cr
    cl = 0
    cr = 0
    if direction_l>0:
        direction_l=1
    if direction_l<0:
        direction_l=-1
    if direction_r>0:
        direction_r=1
    if direction_r<0:
        direction_r=-1
    l_dc=10
    r_dc=10
    l_p.ChangeDutyCycle(l_dc)
    r_p.ChangeDutyCycle(r_dc)



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
                l_d=-1
            else:
                l_d=1
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
        if r_time - r_time_last > 0.1:

            r_d = 0

        if (r_d != 0 and (r_a - r_b != r_a_last - r_b_last)):
            r_k += 1
        if (r_k > 2):
            r_d = 0
        if r_d == 0:
            r_k = 0
            if r_a == r_b:
                r_d = -1
            else:
                r_d = 1
            r_a_last = r_a
            r_b_last = r_b

        counter_r += r_d
        r_time_last = r_time



def timer_callback(self):
    global counter_l
    global counter_r
    global cl
    global cr
    global location_x
    global location_y
    global location_th
    global direction_l
    global direction_r
    global r_a_last
    global r_b_last
    global l_a_last
    global l_b_last
    global l_pid, r_pid
    global l_p, r_p, l_dc, r_dc
    global l_count,r_count
    global current_time, last_time
    # if direction_l ==0 and direction_r ==0:
    #     return





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

    last_time= current_time
    current_time = rospy.Time.now()
    dt = (current_time - last_time).to_sec()

    inc_l = counter_l * unit_length
    inc_r = counter_r * unit_length
    th = (inc_l-inc_r) / axle_length
    vx=(inc_l+inc_r)/2.0*unit_length/dt
    vy=0
    vth=th/dt
    location_th += th
    if location_th>math.pi:
        location_th -=2 * math.pi
    if location_th<-math.pi:
        location_th += 2 * math.pi
    location_x += math.cos(location_th) * (inc_l + inc_r) / 2
    location_y += math.sin(location_th) * (inc_l + inc_r) / 2
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
    odom_broadcaster.sendTransform(
        (location_x, location_y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom.pose.pose = Pose(Point(location_x, location_y, 0.), Quaternion(*odom_quat))
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
    # publish the message
    odom_pub.publish(odom)
    #print (counter_l,counter_r)
    #if direction_l != 0 or direction_r != 0:
    if l_count!=0 or r_count!=0:
        cr+=counter_r
        cl+=counter_l
        if (direction_l!=0 or direction_r!=0) and (abs(cr)>abs(r_count) or abs(cl)>abs(l_count)):
            direction_l = 0
            direction_r = 0
            action_done=actionfinish()
            action_done.x=location_x
            action_done.y=location_y
            action_done.th = location_th
            action_finish.publish(action_done)
    l_pid.update(abs(counter_l))
    r_pid.update(abs(counter_r))
    l_dc+=l_pid.output
    r_dc+=r_pid.output
    if l_dc>90:
        l_dc=90.0
    if r_dc>90:
        r_dc=90.0
    if l_dc<10:
        l_dc=10.0
    if r_dc<10:
        r_dc=10.0
    l_p.ChangeDutyCycle(l_dc)
    r_p.ChangeDutyCycle(r_dc)
    # print l_dc,r_dc
    # print direction_l,direction_r
    # print counter_l, counter_r
    # print "---------------"
    counter_l = 0
    counter_r = 0


init()
pub = rospy.Subscriber('wheel',wheel,callback_wheel)

GPIO.add_event_detect(MH1,GPIO.FALLING,callback=my_callback_l)
#GPIO.add_event_detect(MH2,GPIO.RISING,callback=my_callback_l)
GPIO.add_event_detect(MH3,GPIO.FALLING,callback=my_callback_r) # ,bouncetime=1)
#GPIO.add_event_detect(MH4,GPIO.RISING,callback=my_callback_r)
l_p = GPIO.PWM(ENA, 2000.0)
r_p = GPIO.PWM(ENB, 2000.0)
l_p.start(l_dc)
r_p.start(r_dc)
l_pid=pid.PID(0.1, 0.015, 0.003)
r_pid=pid.PID(0.1, 0.015, 0.003)


timer_period = rospy.Duration(0.03)

tmr = rospy.Timer(timer_period, timer_callback)

rospy.spin()