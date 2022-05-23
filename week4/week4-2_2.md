# [과제] RVIZ 3D자동차 주행프로그래밍

> odom_8_drive.py 

``` python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- coding: euckr -*-

# import 필요한 모듈 가져오기
import rospy
import time
from xycar_motor.msg import xycar_motor


motor_control = xycar_motor()

# 노드 새로 만들기
rospy.init_node('driver')

# 토픽의 발행을 준비
pub = rospy.Publisher('xycar_motor',xycar_motor,queue_size=1)


# 토픽을 발행하는 함수 만들기

def motor_pub(angle,speed):
	global pub
	global motor_control
	motor_control.angle=angle
	motor_control.speed=speed
	
	pub.publish(motor_control)

# 차량의 속도는 고정시킨다
speed = 3

# 차량의 조향각을 바꿔가면서 8자로 주행시킨다.

while not rospy.is_shutdown():
	angle = -50
	for i in range(40):
		motor_pub(angle,speed)
		time.sleep(0.1)

	angle = 0
	
	for i in range(30):
		motor_pub(angle,speed)
		time.sleep(0.1)

	angle = 50

	for i in range(40):
		motor_pub(angle,speed)
		time.sleep(0.1)

	angle=0
	for i in range(30):
		motor_pub(angle, speed)
		time.sleep(0.1)

		
```

> rviz_odom.py
```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- coding: euckr -*-

import math
from math import sin, cos,pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import JointState

global Angle

def callback(msg):
	global Angle
	Angle = msg.position[msg.name.index("front_left_hinge_joint")]

rospy.Subscriber('joint_states',JointState, callback)

rospy.init_node('odometry_publisher') # init odometry_publisher node

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50) # odom 토픽 발행 준비

odom_broadcaster = tf.TransformBroadcaster()

current_speed = 0.4 # 속도 초속 40cm
wheel_base =0.2 # 축간 거리는 20cm
x_=0
y_ =0
yaw_=0
Angle = 0

current_time =rospy.Time.now()      # 시간 정보 계산용 변수
last_time = rospy.Time.now()

r=rospy.Rate(30.0)                   # 1초에 한번 루프돌기

while not rospy.is_shutdown():
	current_time = rospy.Time.now()
	dt=(current_time - last_time).to_sec()

	current_steering_angle=Angle
	current_angular_velocity = current_speed * math.tan(current_steering_angle) / wheel_base

	x_dot = current_speed * cos(yaw_)
	y_dot = current_speed * sin(yaw_)
	x_ +=x_dot * dt;
	y_ +=y_dot * dt;
	yaw_ += current_angular_velocity * dt

	odom_quat = tf.transformations.quaternion_from_euler(0, 0, yaw_)

	odom_broadcaster.sendTransform(
    	(x_,y_,0.),               # 위치정보(자세정보)에 대한 발행을 준비
    	odom_quat,              # odom과 base_link를 연결하는 효과
    	current_time,
    	"base_link",
    	"odom"
	)

	odom=Odometry()
	odom.header.stamp = current_time        #Odometry 메시지의 헤더 만들기
	odom.header.frame_id = "odom"

	odom.pose.pose = Pose(Point(x_,y_,0.), Quaternion(*odom_quat))        # Position 값 채우기

	odom.child_frame_id = "base_link"
	odom.twist.twist = Twist(Vector3(current_speed, wheel_base,0),Vector3(0,0,yaw_))                # 속도 값 채우기

	odom_pub.publish(odom)  # /odom 토픽 발행하기

	last_time = current_time    # 잠깐 쉬었다가 루프 처음으로 되돌아감
	r.sleep()

```

> rviz_odom.launch
``` html
<launch>
    <param name="robot_description" textfile="$(find rviz_xycar)/urdf/xycar_3d.urdf"/>
    <param name="use_gui" value="true"/>

    <!-- rviz display -->
    <node name="rviz_visualizer" pkg="rviz" type="rviz" required="true" 
                args="-d $(find rviz_xycar)/rviz/rviz_odom.rviz"/>

    <node name="robot_state_publisher" pkg="robot_state_publisher" 
                type="state_publisher"/>

	<node name="driver" pkg="rviz_xycar" type="odom_8_drive.py" /> 
	<node name="odometry" pkg="rviz_xycar" type="rviz_odom.py" />
    <node name="converter" pkg="rviz_xycar" type="converter.py" />

</launch>

```