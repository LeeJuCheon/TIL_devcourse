# [과제] 라이다 기반 장애물 회피 주행

* lidar_gostop.launch
```html
<launch>
    <include file="$(find xycar_motor)/launch/xycar_motor.launch" />
    <include file="$(find xycar_lidar)/launch/lidar_viewer.launch" />
    <node name="lidar_driver" pkg="lidar_drive" type="lidar_gostop.py" output="screen"/>
<launch> 
```

* lidar_gostop.py
```python
#!/usr/bin/env python

import rospy, time
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor

motor_msg = xycar_motor()
distance = []

def callback(data):
    global distance, motor_msg
    distance = data.ranges
      
def drive_go():
    global motor_msg, pub
    motor_msg.speed = 10
    motor_msg.angle = 0
    pub.publish(motor_msg)

def drive_stop():
    global motor_msg, pub
    motor_msg.speed = 0
    motor_msg.angle = 0
    pub.publish(motor_msg)
    
rospy.init_node('lidar_driver')
rospy.Subscriber('/scan', LaserScan, callback, queue_size=1)
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

time.sleep(3)

while not rospy.is_shutdown():
    ok = 0
    for degree in range(0, 60):         # 실제 라이다 범위 : 0~505
        if distance[degree] <= 0.3:
            ok += 1
        if distance[degree+440] <= 0.3:
            ok += 1
        if ok > 4:
            drive_stop()
            break
    if ok <= 4:
        drive_go()
```

* lidar_drive.launch
```html
<launch>
    <include file="$(find xycar_motor)/launch/xycar_motor.launch" />
    <include file="$(find xycar_lidar)/launch/lidar_noviewer.launch" />
    <node name="lidar_driver" pkg="lidar_drive" type="lidar_drive.py" output="screen"/>
<launch> 
```

* lidar_drive.py
```python
#!/usr/bin/env python

import rospy, time
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor

motor_msg = xycar_motor()
distance = []

def callback(data):
    global distance, motor_msg
    distance = data.ranges
      
def drive_go(angle):
    global motor_msg, pub
    motor_msg.speed = 6
    motor_msg.angle = angle
    pub.publish(motor_msg)

rospy.init_node('lidar_driver')
rospy.Subscriber('/scan', LaserScan, callback, queue_size=1)
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

time.sleep(3)

while not rospy.is_shutdown():
    left = 0
    right = 0
    for degree in range(0, 505):
        if degree<=100 and 0.12 < distance[degree] <=0.25 :         # 차량의 좌측의 약 1/5 범위 & 각 포인트의 거리 25cm
            left+=1
        
        elif degree>405 and 0.12 < distance[degree] <=0.25 :        # 차량의 우측의 약 1/5 범위 & 각 포인트의 거리 25cm
            right+=1     

    if left>=right and left>2:                                      # 왼쪽 장애물이 더 많다고 판단하면 우회전
        drive_go(30)
        print("right", left, right)
    elif right>left and right>2:                                    # 오른쪽 장애물이 더 많다고 판단하면 좌회전
        drive_go(-30)
        print("left", left,right)
    else:
        drive_go(0)
        print("str", left,right)
```