# [과제] 초음파 센서 기반 장애물 회피 주행

* ultra_gostop.launch
```html
<launch>
	<include file="$(find xycar_motor)/launch/xycar_motor.launch" />
	<node name="xycar_ultra" pkg="xycar_ultrasonic" type="xycar_ultrasonic.py" output="screen"/>
	<node pkg="ultra_drive" type="ultra_gostop.py" name="ultra_driver"/>
</launch>
```

* ultra_gostop.py
```python
#!/usr/bin/env python

import rospy, time
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

ultra_msg = None
motor_msg = xycar_motor()

def callback(data):
    global motor_msg, ultra_msg
    ultra_msg=data.data

def drive_goback():
    global motor_msg, pub
    motor_msg.speed=-10
    motor_msg.angle=0
    pub.publish(motor_msg)


def drive_stop():
    global motor_msg, pub
    motor_msg.speed=0
    motor_msg.angle=0
    pub.publish(motor_msg)

#노드 선언 & 구독과 발행할 토픽 선언
rospy.init_node('ultra_driver')
rospy.init_node('xycar_ultrasonic',Int32MultiArray, callback,queue_size=1)
pub = rospy.publisher('xycar_motor',xycar_motor,queue_size=1)

time.sleep(3)
while not rospy.is_shutdown():
    if ultra_msg[6] > 0 and ultra_msg[6] <10:       # 후진 주행 가정 
        drive_stop()
    else:
        drive_goback()
```

* ultra_drive.launch
```html
<launch>
	<include file="$(find xycar_motor)/launch/xycar_motor.launch" />
	<node name="xycar_ultra" pkg="xycar_ultrasonic" type="xycar_ultrasonic.py" output="screen"/>
	<node pkg="ultra_drive" type="ultra_drive.py" name="ultra_driver"/>
</launch>
```

* ultra_drive.py
```python
#!/usr/bin/env python

import rospy, time
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

ultra_msg = None
motor_msg = xycar_motor()

def callback(data):
    global motor_msg, ultra_msg
    ultra_msg=data.data

def drive_goback():
    global motor_msg, pub
    motor_msg.speed=-10
    motor_msg.angle=0
    pub.publish(motor_msg)


def drive_stop():
    global motor_msg, pub
    motor_msg.speed=0
    motor_msg.angle=0
    pub.publish(motor_msg)

#노드 선언 & 구독과 발행할 토픽 선언
rospy.init_node('ultra_driver')
rospy.init_node('xycar_ultrasonic',Int32MultiArray, callback,queue_size=1)
pub = rospy.publisher('xycar_motor',xycar_motor,queue_size=1)

time.sleep(3)
while not rospy.is_shutdown():
    if ultra_msg[6] > 0 and ultra_msg[6] <10:       # 후진 주행 가정 
        drive_stop()
    else:
        drive_goback()

```