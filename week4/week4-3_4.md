# [과제] RVIZ기반 IMU뷰어 제작

* imu_generator.py
```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, math,os, time,rospkg

from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler


Imu_msg=Imu()

degrees2rad= float(math.pi)/float(180.0)
rad2degrees = float(180.0)/float(math.pi)

rospy.init_node("Imu_generator")
pub = rospy.Publisher("imu",Imu, queue_size=1)

# imu_data.txt 파일에서 한 줄씩 읽어다가
path =rospkg.RosPack().get_path('rviz_imu')+"/src/imu_data.txt"
rpy= file(path,"r")
lines = rpy.readlines()
rpy_list=[]

for line in lines:
	values=line.split(",")
	roll=float(values[0].split(' : ')[1])
	pitch=float(values[1].split(' : ')[1])
	yaw=float(values[2].split(' : ')[1])
	rpy_list.append([roll,pitch,yaw])

Imu_msg.header.frame_id = 'map'

r = rospy.Rate(10)
seq=0

for i in range(len(rpy_list)):
	line= quaternion_from_euler(rpy_list[i][0], rpy_list[i][1], rpy_list[i][2])
	Imu_msg.orientation.x=line[0]
	Imu_msg.orientation.y=line[1]
	Imu_msg.orientation.z=line[2]
	Imu_msg.orientation.w=line[3]

	Imu_msg.header.stamp=rospy.Time.now()
	Imu_msg.header.seq = seq
	seq = seq +1

	pub.publish(Imu_msg)
	r.sleep()


```

* imu_generator.launch
```html
<launch>
	<!-- rviz display -->
	<node name="rviz_visualizer" pkg="rviz" type="rviz" required="true" 
              args="-d $(find rviz_imu)/rviz/imu_generator.rviz"/>
	<node name="Imu_generator" pkg="rviz_imu" type="imu_generator.py" />
</launch>

```