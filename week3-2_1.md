# 터틀심 8자 주행

> 실습과정

1) 마스터(roscore)의 실행 [terminal 1]
```bash
$roscore
```

2) ROS node 실행 [terminal 2]
    
```bash
$rosrun turtlesim turtlesim_node
```

3) Rostopic을 활용하여 토픽 분석 [terminal 3]

```bash
$rostopic list
$rostopic type /turtle1/cmd_vel # 타입확인
$rosmsg show geometry_msgs/Twist
$rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
``` 

![](2022-03-01-13-09-36.png)

4) pub8.py 작성 [terminal 3]

```bash
$cp pub.py pub8.py
$vi gedit pub8.py
``` 

```python
#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('my_node', anonymous=True)  # 노드 생성
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) # Publisher 객체 생성 (topic, message format, size)

msg = Twist()
msg.linear.x=2.0
msg.linear.y=0.0
msg.linear.z=0.0
msg.angular.x=0.0
msg.angular.y=0.0
msg.angular.z=1.8
idx=0
rate = rospy.Rate(1)  # 1초에 n번

while not rospy.is_shutdown():  # 종료되지 않으면
    idx+=1
    if idx==4:
        msg.angular.z= -1 * msg.angular.z
        idx=0
    pub.publish(msg)    # publish
    rate.sleep()       

```


4) pub8.py 프로그램 실행 [terminal 4]
    
    ```bash
    $chmod +x pub8.py
    $rosrun my_pkg1 pub8.py
    ```

    ![](2022-03-01-14-04-54.png)