# [과제] 초음파센서 ROS패키지 확장

* ultrasonic_4_fw.ino
```cpp
/*
HC-SR04 초음파 센서
*/

void setup()
{
  Serial.begin(9600);     // 통신속도 9600bps로 시리얼 통신 시작
  // Serial.println("Start... Ultrasonic Sensor");
  for(int i=1;i<=4;i++){
    pinMode(i*2, OUTPUT);  // 트리거 핀을 출력으로 선언
    pinMode(i*2+1, INPUT);   // 에코핀을 입력으로 선언
  }
}
long distance[4]={0.0, 0.0, 0.0, 0.0};
void loop() { 
  long duration;  // 거리 측정을 위한 변수 선언
  // 트리거 핀으로 10us 동안 펄스 출력
  for(int i=2;i<=8;i+=2){
    digitalWrite(i, LOW);  // Trig 핀 Low
    delayMicroseconds(2);     // 2us 딜레이
    digitalWrite(i, HIGH); // Trig 핀 High
    delayMicroseconds(10);    // 10us 딜레이
    digitalWrite(i, LOW);  // Trig 핀 Low
    // pulseln() 함수는 핀에서 펄스신호를 읽어서 마이크로초 단위로 반환
    duration = pulseIn(i+1, HIGH);
    distance[i/2-1] = duration * 170 / 1000; // 왕복시간이므로 340m를 2로 나누어 170 곱하
  }
  
  Serial.print("Distance(mm):");
  for(int i=0;i<4;i++){
    Serial.print(distance[i]); // 거리를 시리얼 모니터에 출력
    Serial.print(" ");  
  }
  Serial.println("");
  delay(100);
}
```

* ultra_pub.py
```python
#!/usr/bin/env python

import serial, time, rospy, re
from std_msgs.msg import Int32, Int32MultiArray

ser_front = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=9600,
    )

def read_sensor():
    serial_data = ser_front.readline()
    ser_front.flushInput()
    ser_front.flushOutput() 
    ultrasonic_data = serial_data.split(":")[1]
    four_distance = map(int,ultrasonic_data.split(" "))
    msg.data = four_distance
  
if __name__ == '__main__':

    rospy.init_node('ultra4_pub', anonymous=False) # initialize node
    pub = rospy.Publisher('ultrasonic', Int32MultiArray, queue_size=1)

    msg = Int32MultiArray() # message type

    while not rospy.is_shutdown():
        read_sensor() 
        pub.publish(msg) # publish a message
        time.sleep(0.2)
    
    ser_front.close()

```

* ultra_4_sub.py
```python
#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray

def callback(msg):
    print(msg.data)

rospy.init_node('ultra4_sub')
sub = rospy.Subscriber('ultrasonic', Int32MultiArray, callback)

rospy.spin()
```

* ultra4.launch
```html
<launch>
    <node pkg="ultrasonic" type="ultra4_pub.py" name="ultra4_pub"/>
    <node pkg="ultrasonic" type="ultra4_sub.py" name="ultra4_sub" output="screen"/>
</launch> 
```