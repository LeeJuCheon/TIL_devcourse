# 초음파센서 ROS 패키지 제작

1. 초음파 토픽 (/xycar_ultrasonic 토픽)

    * 타입 : sensor_msgs/Int32MultiArray
    * 구성 :
        std_msgs/MultiArrayLayout layout
            std_msgs/MultiArrayDimension[] dim
            string label
            uint32 size
            uint32 stride
            uint32 data_offset
        
        int32[] data

    
    * 패키지 생성
    ``` bash
    $catkin_create_pkg my_ultra std_msgs rospy
    ```

    * ultra_scan.py 작성
    ```python
    #!/usr/bin/env python

    import rospy
    import time
    from std_msgs.msg import Int32MultiArray

    ultra_msg = None

    def ultra_callback(data):
        global ultra_msg
        ultra_msg = data.data

    rospy.init_node("ultra_node")

    rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ultra_callback)

    while not rospy.is_shutdown():
        if ultra_msg == None:
            continue

        print(ultra_msg)
        time.sleep(0.5)
    ```

    * ultra_scan.launch 작성
    ```html
    <launch>
        <node pkg="xycar_ultrasonic" type="xycar_ultrasonic.py" name="xycar_ultrasonic" output="screen" />
        <node pkg="my_ultra" type="ultra_scan.py" name="my_ultra" output="screen" />
    </launch>
    ```

    * 실행
    ```bash
    $roslaunch my_ultra ultra_scan.launch
    ```
    
2. 아두이노를 사용하여 직접 제작
    * 초음파센서 ROS 패키지 기능
        + 초음파 센서를 제어하여 물체까지의 거리를 알아내고 그 정보를 ROS 토픽으로 만들어 노드들에게 보내줌

    * 초음파 센서
        + 물체로 초음파를 쏘고 반사된 초음파 신호를 감지
        + 처음 초음파를 쏜 시점과 반사파를 수신한 시점을 표시한 펄스 신호를 아두이노에게 전송
        + 물체까지의 거리 = (송신과 수신의 시간차(us)/2) / 29 us (cm)
    * 아두이노
        + 초음파센서가 보내주는 펄스 신호를 받아 분석
        + 초음파를 쏜 시점과 반사파를 받은 시점의 시간차이를 이용해서 물체까지의 거리를 계싼하고 이를 리눅스(ROS)에 알려줌
        + Arduino IDE를 활용
        ```bash
        $cd ~/Downloads/Arduino-1.8.12
        $sudo ./install.sh
        $sudo arduino
        ```

    * 펌웨어 소스코드
    ```cpp
    /*
    HC-SR04 초음파 센서
    */

    #define trig 2  // 트리거 핀 선언
    #define echo 3  // 에코 핀 선언

    void setup()
    {
    Serial.begin(9600);     // 통신속도 9600bps로 시리얼 통신 시작
    // Serial.println("Start... Ultrasonic Sensor");
    pinMode(trig, OUTPUT);  // 트리거 핀을 출력으로 선언
    pinMode(echo, INPUT);   // 에코핀을 입력으로 선언
    }

    void loop() { 
    long duration, distance;  // 거리 측정을 위한 변수 선언
    // 트리거 핀으로 10us 동안 펄스 출력
    digitalWrite(trig, LOW);  // Trig 핀 Low
    delayMicroseconds(2);     // 2us 딜레이
    digitalWrite(trig, HIGH); // Trig 핀 High
    delayMicroseconds(10);    // 10us 딜레이
    digitalWrite(trig, LOW);  // Trig 핀 Low

    // pulseln() 함수는 핀에서 펄스신호를 읽어서 마이크로초 단위로 반환
    duration = pulseIn(echo, HIGH);
    distance = duration * 170 / 1000; // 왕복시간이므로 340m를 2로 나누어 170 곱하
    Serial.print("Distance(mm): ");
    Serial.println(distance); // 거리를 시리얼 모니터에 출력
    delay(100);
    }
    ```

    * pc에서 아두이노 연결확인
        + usb 케이블로 연결한 후 아래 명령으로 인식됐는지 확인
        ```bash
        $lsusb
        ```
    
    * usb 케이블 연결
        + 아두이노와 PC는 물리적으로 USB 케이블로 연결
        + 하지만 내부적으로는 serial통신이 이루어짐

    * 리눅스에서 아두이노 연결 확인
        + Tools 메뉴에서 Board / Processor Port 체크
            - Board " Arduino Nano
            - Procesor "ATmega328P
            - Port "/dev/ttyUSB0"
                or "/dev/ttyACM0"

3. 리눅스 프로그래밍
    * 아두이노가 보내주는 물체까지의 거리정보를 사용하기 좋은 형태로 적절히 가공 후에 토픽에 담아 필요한 노드들에게 Publish

    * 패키지 생성
    ``` bash
    $catkin_create_pkg ultrasonic std_msgs rospy
    ```
    
    * ultrasonic_pub.py 작성
    ```python
    #!/usr/bin/env python

    import serial, time, rospy, re
    from std_msgs.msg import Int32

    ser_front = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate=9600,
        )

    def read_sensor():
        serial_data = ser_front.readline()
        ser_front.flushInput()
        ser_front.flushOutput() 
        ultrasonic_data = int(filter(str.isdigit, serial_data))
        msg.data = ultrasonic_data
    
    if __name__ == '__main__':

        rospy.init_node('ultrasonic_pub', anonymous=False) # initialize node
        pub = rospy.Publisher('ultrasonic', Int32, queue_size=1)

        msg = Int32() # message type
        while not rospy.is_shutdown():
            read_sensor() 
            pub.publish(msg) # publish a message
            time.sleep(0.2)
        
        ser_front.close()
    ```

    * ultrasonic_sub.py 작성
    ```python
    #!/usr/bin/env python

    import rospy
    from std_msgs.msg import Int32

    def callback(msg):
        print(msg.data)

    rospy.init_node('ultrasonic_sub')
    sub = rospy.Subscriber('ultrasonic', Int32, callback)

    rospy.spin()

    ```

    * ultra.launch
    ```html
    <launch>
        <node pkg="ultrasonic" type="ultrasonic_pub.py" name="ultrasonic_pub"/>
        <node pkg="ultrasonic" type="ultrasonic_sub.py" name="ultrasonic_sub" output="screen"/>
    </launch> 
    ```