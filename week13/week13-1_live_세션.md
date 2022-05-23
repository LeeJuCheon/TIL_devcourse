16:00 특강

대세 자율주행 오픈소스 플랫폼 AutoWare

자율주행 소프트웨어 풀스택을 처음으로 제공

자율주행 2~3 Level을 겨냥

AI, IO, AUTO 프로젝트

AI : ROS 1 기반 프로젝트
AUTO : ROS 2 기반 프로젝트

아폴로 ? -> 중국의 바이두에서 개발, 오픈소스 배포


Timing - Accrate computing 연구

경주차에서 중요한 요소
Predictability
Localizability
Optimality


Software Stack

Detection Pipeline - GPS/IMU , Lidar, Radar, Camera -> 좌표, 크기
Localization Pipeline - VectorMap, 3D Point Map -> 경로 판단
Planning & Control Pipeline

제어 기법 - Model Predictive Control(MPC)
보통 Planning and Control 계산을 할 때 이전의 state를 계산에 활용해 왔음
computing의 텀에 진행된 위치를 유추해서 계산 할 수 있어야함 
매순간 조향각, 가속도를 알면 다른 차량 및 본인 차량의 위치를 유추할 수 있음

전장에 제어 신호를 보내면 반응을 함
computing delay와 action delay가 있음

Computing Delay를 자동적으로 보상하는 AV software stack


CarLA랑 SVL 차이 ?

ROS2 vs ROS1 

신뢰성, 
DDN 최적화 과정
실전제품은 ROS2가 좋다




------------------------------------------------------------
19:00 특강

1. 데이터 라벨링
2. 딥러닝 모델 디자인
3. 손실함수 세팅 & 최적 하이퍼 파라미터 도출
4. KPI를 만족하는 평가모델
5. 모델 압축 -> 무거운 모델을 가볍게

자율주행에서는 SOC(System on chip) 사용

Model compression (모델 압축 기법)
1. Light architecture 
reduce MAC(Multiply Add Calculation)
e.g. 필터 size를 5x5 -> 3x3으로 축소
    해상도 축소(1920x1080 -> 1280*720)
    Depth scaling(레이어 축소)
    Width scaling(채널 축소)

2. Pruning(가지치기)
    weight를 지운다는 개념, 0과 가까운 weight를 제거
    SOC에 따라 적용되지 않을 수도 있음(sparse convolution)

3. Quantization(양자화) -> floating point를 줄이는 기법 e.g.(float32 -> float16)
    round-off 에러 : 반올림, 올림 등으로 생기는 오차
    min, max 위치를 어디로 조정할 것인지 중요 -> Calibration
    input value, weight&bias, output

4. BatchNorm folding
    Convolution과 BatchNorm을 수식적으로 합쳐 모델경량화


* ONNX(Open Neural Network Exchange)
* TensorRT : Nvidia에서 가능, 가속화

* TSTL 프로젝트
    * Traffic sign& Traffic light detection
    * 로터리, 정지, 횡단보도신호, 좌회전, 우회전, 신호등(적색,청색)
    1. Data Labeling
        * training data : 766
        * Evaluation data : 194
        * 라벨링 되어있지 않음
        * 라벨링 Policy
    2. Design model & training
        * input resolution, 352 416?
        * Augmentation **** 매우중요!!
        * Tuning loss weight
    3. Model Optimization
        * Pytorch -> Darknet -> ONNX -> TRT 
    4. Make driving algorithm in Xycar 