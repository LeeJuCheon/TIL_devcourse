# 자율주행 단계별 요소 기술

1. 자율주행 3단계 프로세스
    1) 인지
    + 경로탐색(디지털 지도) / V2X(차량간 통신) / ADAS 센서(카메라, 레이더 등)
    + 고정지물 인식 및 경로 탐색
    + 변동지물 및 이동물체 인식

    2) 판단
    + 주행상황 판단 및 주행전략 결정
    + 주행경로 생성

    3) 제어
    + 차량제어(엔진 가감속, 조향)

2. 자율주행 SW가 운전하는 방식
    + 보다 정확한 지도가 필요
    + 지도에서 자기의 현재 위치 알아내기 (GPS, 영상 매칭, 라이다 매칭)
    + 목적지까지 경로 찾기
    + 센서를 통한 주변 살피기
    + 상황에 맞는 제어

3. 자율주행에서 필요한 요소기술

    1) 고정밀 지도
    + HD Map(High Definition Map) : 10~20센티 정밀도 제공
    + 다양한 부가정보 포함
        - 차선 정보
        - 가드레일
        - 도로 곡률,경사
        - 신호등 위치
        - 교통 표식

    + MMS(Mobile Mapping System) 을 통해 제작
    <span style='background-color:#fff5b1'>벡터맵 : 차선 중점</span>

    <span style='background-color:#fff5b1'>* 포인트맵 : 레이더를 이용한 3차원 지도</span>
    
    2) Localization 기술
    + 정밀지도와 연동하여 차량의 현재 위치 파악

    3) Global Path Planning (Route Planning)
    + 목적지 까지의 경로 찾기
        - 중간목적지, 교차로에서의 행위 결정
    
    4) Object Detection
    + 주변 차량, 보행자, 오토바이, 자전거 등을 인식

    5) Object Tracking
    + 각 오브젝트에 고유 ID 부여하여 추적, 예상되는 주행경로 예측

    6) Local Path Planning (Trajectory Planning)
    + 다음 이동할 곳으로의 경로 찾기 (충돌회피 고려)
    + 실시간성이 중요하므로 시스템 최적화가 필요

    7) Behavior Selector
    + 행위 결정 (운전 의사결정, 운전 방법, 성향)

    > 제어 파트
    8) Local Path Following (Trajectory Following)
    + 경로 따라 차량을 운전하기

    9) Vehicle Control
    + 주행 제어(원하는 대로 차량을 움직이게)
    + 차량 운동학, 관성, 마찰력, 미끄러짐 모두 고려


4. 자율주행 통합 플랫폼
    1) Autoware
    + https://www.autoware.org
    + SAE - 레벨 2
    + 실차에 적용 가능한 솔루션

5. 자율주행 자동차 구현에 필요한 기술들
    1) 자율주행 알고리즘

        (1) sensing

            GPS
            IMU
            라이다
            초음파
            카메라

        (2) perception

            Localization 자기 위치 파악
            Object Detection (인식)
            Object Tracking (추적)

        (3) decision

            동작 예측
            경로 계획
            장애물 회피 (충돌방지)

    2) 자율주행 클라이언트 시스템
    
        (1) 소프트웨어

            실시간성과 신뢰성 확보 필요
            ROS 문제점 해결 필요
        
        (2) 하드웨어

            성능 향상 필요

        (3) 분산컴퓨팅

            시뮬레이션
            HD맵 생성

        (4) 분산 스토리지

            딥러닝 모델 학습
    
    3) 자율주행 클라우드 플랫폼

    