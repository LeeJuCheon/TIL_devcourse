# OpenCV 개요와 설치

1. OpenCV 개요
* URL : https://opencv.org
* BSD/Apache 2 license
* 여러 언어& 여러 운영체제와의 호환성이 높음

* OpenCV 의 역사
    + 1998 : Intel주도로 시작하여 이후 오픈소스로 개발
    + 2006 : OpenCV 1.0, C로 구현(함수와 구조체, Ipllmage)
    + 2009 : OpenCV 2.0, C++로 전환(클래스, Mat)
    + 2015 : OpenCV 3.0, 프로젝트 구조 개선, GPU, IPP 활용 확대
    + 2017 : OpenCV 3.3, DNN 모듈 지원
    + 2018 : OpenCV 4.0, C++ 11/C++17 지원. DNN 지원 강화
    + 2021 : OpenCV 4.5.5

2. OpenCV 구성
    * OpenCV 모듈
        + OpenCV 메인 모듈 : 핵심 기능, 널리 사용되는 기능, 기반기능(infrastructure)
        + OpenCV 추가 모듈 : 최신 기능, 널리 사용되지 않는 기능, 특허, HW 의존적 기능(CUDA) 등

    * 주요 메인 모듈
        + core : 행렬, 벡터 등의 OpenCV 핵심 클래스와 연산 함수
        + dnn : 심층 신경망 기능
        + highgui : 영상의 화면 출력, 마우스 이벤트 처리 등의 사용자 인터페이스
        + imgproc : 필터링, 기하학적 변환, 색 공간 변환 등의 영상 처리 기능
        + ml : 통계적 분류, 회기 등의 머신 러닝 알고리즘
        + java, js, python 
    * 주요 추가 모듈
        + cudaXXXXXX : GPU 가동 관련

    
    ※ 영상 입출력 -> 전처리 -> 특징 추출 -> 객체 검출, 영상분할 -> 분석 -> 화면출력/ 최종 판단
    
    해당 기능을 묶어주는 world 사용하여 실습

    * opencv 설치 이후 환경변수 설정 필요