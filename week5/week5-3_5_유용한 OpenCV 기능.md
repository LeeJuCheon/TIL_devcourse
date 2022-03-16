# 유용한 OpenCV 기능

1. 연산 시간 측정
* 대부분의 영상 처리 시스템은 대용량 영상 데이터를 다루고 복잡한 알고리즘 연산을 수행
* 영상 처리 시스템 각 단계에서 소요되는 연산 시간을 측정하고 시간이 오래 걸리는 부분을 찾아 개선하는 시스템 최적화 작업이 필수적
* 그러므로 OpenCV에서도 연산 시간 측정을 위한 함수를 지원
* 기본적인 연산 시간 측정 방법
    ```cpp
    int64 t1 = getTickCount();
    my_func();

    int64 t2 = getTickCount();
    double ms = (t2-t1)*1000 / getTickFrequency();
    ```

* TickMeter 클래스
    + 연산 시간 측정을 위한 직관적인 인터페이스 제공
    + 클래스 내부에서 getTickCount()와 getTickFrequency() 함수를 조합해서 시간을 측정

    + 코드 정의
        ```cpp
        class TickMeter{
            public:
                TickMeter(); //기본생성자
                void start();
                void stop();
                void reset();

                double getTimeMicro() const;
                double getTimeMilli() const;
                double getTimeSec() const;
        }
        ```

2. 마스크 연산과 ROI
* 관심영역 (ROI)
    + Region of Interest
    + 영상에서 특정 연산을 수행하고자 하는 임의의 부분 영역

* 마스크 연산
    + OpenCV는 일부 함수에 대해 ROI 연산을 지원하며, 이때 마스크 영상을 인자로 함께 전달해야함
    + 마스크 영상은 CV_8UC1 타입
    + 마스크 영상의 픽셀값이 0이 아닌 위치에서만 연산이 수행됨
        - 보통 마스크 영상으로는 0또는 255로 구성된 이진 영상을 사용

* 마스크 연산을 지원하는 픽셀 값 복사 함수
    ```cpp
    void Mat::copyTo(InputArray m, InputArray mask) const;
    ```
    + m : 출력 영상. 만약 * this와 크기 및 타입이 같은 m를 입력으로 지정하면 m를 새로 생성하지 않고 연산을 수행. 그렇지 않으면 m를 새로 생성하여 연산을 수행한 후 반환함
    + mask : 마스크 영상. CV_8U. 0이 아닌 픽셀에 대해서만 복사 연산을 수행

* 마스크 연산을 지원하는 픽셀 값 복사 함수(전역 함수)
    ```cpp
    void copyTo(InputArray src, OutputArray dst, InputArray mask);
    ```
    + src : 입력 영상
    + mask : 마스크 영상. CV_8U. 0이 아닌 픽셀에 대해서만 복사 연산을 수행.
    + dst : 출력 영상