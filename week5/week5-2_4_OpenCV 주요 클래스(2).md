# OpenCV 주요 클래스(2)

1. Mat 클래스
* n차원 1채ㅐ널 또는 다채널 행렬 표현을 위한 클래스
    + 실수 또는 복소수 행렬, 그레이스케일 또는 트루컬러 영상, 벡터 필드, 히스토그램, 텐서 등을 표현
* 다양한 형태의 행렬 생성, 복사, 행렬 연산 기능을 제공
* Mat 클래스의 깊이(depth)
    + 행렬 원소가 사용하는 자료형 정보를 가리키는 매크로 상수
    + Mat::depth()함수를 이용하여 참조 
    + 형식 : CV_<bit-depth> {U|S|F}
* Mat 클래스의 채널(channel)
    + 원소 하나가 몇 개의 값으로 구성되어 있는가?
    + Mat::channels()함수를 이용하여 참조

* Mat 클래스의 타입(type)
    + 행렬의 깊이와 채널 수를 한꺼번에 나타내는 매크로 상수
    + Mat::type()함수를 이용하여 참조
    + 형식 : CV_8UC1

2. InputArray와 OutputArray 클래스
* InputArray 클래스
    + 주로 Mat클래스를 대체하는 프록시 클래스로 OpenCV 함수에서 입력 인자로 사용됨
    ```cpp
    typedef const _InputArray& InputArray;
    typedef InputArray InputArrayOfArrays;
    ```

    + 사용자가 명시적으로 _InputArray 클래스의 인스턴스 또는 변수를 생성하여 사용하는 것을 금지

* OutputArray
    + OpenCV 함수에서 출력 인자로 사용되는 프록시 클래스
    
    ```cpp
    typedef const _OutputArray& OutputArray;
    typedef OutputArray OutputArrayOfArrays;
    ```

    + _OutputArray 클래스는 _InpputArray 클래스를 상속받아 만들어졌으며, 새로운 행렬을 생성하는 create() 함수가 정의되어 있음