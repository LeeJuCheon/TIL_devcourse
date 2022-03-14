# 영상 데이터 구조와 표현 방법

> 영상
* 픽셀이 바둑판 모양의 격자에 나열되어 있는 형태 (2차원 행렬)
* 픽셀 : 영상의 기본 단위, picture element, 화소

<br>

1. 영상 데이터 구조와 특징
* 그레이스케일 영상
    + 흑백 사진처럼 색상 정보가 없이 오직 밝기 정보만으로 구성된 영상
    + 밝기 정보를 256 단계로 표현
    + 0~255 사이의 정수값
    + 그레이스케일 범위 : [0,255] 또는 [0,256)
    ``` cpp
    typedef unsigned char BYTE;     //Windows
    typedef unsigned char uint8_t;     //Linux
    typedef unsigned char uchar;     //OpenCV
    
    ```

* 트루컬러 영상
    + 컬러 사진처럼 다양한 색상을 표현할 수 있는 영상
    + rgb 색 성분을 각각 256 단계로 표현
    + R,G,B 색 성분의 크기를 각각 0~255 범위의 정수로 표현

    + C/C++ 에서의 표현 방법
        - unsigned char 자료형 3개 있는 배열 또는 구조체
        - 3Bytes 사용
    ```cpp
    class RGB{
        unsigned char R;
        unsigned char G;
        unsigned char B;
    }
    ```

2. 영상 데이터 표현 방법
* 정적 2차원 배열의 생성
    ```cpp
    unsigned char a[480][640] {};
    ```
    + unsigned char : 1바이트 사용(0~255 사이의 정수 표현)
    + 2차원 배열 전체 크기만큼의 메모리 공간이 연속적으로 할당됨
    + 단점
        - 배열의 크기를 미리 알고 있어야 함, 따라서 다양한 크기의 영상을 표현하기에 부적절
        - Stack 영역에 메모리 할당 : 대략 1MB까지 할당 가능

* 동적 2차원 배열의 생성
    ``` cpp
    int w=640;
    int h=480;

    unsigned char** p;
    p = new unsigned char* [h];
    for(int i=0;i<h,i++){
        p[i] = new unsigned char[w]{};
    }
    ```
    + 행 단위로만 연속된 메모리 공간이 보장됨
    + 프로그램 동작 중 다양한 크기의 영상을 생성할 수 있음
    + Heap 영역에 메모리 할당 : x86의 경우 2GB 까지 할당 가능(x64의 경우 8TB)

* 간단한 형태의 영상 데이터 저장 클래스
``` cpp
class MyImage{
    public:
        MyImage() : w(0), h(0), data(0) {}
        MyImage(int _w, int _h) : w(_w), h(_h) {
            data = new unsigned charp[w*h] {};
        }
        ~MyImage(){
            if (data) delete[] data;
        }
        unsigned char& at(int x, int y){
            return *(data+y*w+x);
        }
    private:
    int w,h;
    unsigned char * data;
}
```

    
