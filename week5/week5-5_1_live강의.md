# live 강의 2일차

1. 히스토그램 스트레칭

* 영상의 히스토그램이 그레이스케일 전 구간에서 걸쳐 나타나도록 변경하는 선형 변환 기법
* 특정 구간에 집중되어 나타난 히스토그램을 마치 고무줄을 늘이듯이 펼쳐서 그레이스케일 범위 전구간에서 히스토그램이 골고루 나타나도록 변환

* 히스토그램 스트레칭 변환 함수
    + 변환 함수의 직선의 방정식
        - 기울기 : 255/(Gmax - Gmin)
        - y 절편 : -(255 x Gmin)/(Gmax-Gmin)

    + dst(x,y) = (src(x,y)-Gmin)/(Gmax-Gmin) x 255

```cpp
#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

int main()
{
	Mat src = imread("lenna.bmp", IMREAD_GRAYSCALE);

	if (src.empty()) {
		cerr << "Image laod failed!" << endl;
		return -1;
	}
	/*
	double gmin, gmax;
	minMaxLoc(src, &gmin, &gmax);

	Mat dst = (src-gmin)*255/(gmax-gmin);
	*/
	Mat dst;
	normalize(src, dst, 0, 255, NORM_MINMAX);

	imshow("src", src);
	imshow("dst", dst);
	waitKey();
}


```

2. [HW] 히스토그램 스트레칭 개선

* 개선된 히스토그램 스트레칭 함수 작성
```cpp
void histogram_stretching_mod(const Mat& src,Mat& dst){
    int hist[256] = {0,};
    //TODO: src 영상 전체를 스캔하면서 히스토그램을 hist에 저장
    
}
```

3. 히스토그램 평활화
* 히스토그램이 그레이스케일 전체 구간에서 균일한 분포로 나타나도록 변경하는 명암비 향상 기법
* 히스토그램 균등화, 균일화, 평탄화
* 히스토그램 평활화를 위한 변환함수
    + 히스토그램 함수 : h(g) = Ng
    + 정규화된 히스토그램 함수 : p(g) = h(g)/wxh
    + 누적분포 함수(cdf) : cdf(g) = sigma(0<=i<g) p (i)
    + 변환 함수 : dst(x,y) = round(cdf(src(x,y))x Lmax)

* 구현 코드
```cpp
```

4. 영상의 산술연산
* 덧셈 연산
    + 두 영상의 같은 위치에 존재하는 픽셀 값을 더하여 결과 영상의 픽셀 값으로 설정
    + 덧셈 겨로가가 255보다 크면 픽셀 값을 255로 설정(포화 연산)

    + dst(x,y) = saturate(src1(x,y)+src2(x,y))

* 가중치 합(weighted sum)
    + 두 영상의 같은 위치에 존재하는 픽셀 값에 대하여 가중합을 계산하여 결과 영상의 픽셀 값으로 설정
    + 보통 a+b = 1 이 되도록 설정 -> 두 입력 영상의 평균 밝기를 유지
    + dst(x,y) = saturate(a x src1(x,y) + b x src2(x,y))

* 평균 연산(average)
    + 가중치를 a = b = 0.5 로 설정한 가중치 합
    + dst(x,y) = 0.5 x (src1(x,y)+src2(x,y))

* 평균 연산의 응용
    + 잡음 제거

* 뺄셈 연산
    + 두 영상의 같은 위치에 존재하는 픽셀 값에 대하여 뺄셈 연산을 수행하여 결과 영상의 픽셀 값으로 설정
    + 뺼셈 결과가 0 보다 작으면 픽셀 값을 0 으로 설정(포화연산)
    + dst(x,y) = saturate(src1(x,y)-src2(x,y))

* 차이 연산
    + 두 입력 연산에 대하여 뺄셈 연산을 수행한 후, 그 절댓 값을 이용하여 결과 영상을 생성하는 연산
    + 뺄셈 연산과 달리 입력 영상 순서에 영향을 받지 않음
    ++ dst(x,y) = |src1(x,y)-src2(x,y)|

* 행렬의 덧셈 & 뺄셈 연산
    ```cpp
    void add(InputArray src1, InputArray src2, OutputArray dst, InputArray mask = noArray(), int dtype = -1);
    void subtract(InputArray src1, InputArray src2, OutputArray dst, InputArray mask=noArray(), int dtype=-1);
    ```
    + src1 : 첫 번째 입력 행렬 또는 스칼라
    + src2 : 두 번째 입력 행렬 또는 스칼라

* 행렬의 가중합 연산
    ```cpp
    void addWeighted(InputArray src1, double alpha,...)
    ```
* 행렬의 차이 연산
    ```cpp
    void absdiff(...)
    ```

* 행렬의 논리 연산
    ```cpp
    void bitwise_and(...)
    void bitwise_or(...)
    void bitwise_xor(...)
    void bitwise_not(...)
    ```
* 사용 예제
```cpp

int main(){
	VideoCaputure cap("../data/base_Camera_dark.avi");
    Mat mask(480,  640, CV_8UC1, Scalar(0));

    vector<Point> pts(4);
    pts[0] = Point(240,280);
    pts[1] = Point(400,280);
    pts[2] = Point(620,440);
    pts[3] = Point(20,440);

    fillPoly(mask,pts,Scalar(255));
    imshow("mask",mask)
    waitKey();

    Mat mask2;
    mask2 = (mask==255);


    Mat frame, gray, dst;
    while (true){
    cap>>frame;
    cvtColor(frame,gray,COLOR_BGR2GRAY);
    bitwise_and(gray, mask, dst);
    imshow("dst",dst);
    if (waitKey(20) == 27)
        break;
    }
}
```


5. 필터링
* 영상에서 필요한 정보만 통과시키고 원치 않는 정보는 걸러내는 작업

* 주파수 공간에서의 필터링
    + 푸리에 변환을 이용하여 영상을 주파수 공간으로 변환하여 필터링을 수행하는 방법

* 공간적 필터링
    + 영상의 픽셀 값을 직접 이용하는 필터링
    + 주로 마스크 연산을 이용
    + OpenCV에서는 공간적 필터링 마스크 크기가 커질 경우 주파수 공간에서의 필터링을 수행

* 다양한 모양과 크기의 마스크
    + 필터링에 사용되는 마스크(mask)는 다양한 크기, 모양을 지정할 수 있지만, 대부분 3x3 정방형 필터를 사용
    + 마스크의 형태와 값에 따라 필터의 역할이 결정됨
        - 영상 부드럽게 만들기
        - 영상 날카롭게 만들기
        - 에지 검출
        - 잡음 제거
    
* 최외각 픽셀 처리
    + BORDER_REFLECT_101
    + BORDER_DEFAULT
* 기본적인 2D 필터링 항수
    ```cpp
    void filter2D(InputArray src,OutputArray dst, int ddepth, InputArray kernel, Point anchor = Point(-1,-1), double delta = 0, int border Type = BORDER_DEFAULT);
    ```
    + src : 입력영상
    + dst : 출력 영상
    + ddepth : 원하는 결과 영상의 깊이를 지정. -1이면 src와 같은 깊이를 사용
    + kernel : 필터 마스크 행렬

* 엠보싱 필터
    + 엠보싱 : 직물이나 종이, 금속판 등에 올록볼록한 형태로 만든 객체의 윤곽 또는 무늬
    + 엠보싱 필터 코드
    ```cpp
    #include <iostream>
    #include "opencv2/opencv.hpp"

    using namespace std;
    using namespace cv;

    int main()
    {
        Mat src = imread("lenna.bmp", IMREAD_GRAYSCALE);

        if (src.empty()) {
            cerr << "Image laod failed!" << endl;
            return -1;
        }

        float data[] = { -1,-1,0,-1,0,1,0,1,1 };
        Mat kernel(3, 3, CV_32FC1, data);

        


        Mat dst = 255 - src;
        filter2D(src, dst, -1, kernel,Point(-1,-1),128);


        imshow("src", src);
        imshow("dst", dst);
        waitKey();
    }

    ```



* 평균 값 필터
    + 영상의 특정 좌표 값을 주변 픽셀 값들의 산술 평균으로 설정
    + 픽셀들 간의 그레이 스케일 값 변화가 줄어들어 날카로운 에지가 무뎌지고, 영상에 있는 잡음의 영향이 사라지는 효과