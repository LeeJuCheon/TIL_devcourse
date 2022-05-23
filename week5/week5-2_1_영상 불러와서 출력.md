# 영상 불러와서 출력하기

> 실습 내용
* 현재 폴더에 있는 lenna.bmp 파일을 불러와서 화면에 출력하는 OpenCV 예제 프로그램
* 편의상 이전 강의에서 작성한 HelloCV 프로젝트에 필요한 소스 코드를 추가
* 가장 기본적인 OpenCV 함수로 구성된 프로그램

1. 소스코드
```cpp
#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

int main()
{
	Mat img = imread("lenna.bmp");	// 파일 불러오기

	if (img.empty()) {		//영상 파일 불러오기가 실패하면 에러메세지 출력 후 종료
		cerr << "Image load failed!" << endl;
		return -1;
	}

	namedWindow("image");		// image라는 이름의 새창
	imshow("image", img);		// img 영상 출력
	waitKey();					// 키보드 입력이 있을 때까지 프로그램 대기, 입력이 있다면 모든 창을 닫고 종료
	destroyAllWindows();

}
```

2. 소스코드 분석
* 영상파일 불러오기
    ```cpp
    Mat imread(const String& filename, int flags =IMREAD_COLOR);
    ```
    + filename : 불러올 영상 파일 이름
    + flags : 영상 파일 불러오기 옵션 플래그
        - IMREAD_UNCHANGED : 영상 속성 그대로 읽기 (e.g. 투명한 PNG 파일 -> 4채널(B,G,R,alpha)영상)
        - IMREAD_GRAYSCALE : 1채널 그레이스케일 영상으로 읽기
        - IMREAD_COLOR : 3채널 BGR 컬러 영상으로 읽기
    + 반환값 : 불러온 영상 데이터(Mat 객체)

* 비어있는 Mat 객체 확인
    ```cpp
    bool Mat::empty() const
    ```
    + 반환 값 : rows,cols,data 멤버 변수가 0이면 true 반환

* 영상 파일 저장하기
    ```cpp
    bool imwrite(const String& filename, InputArray img, const std::vector<int>& params = std::vector<int>());
    ```
    + filename: 저장할 영상 파일 이름. 파일 이름에 포함된 확장자를 분석하여 해당 파일 형식으로 저장됨
    + img : 저장할 영상 데이터(Mat 객체)
    + params : 파일 저장 옵션 지정(속성 & 값의 정수 쌍), 예를 들어, JPG 압출율을 90%로 지정하고 싶으면 {IMWRITE_JPEG_QUALITY, 90}을 지정
    + 반환값 : 정상적으로 저장하면 true, 실패하면 false

* 새 창 띄우기
    ```cpp
    void namedWindow(const String& winname, int flags = WINDOW_AUTOSIZE);
    ```
    + winname : 창 고유 이름, 이 이름으로 창을 구분함
    + flags : 창 속성 지정 플래그
        - WINDOW_NORMAL : 영상 크기가 창 크기에 맞게 지정됨
        - WINDOW_AUTOSIZE : 창 크기가 영상 크기에 맞게 자동으로 변경됨
        - WINDOW_OPENGL : OpenGL 지원

* 창 닫기
    ```cpp
    void destroyWindow(const String& winname);
    void destroyAllWindows();
    ```
    + winname : 닫고자 하는 창 이름
    + 참고사항 : 일반적인 경우 프로그램 종료 시 운영체제에 의해 열려 있는 모든 창이 자동으로 닫힘

* 창 위치 지정
    ```cpp
    void moveWindow(const String& winname, int x, int y);
    ```
    + winname : 창 이름
    + x, y : 이동할 위치 좌표

* 창 크기 지정
    ```cpp
    void resizeWindow(const String& winname, int width, int height);
    ```
    + winname : 창 이름
    + width, heiight : 변경할 창 크기
    + 참고사항 : 윈도우가 WINDOW_NORMAL 속성으로 생성되어야 동작함

* 영상 출력하기
    ```cpp
    void imshow(const Styring& winname, InputArray mat);
    ```

    + winname : 영상을 출력할 대상 창 이름
    + mat : 출력할 영상 데이터(Mat 객체)

    + 영상 출력 방식 
        - 8-bit unsigned: 픽셀 값을 그대로 출력
        - 16-bit unsigned or 32-bit integer : 픽셀 값을 255로 나눠서 출력
        - 32-bit or 64-bit floating-point : 픽셀 값에 255를 곱해서 출력

    + 참고사항
        - 만약 winname에 해당하는 창이 없으면 WINDOW_AUTOSIZE 속성의 창을 새로 만들고 영상을 출력함
        - 실제로는 waitKey()함수를 호출해야 화면에 영상이 나타남

* 키보드 입력 대기
    ```cpp
    int waitKey(int delay = 0);
    ```
    + delay : 밀리초 단위 대기 시간. delay<=0 이면 무한히 기다림
    + 반환 값 : 눌린 키 값. 키가 눌리지 않으면 -1

    + 참고사항
        - waitKey() 함수는 OpenCV 창이 하나라도 있어야 정상 동작함
        - imshow()함수 호출 후에 waitKey()함수를 호출해야 영상이 화면에 나타남
        - 주요 특수키 코드 : ESC ->27, ENTER-> 13, TAB->9


3. Opencv 라이브러리 참고사항
* URL : https://docs.opencv.org

4. [과제] ocvrt.cpp
```cpp
#include <iostream>
#include "opencv2/opencv.hpp"
#include <cstdio>

using namespace std;
using namespace cv;
int main(int argc, char* argv[]) {
	//명령행 인자 개수가 3개보다 작으면 사용법 출력 후 종료
	if (sizeof(argv)<3) {
		printf("Usage: ocvrt.exe <src_image> <dst_image>\n");
		return 0;
	}
	//첫 번째 이미지 파일을 imread() 함수로 읽어서 img 변수에 저장
	Mat img;
	bool ret;
	img = imread(argv[1]);
	if (img.empty()) {
		printf("Image load failed!\n");
		return -1;
	}

	//두 번째 이미지 파일 이름으로 img 영상 저장
	ret = imwrite(argv[2], img);
	//저장이 제대로 되면 ret 변수에 true, 실패하면 false를 저장

	if (ret)
		printf("%s is successfully saved as %s\n", argv[1], argv[2]);
	else
		printf("File save failed!\n");
}
```