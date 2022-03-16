# 이벤트 처리하기

1. 키보드 이벤트 처리하기
    ```cpp
    int waitKey(int delay = 0);
    ```
    + delay : 밀리초 단위 대기 시간. delay <= 이면 무한히 기다림
    + 반환 값 : 눌린 키 값. 키가 눌리지 않으면 -1 

    + 참고사항
        - waitKey() 함수는 OpenCV 창이 하나라도 있어야 정상 동작함
        - imshow() 함수 호출 후에 waitKey() 함수를 호출해야 영상이 화면에 나타남
        - 주요 특수키 코드  : ESC -> 27 ENTER -> 13, TAB -> 9
        - 화살표키, 함수키 등의 특수키에 대한 이벤트 처리를 수행하려면 WaitKeyEx() 함수를 사용

2. 마우스 이벤트 처리하기
    ```cpp
    void setMouseCallback(const String& winname, MouseCallback  onMouse, void* userdata = 0);
    ```
    + winname : 창 이름
    + onMouse : 마우스 콜백 함수 이름
        ``` cpp
        typedef void (*MouseCallback)(int event, int x, int y, int flags, void* userdata);
        ```
        - event : 마우스 이벤트 종류. MouseEventTypes 상수
        - x, y : 마우스 이벤트 발생 좌표
        - flags : 마우스 이벤트 플래그. MouseEventFlags 상수
        - userdata: setMouseCallback() 함수에서 지정한 사용자 지정 데이터

    + userdata: 콜백 함수에 전달할 사용자 지정 데이터(optional)
* 코드 예제
```cpp
#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

Mat src;
Point ptOld;
void on_mouse(int event, int x, int y, int flags, void*);

int main(void)
{
	src = imread("lenna.bmp");

	if (src.empty()) {
		cerr << "Image load failed!" << endl;
		return -1;
	}
	
	namedWindow("src");
	setMouseCallback("src", on_mouse);

	imshow("src", src);
	waitKey();
}

void on_mouse(int event, int x, int y, int flags, void*)
{
	switch (event) {
	case EVENT_LBUTTONDOWN:
		ptOld = Point(x, y);
		cout << "EVENT_LBUTTONDOWN: " << x << ", " << y << endl;
		break;
	case EVENT_LBUTTONUP:
		cout << "EVENT_LBUTTONUP: " << x << ", " << y << endl;
		break;
	case EVENT_MOUSEMOVE:
		if (flags & EVENT_FLAG_LBUTTON) {
			//cout << "EVENT_MOUSEMOVE: " << x << ", " << y << endl;
			//circle(src, Point(x, y), 2, Scalar(0, 255, 255), -1, LINE_AA);
			line(src, ptOld, Point(x, y), Scalar(0, 255, 255), 3, LINE_AA);
			ptOld = Point(x, y);
			imshow("src", src);
		}
		break;
	default:
		break;
	}
}

```

3. 트랙바 사용하기
    + 영상 출력 창에 부착되어, 프로그램 동작 중에 사용자가 지정된 범위 안의 값을 선택할 수 있는 GUI
    + 슬라이더 컨트롤

* 트랙바 생성 함수
    ```cpp
    int createTrackbar(const String& trackbarname, const String& winname, int* value, int count, TrackbarCallback onChange = 0, void* userdata = 0); 
    ```
    + trackbarname : 트랙바 이름
    + winname : 트랙바를 생성할 창 이름
    + value : 트랙바 위치 값을 받을 정수형 변수의 주소
    + count : 트랙바 최대 위치(최소 위치는 항상 0)

    + onChange : 트랙바 위치가 변경될 때 마다 호출되게 만들 콜백함수 이름
    + userdata : 트랙바 콜백 함수에 전달할 사용자 데이터의 포인터
    + 반환값 : 정상 동작하면 1, 실패하면 0

* 코드 예제
```cpp
#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

void on_level_change(int pos, void* userdata);

int main(void)
{
	Mat img = Mat::zeros(400, 400, CV_8UC1);

	namedWindow("image");
	createTrackbar("level", "image", 0, 16, on_level_change, (void*)&img);

	imshow("image", img);
	waitKey();
}

void on_level_change(int pos, void* userdata)
{
	Mat img = *(Mat*)userdata;

	img.setTo(pos * 16);
	imshow("image", img);
}
```