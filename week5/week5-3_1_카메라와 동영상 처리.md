# 카메라와 동영상 처리

1. VideoCapture 클래스
* OpenCV에서는 카메라와 동영상으로부터 프레임을 받아오는 작업을 VideoCapture 클래스 하나로 처리함
* 일단 카메라와 동영상을 여는 작업이 수행되면, 이후에는 맴 프레임을 받아오는 공통의 작업을 수행
* 코드 정의
```cpp
class VideoCapture{
    public:
        VideoCapture();
        VideoCapture(const String& fiilename, int apiPreference = CAP_ANY);
        virtual ~VideoCapture();

        virtual bool open(const String& filename, int apiPreference = CAP_ANY);
        virtual bool open(int index, int apiPreference = CAP_ANY);
        virtual void release();

        virtual VideoCapture& operator >> (Mat& image);
        virtual bool VideoCapture::read(OutputArray image);

        virtual bool set(int propId, double value);
        virtual double get(int propId) const;
}
```
* 카메라 열기
    ```cpp
    VideoCapture::VideoCapture(int index, int apiPreference = CAP_ANY);
    bool VideoCapture::open(int index, int apiPreference = CAP_ANY);
    ```
    + index : 사용할 캡쳐 장치의 ID (camera_id + domain_offset_id)
    시스템 기본  카메라를 기본 방법으로 열려면 0을 지정.
    컴퓨터에 여러대의 카메라가 연결되어 있으면 0,1,... 순서로 지정
    + apiPreference : 선호하는 카메라 처리방법을 지정
    + 반환값 : VideoCapture 생성자는 VideoCapture 객체를 반환
    VideoCaputer::open() 함수는 작업이 성공하면 true를, 실패하면 false를 반환.

* 동영상 파일 열기
    ```cpp
    VideoCapture::VideoCapture(const String& fiilename, int apiPreference = CAP_ANY);
    bool VideoCapture::open(const String& filename, int apiPreference = CAP_ANY);
    ```
    + filename : 동영상 파일 이름, 정지 영상 시퀸스, 비디오 스트림 URL 등

    + apiPreference : 선호하는 동영상 처리 방법을 지정
    + 반환값 : VideoCapture 생성자는 VideoCapture 객체를 반환. VideoCapture::open() 함수는 작업이 성공하면 true를, 실패하면 false를 반환 

* 현재 프레임 받아오기
    ```cpp
    bool VideoCapture::read(OutputArray image);
    VideoCapture& VideoCapture::operator >> (Mat& image);
    ```

    + image : 현재 프레임. 만약 현재 프레임을 받아오지 못하면 비어있는 영상으로 설정됨
    + 반환값 : VideoCapture::read() 함수는 작업이 성공하면 true를, 실패하면 false를 반환
    + 참고사항 : >> 연산자 오버로딩은 내부에서 read() 함수를 재호출하는 래퍼(wrapper)함수임

* 전체 코드
```cpp
#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

int main()
{
	VideoCapture cap; // 생성과 동시에 인자값으로 초기화 가능
	cap.open(0);
//	cap.open("../../data/test_video.mp4");

	if (!cap.isOpened()) {
		cerr << "Camera open failed!" << endl;
		return -1;
	}

    // cap.set(CAP_PROP_FRAME_WIDTH,1280);
    // cap.set(CAP_PROP_FRAME_HEIGHT,720);

	int w = cvRound(cap.get(CAP_PROP_FRAME_WIDTH));
	int h = cvRound(cap.get(CAP_PROP_FRAME_HEIGHT));
	double fps = cap.get(CAP_PROP_FPS);


	cout << "width: " << w << endl;
	cout << "height: " << h << endl;
    cout << "fps: " << fps << endl;

	Mat frame, edge;
	while (true) {
		cap >> frame;		

		if (frame.empty()) {
			cerr << "Empty frame!" << endl;
			break;
		}

		Canny(frame, edge, 50, 150);	//edge 검출
		imshow("frame", frame);
		imshow("edge", edge);

		if (waitKey(10) == 27)
			break;
	}
	
	cap.release();
	destroyAllWindows();
}
```

* 카메라와 동영상 속성 값 참조와 설정
    ```cpp
    double VideoCapture::get(int propID) const;
    bool VideoCapture::set(int propID, double value);
    ```
    + propId: 속성플래그

* 동영상 저장하기(VideoWriter 클래스)

    + OpenCV는 일련의 정지 영상을 동영상 파일로 저장할 수 있는 VideoWriter 클래스를 제공
    + VideoWriter 클래스 정의
        ```cpp
        class VideoWriter{
            public:
                VideoWriter();
                VideoWriter(const String& filename, int fourcc, double fps, Size frameSize, bool isColor = true);
                virtual ~VideoCapture();

                virtual bool open(const String& filename, int fourcc, double fps, Size frameSize, bool isColor =true);
                virtual void release();

                virtual VideoCapture& operator << (const Mat& image);

                virtual bool set(int propId, double value);
                virtual double get(int propId) const;
                ...
        };
        ```
    
    + VideoWriter 클래스 생성자와 VideoWriter::open() 함수
        ```cpp
        VideoWriter::VideoWriter(const String& filename, int fourcc, double fps, Size frameSize, bool isColor = true);
        bool VideoWriter::open(const String& filename, int fourcc, double fps, Size frameSize, bool isColor=true);
        ```
        - filename : 동영상 파일 이름
        - fourcc : 압축 방식을 나타내는 4개의 문자
        - fps : 초당 프레임 수
        - frameSize : 비디오 프레임 크기
        - isColor : 컬러 동영상 플래그. false로 지정하면 그레이스케일 동영상.

    + 코드 예제
    ```cpp
    #include <iostream>
    #include "opencv2/opencv.hpp"

    using namespace std;
    using namespace cv;

    int main()
    {
        VideoCapture cap(0);

        if (!cap.isOpened()) {
            cerr << "Camera open failed!" << endl;
            return -1;
        }

        int  fourcc = VideoWriter::fourcc('X', 'V', 'I', 'D');
        double fps = 30;
        Size sz((int)cap.get(CAP_PROP_FRAME_WIDTH), (int)cap.get(CAP_PROP_FRAME_HEIGHT));

        cout << "FPS = " << fps << endl;
        cout << "Size = " << sz << endl;

        VideoWriter output("output.avi", fourcc, fps, sz);

        if (!output.isOpened()) {
            cerr << "output.avi open failed!" << endl;
            return -1;
        }

        int delay = cvRound(1000 / fps);
        Mat frame, edge;
        while (true) {
            cap >> frame;
            if (frame.empty())
                break;

            Canny(frame, edge, 50, 150);
            cvtColor(edge, edge, COLOR_GRAY2BGR);

            output << edge;

            imshow("frame", frame);
            imshow("edge", edge);

            if (waitKey(delay) == 27)
                break;
        }

        cout << "output.avi file is created!!!" << endl;
        
        output.release();
        cap.release();
        destroyAllWindows();
    }

    ```