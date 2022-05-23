# OpenCV 그리기 함수

1. 선 그리기
* 직선 그리기
    ```cpp
    void line(InputOutputArray img, Point pt1, Point pt2, const Scalar& color, int thickness =1, int lineType=LINE_8, int shift = 0);
    ```

    + img : 입출력 영상
    + pt1 : 시작점 좌표
    + pt2 : 끝점 좌표
    + color : 선 색상(또는 밝기)
    + thickness : 선 두께
    + lineType : 선타입. LINE_4. LINE_8, LINE_AA 중 하나를 지정
    + shift : 그리기 좌표 값의 축소 비율
2. 도형 그리기
* 사각형 그리기 함수
    ```cpp
    void rectangle(InputOutputArray img, Rect rec, const Scalar& color, int thickness =1, int lineType=LINE_8, int shift = 0)
    ```
    + img : 입출력 영상
    + rec : 사각형 위치 정보
    + color : 선 색상
    + thickness : 선 두께. 음수(-1)를 지정하면 내부를 채움
    + lineType : 선타입. LINE_4. LINE_8, LINE_AA 중 하나를 지정
    + shift : 그리기 좌표 값의 축소 비율
* 원 그리기 함수
    ```cpp
    void circle(InputOutputArray img, Point center,int radius, const Scalar& color, int thickness =1, int lineType=LINE_8, int shift = 0)
    ```
    + img : 입출력 영상
    + center : 원 중심 좌표
    + radius : 원 반지름
    + color : 선 색상
    + thickness : 선 두께. 음수(-1)를 지정하면 내부를 채움
    + lineType : 선타입. LINE_4. LINE_8, LINE_AA 중 하나를 지정
    + shift : 그리기 좌표 값의 축소 비율
* 다각형 그리기 함수
    ```cpp
    void polylines(InputOutputArray img, InputArrayOfArrays pts, bool isClosed, const Scalar& color, int thickness =1, int lineType=LINE_8, int shift = 0)
    ```
    + img : 입출력 영상
    + pts : 다각형 외곽선 점들의 집합. vector<Point>
    + isClosed : true 이면 시작점과 끝점을 서로 이음(폐곡선)
    + color : 선 색상
    + thickness : 선 두께. 음수(-1)를 지정하면 내부를 채움
    + lineType : 선타입. LINE_4. LINE_8, LINE_AA 중 하나를 지정
    + shift : 그리기 좌표 값의 축소 비율


3. 문자열 출력하기
* 문자열 출력하기
    ```cpp
    void putText(InputOutputArray img, const Scalar& text, 
    Point org, int fontFace, double fontScale, Scalar color, int thickness =1, int lineType=LINE_8, bool bottomLeftOrigin = false);
    ```
    + img : 문자열을 출력할 영상
    + text : 출력할 문자열
    + org : 문자열이 출력될 좌측 하단 시작 좌표
    + fontFace : 폰트 종류. cv::HersheyFonts 참조
    + fontScale : 폰트 크기 지정. 기본 폰트 크기의 배수를 지정
    + color : 문자열 색상
    + thickness : 폰트 두께
    + lineType : 선타입. LINE_4. LINE_8, LINE_AA 중 하나를 지정
    ```
    