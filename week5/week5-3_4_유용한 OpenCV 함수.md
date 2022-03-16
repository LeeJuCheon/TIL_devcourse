# 유용한 OpenCV 함수

1. OpenCV 함수
* 행렬의 합
    ```cpp
    Scalar sum(InputArray src);
    ```
    + src : 입력행렬. 1~4 채널
    + 반환값 : 행렬 원소들의 합


* 행렬의 평균
    ```cpp
    Scalar mean(InputArray src, InputArray mask = noArray());
    ```
    + src : 입력행렬. 1~4 채널
    + mask : 마스크 영상
    + 반환 값 : 행렬의 평균값

* 행렬의 최댓값/ 최솟값 구하기
    ```cpp
    void minMaxLoc(InputArray src, double* minVal, double* maxVal=0, Point* minLoc=0, Point* maxLoc=0, InputArray mask = noArray());
    ```
    + src : 입력영상. 단일 채널
    + minVal,maxVal : 최솟값/최댓값 변수포인터(필요없으면 NULL 지정)
    + minLoc,maxLoc : 최솟값/최댓값 위치변수포인터(필요없으면 NULL 지정)
    + mask : 마스크 영상. mask 행렬 값이 0이 아닌 부분에서만 연산을 수행

2. 영상의 속성 변환
* 행렬의 자료형 변환
    ```cpp
    void Mat::convertTo(OutputArray m, int rtype, double alpha=1, double beta=0) const;
    ```
    + m : 출력 영상(행렬)
    + rtype : 원하는 출력 행렬 타입
    + alpha : 추가적으로 곱할 값
    + beta : 추가적으로 더할 값

* 행렬의 정규화
    ```cpp
    void normalize(InputArray src, InputOutputArray dst, double alpha =1, double beta = 0, int norm_type = NORM_L2, int dtype = -1, InputArray mask = noArray());
    ```
    + src : 입력 행렬(영상)
    + dst : 출력행렬. src와 같은 크기
    + alpha : (노름 정규화인 경우) 목표 노름(norm) 값, (NORM_MINMAX인 경우) 최솟값
    + beta : (NORM_MINMAX인 경우) 최댓값
    + norm_type : 정규화 타입. NORM_INF, NORM_L1, NORM_L2, NORM_MINMAX 중 하나를 지정. NORM_MINMAX를 지정할 경우, 출력 행렬 dst의 최솟값은 alpha, 최댓값은 beta가 되도록 설정함
    + dtype : 출력 행렬의 타입
    + mask : 마스크 영상

* 색 공간 변환 함수
    ```cpp
    void cvtColor(InputArray src, InputOutputArray dst, int code, int dstCn = 0);
    ```
    + src : 입력 영상
    + dst : 출력 영상
    + code : 색변환 코드
    + dstCn : 결과 영상의 채널 수. 0이면 자동 결정

* 채널 분리
    ```cpp
    void split(const Mat& src, Mat* mybegin);
    void split(InputArray src, OutputArrayOfArrays mv);
    ```
    + src : (입력) 다채널 행렬
    + mvbegin : (출력) Mat 배열의 주소
    + mv : (출력) 행렬의 벡터. vector<Mat>

* 채널 결합
    ```cpp
    void merge(const Mat& mv, size_t count, OutputArray dst);
    void merge(InputArrayOfArrays mv, OutputArray dst);
    ```
    + mv : (입력) 1채널 Mat 배열 또는 행렬의 벡터
    + count : (mv가 Mat 타입의 배열인 경우) Mat 배열의 크기
    + dst : (출력) 다채널 행렬