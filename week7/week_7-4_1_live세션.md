# live 세션

1. OpenCV 3.0 T-API(Transparent-API)
* 최소한의 소스 코드 변경을 통해 HW 가속 기능을 사용
* Mat과 UMat 상호 변환(copyTo,getMat(getUMat)활용)

    + Mat -> UMat
    ```cpp
    Mat mat = imread("lenna.bmp");

    UMat umat1;
    mat.copyTo(umat1);
    UMat umat2 = mat.getUMat(ACCESS_READ);
    ```

    + UMat -> Mat
    ```cpp
    UMat umat;
    videoCap >> umat;

    Mat mat1;
    umat.copyTo(mat1);

    Mat mat2 = umat.getMat(ACCESS_READ);
    ```

* OpenCL T-API 주의사항
    + UMat을 사용한다고 항상 좋지는 않음
        - 주로 영상을 표현하는 Mat에 대해서만 UMat으로 변환 사용
    + Border 처리 연산시 BORDER_REPLICATE 옵션을 권장
    + Mat::getUMat() 또는 UMat::getMat() 함수 사용시 주의 사항
        + getUMat()함수를 통해 UMat 객체를 생성할 경우, 새로 생성한 UMat 객체가 완전히 소멸한 후 원본 Mat 객체 사용
        ```cpp
        cv::Mat mat1(height, width, CV_32FC1);
        mat1.setTo(0);
        {
            cv:UMat umat1 = mat1.getUMat(cv::ACCESS_READ);
        }


2. 병렬 프로그래밍
* OpenCV에서 지원하는 병렬 프로그래밍 기법
    + Intel TBB(Threading Building Blocks)
    + HPX(High Performance ParalleX)
    + OpenMP(Open Multi-Processing)
    + APPLE GCD(Grand Central Dispatch)
    + Windows RT concurrency
    + Windows concurrency
    + Pthreads

* 병렬 처리용 for 루프
    ```cpp
    void parallel_for_(const Range& range, const ParallelLoopBody& body, double nstriples= -1.)
    void parallel_for_(const Range& range, std::function<void(const Range&)> functor, double nstripes =-1.)
    ```
    + range : 병렬 처리를 수행할 범위
    + body : 함수 객체. ParallelLoopBody 클래스를 상속받은 클래스 또는 C++11 람다 표현식

* ParallelLoopBody 자식 클래스 사용 예제
```cpp
class ParallelContrast : public ParallelLoopBody{
public:
    ParallelContrast(Mat& src, Mat& dst, const float alpha):m_src(src),m_dst(dst),m_alpha(alpha){
        m_dst=Mat::zeros(src.rows, src.cols, src.type());
    }

    virtual void operator()(const Range& range) const{
        for(int r=range.start; r<range.end; r++){
            uchar* pSrc=m_src.ptr<uchar>(r);
            uchar* pDst=m_dst.ptr<uchar>(r);

            for(int x=0;x<m_src.cols;x++){
                pDst[x]=saturate_cast<uchar>((1+m_alpha)*pSrc[x]-128*m_alpha);
            }
        }
    }
    ...
    ParallelContrast& operator = (const ParallelContrast &){
        return *this;
    };
private:
    Mat& m_src;
    Mat& m_dst;
    float m_alpha;
};

int main(){
    Mat src = imread("hongkong.jpg",IMREAD_GRAYSCALE);
    Mat dst;
    parallel_for_(Range(0,src.rows), ParallelContrast(src,dst,1.f));
}
```
* 람다 표현식 사용 예제
```cpp
int main(){
    Mat src = imread("hongkong.jpg",IMREAD_GRAYSCALE);
    Mat dst(src.rows, src.cols, src.type());
    float alpha = 1.f;
    
    parallel_for_(Range(0,src.rows),[&]const Range& range){
        for(int r=range.start; r<range.end; r++){
            uchar* pSrc=m_src.ptr<uchar>(r);
            uchar* pDst=m_dst.ptr<uchar>(r);

            for(int x=0;x<m_src.cols;x++){
                pDst[x]=saturate_cast<uchar>((1+m_alpha)*pSrc[x]-128*m_alpha);
            }
        }
    }
}
```

3. 기타 OpenCV 기능
* 메모리 버퍼로부터 Mat 객체 생성
    + 외부 함수 또는 외부 라이브러리로부터 생성된 영상 데이터 메모리 버퍼가 있을 경우, 해당 메모리 버퍼를 참조하는 Mat 객체를 생성하여 사용 가능
    + Mat 객체 생성 후 OpenCV 라이브러리를 사용할 수 있음
* 룩업테이블
    + 특정 연산에 대해 미리 결과값을 계산하여 배열등으로 저장해 놓은 것
    + 픽셀 값을 변경하는 경우, 256x1 크기의 unsigned char 행렬에 픽셀 값 변환 수식 결과 값을 미리 저장한 후, 실제 모든 픽셀에 대해 ㅐ실제 연산을 수행하는 대신 행렬 값을 참조하여 결과 영상 픽셀 값을 설정
    ```cpp
    void LUT(InputArray src, InputArray lut, OutputArray dst);
    ```
    + 사용예제
        ```cpp
        int main(void){
            Mat src = imread("hongkong.jpg",IMREAD_GRAYSCALE);

            Mat lut(1,256,CV_8U);
            uchar* p =lut.ptr();
            ...
        }
        ```

4. 코너 검출 기법
* 코너의 특징
    + 평탄한 영역(flat) & 에지(edge) 영역은 고유한 위치를 찾기 어려움
    + 코너(corner) 변별력이 높은편, 영상의 이동 및 회전 변환에 강인함

* 해리스
    + 영상 내부 작은 영역이 모든 방향에 대해 변화가 큰 경우 코너로 규정
    + 코너  응답 함수 R을 반환 ->R(x,y)가 충분히 크면 코너로 구분
    + cornerHarris() 함수 사용
* 추적하기 좋은 특징(GoodFeatures to Track)
    + 해리스 코너 검출방법을 기반으로 향상된 방법
    + 비최대 억제 수행
    + 코너 품질 함수를 정의 -> 가장 값이 큰 순서대로 정렬하여 반환
    + goodFeaturesToTrack() 함수 사용
* FAST(Features from Accelerated Segment Test)
    + 주변 16개 픽셀 값 크기를 분석
    + 기준 픽셀보다 충분히 밝거나(>p+t) 또는 충분히 어두운(<p-t) 픽셀이 n개 연속으로 나오면 코너로 인식
    + 해리스, GFTT 방법보다 매우 빠르게 동작


> 최근 직선 찾는 알고리즘 : LineSegmentDetector()