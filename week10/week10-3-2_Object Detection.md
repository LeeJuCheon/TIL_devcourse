# Object Detection

1. 객체 탐지 개요
* Classification과 Localization을 모두 수행하는 것을 지칭
* Classifier는 객체 분류, Regressor는 좌표 연산

2. Object Detection 주요 내용
* One-Stage 기법
  * SSD, YOLO
  * 빠른 속도, 낮은 성능
  * Backbone : Feature extractor
    * 이미 pre-trained 된 모델
    * 추상화 진행
  * Neck : Merge the different resolution feature maps
    * concat 혹은 add를 통해 feature map 생성
      * concat은 채널 상관 X, add는 채널이 동일해야 함
    * Backbone에서 추출된 high-level feature map은 semantic 정보와 큰 객체를 잘 표현한다.
    * Neck에서는 high-level feature map을 low-level로 만들어주고 해당 데이터는 선,점,기울기 등의 정보와 작은 객체를 잘 표현한다

  * Dense Preditcion : 예측값, bbox 정의
    * Regression layers
  
* Two-Stage 기법
  * Faster-RCNN
    * 두 번의 forward
    * Object의 위치 Region을 생성, 후보 영역
    * 각 후보의 classification 진행
  * 느린 속도, 좋은 성능

* Grid
  * 그리드 내의 정보
    * [Box Co-ordinates, Objectness Score, Class Scores]로 구성

* Anchor
  * 사전에 정의된 bbox

* Softmax
  * 각 Classification의 예측 값을 total_sum=1이 되도록 정규화

* IOU(intersection over union)
  * IOU = intersection / union
  * default, IOU>0.5면 positive box, threshold 값은 변경 가능

* NMS(Non-Maximum Suppression)
  * 중복된 bbox를 방지하기 위한 기법
  * IOU와 confidence score를 사용하여 필터링

* Data annotation
  * 데이터 bbox 직접 정의에 대한 내용
  * 데이터의 truncated(일부 잘린현상), occluded(가린현상)등을 주의해야함 (일부 데이터셋에는 해당 내용에 대한 값들이 존재하기도 함)

> 다운 링크
> > labeling tool : https://github.com/tzutalin/labelImg
> > KITTI dataset : http://www.cvlibs.net/datasets/kitti/eval_object.php?obj_benchmark