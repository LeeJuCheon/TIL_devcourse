# Object Detection model evaluation

1. One stage
* YOLO(You look only once)
* SSD(Single Shot Detectors)

2. Two Stage
* R-CNN
* Fast-RCNN
  * ROI Pooling 적용

3. 모델 평가 용어
* Precision(정밀도) : 검출한 범위 중 진짜 맞은 정도
* Recall(재현율) : 모든 정답 중 내가 맞춘 정도 
* F1 score : Precision과 Recall의 조화평균
  * low recall + high precision : bbox를 잘 잡지 못한 상황
  * high recall + low precision : bbox는 제대로 잘 잡았지만 class 정확도가 낮은 상황
* PR curve
  * Confidence가 큰 값을 기준으로 정렬
  * Precision은 점점 감소, recall 값은 점점 증가하는 구조
* mAP(mean average precision)
  * PR curve area, yolo에서 가장 많이 사용하는 지표
* Confusion matrix
  * class 간의 상호관계를 알 수 있는 행렬
  * 각 class의 tp fp 등을 확인할 수 있음

4. Yolo Paper
> arxiv : 논문 게제 홈페이지
* Yolov1
  * 기존의 Two-Stage OD의 속도를 한 번의 eval로 획기적으로 속도를 줄이는 데 초점을 맞춤
  * FP가 적다 : 잘못된 bbox 생성이 적다
* Yolov2
  * 기존 45FPS - > 67FPS 로 향상시킴
  * Batch Normalization 추가
  * FC 레이어 삭제 -> dimension 정보 삭제를 방지
  * k-meams clustering을 통해 box의 크기 결정 -> 더욱 정교한 bbox 형성
  * anchor box를 도입하여 yolo의 불안정성 개선 
  * GoogleNet 대신 Darknet-19 도입
* Yolov3
  * 1개의 anchor에서 하나의 bbox 찾기
  * Darknet-19 -> Darknet 53으로 변경 
  * cfg파일에서 hyperparameter, model architecture, Data augmentation, Input resolution 설정(yaml과 유사)
  * Annotation labeling format은 YOLO format과 PASCAL VOC format이 존재함
  * Yolo Labeling format
    * Class_index x_center y_center width height
    * 0~1의 값으로 정규화 됨

5. Kitti Prepare data
* Convert label format(KITTI to Yolo)
  * https://github.com/ssaru/convert2Yolo