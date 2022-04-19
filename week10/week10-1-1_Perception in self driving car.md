# Perception in self driving car

1. 자율주행에서의 인식 기법
> 자율주행 프로세스
> Perception - Localization - Path planning - Control

* Classification
  * 표지판 등에 대한 분류
* Object detection
  * 오브젝트 판단, 위치 정보를 판단
* Semantic segmentation
  * 하나하나의 픽셀값이 어떤 것에 분류되는지 판단
* Instance segmentation
  * 원하는 오브젝트에 대해서만 세그멘테이션
* Depth/Distance estimation
  * 카메라를 사용해서 거리정도를 구하는 기법
* Object tracking 
  * 객체를 추적하여 동일한 객체인지 판단

2. Image Classification
* CNN(Convolutional Neural Network)
  * 이미지 처리에 특화된 신경망
  * Convolution - pooling의 반복
* AlexNet(2012)
  * GPU의 성능향상에 따라 복잡한 딥러닝 연산이 가능해 짐
* VGGNet(2014)
  * 최근 학습용으로 가볍게 적용해보기 위해 주로 사용하는 모델
* ResNet(2015)
  * 파라미터의 수를 획기적으로 줄임
* 사용 데이터 : MNIST 손글씨 데이터

3. Pytorch
* Tensorflow는 Define과 run이 분리되어 있음
* Pytorch에서 Tensor의 정의는 3차원 이상뿐만 아니라 1~2차원의 데이터도 포함한다