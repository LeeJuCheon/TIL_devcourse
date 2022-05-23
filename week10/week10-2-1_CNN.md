# CNN 학습
 
1. Convolution
* Filter는 행렬로 존재하며 수치를 어떻게 적용하냐에 따라 Blur, Sharp 등 이미지에 다양하게 적용될 수 있다.
* 그 수치를 학습을 통해 갱신하며 만드는 것이 Convolution
* Convolution 계산은 matmul이 아닌 mul
* Padding = 0으로 채워진 외곽 범위
* Stride = 필터가 이동하는 간격
* OH = (ih - kh + padding*2) // stride + 1
* OW = (iw - kw + padding*2) // stride + 1
* A shape = [b(batch), ic, ih, iw]
* W shape = [oc, kc, kh, kw]
* B shape = [b, oc, oh, ow]
* MAC : Multiply Accumulation operation(연산량)
* MAC = kw * kc * kh * oc * ow * oh * b
  * for문으로 만들면 7개의 loop, 비효율
* IM2COL & GEMM
  * 행렬을 활용한 계산, 컴퓨터 친화적, 효율적
  * IM2COL
    * n dim 데이터를 2dim 행렬로 변환
    * output channel의 행을 가진 (kh*kw*ic)
    * kh*kw*ic의 행을 가진 oh*ow
    * 연산의 결과 : oc x oh*ow

2. Pooling
* 행렬을 줄이는 기법
  * Max pooling : 범위 내에서 Max값을 반환
  * Average pooling : 범위의 mean값을 반환

3. FC(Fully Connected Layer)
* 2 dim feature map을 1 dim으로 reshape
* 위의 1dim feature map에 대응되는 FC weight를 곱하여 하나의 단일 값으로 산출

4. Activation(활성함수)
* 딥러닝에서는 비선형 함수를 주로 사용
  * sigmoid
  * tanh
  * ReLU
  * LeakyReLU

5. Shallow CNN(깊이가 얇은 CNN)
* batch 1, channel 1, 6x6의 입력값
* Conv1
  * W1 : [1,1,3,3]
  * kernel : 3x3
  * stride : 1
  * padding : 0
  * sigmoid
* Max pool
  * kernel : 2x2
  * stride : 2
  * padding : 0
* FC
  * W2 : [4,1]
  * sigmoid
* Error : L2_norm
* Maxpool에서의 Back Propagation
  * 해당위치가 1의 값


> 궁금한 사항
> > MaxPool에서의 Back Propagation에서는 해당하는 위치 값이 1, 나머지는 0이 부여
> > MeanPool에서의 Back Propagation은 ?