# Week 15 Day3, 동차좌표계

1. Projective Geometry
* 사영기하학
* 소실점(vanishing point) 에 대한 연구가 많이 진행됨
* 3D의 무한한 공간을 2D로 매핑하면서 숫자로 표현할 수 있음
* Euclidiean
  * Rotation * Translation 표현 가능
* similarity
  * Euclidean transformation에서 uniform scaling 표현 가능
  * Euclidean transformation에서 length 정보 소실
* Affine
  * Similarity transformation 에서 Non-uniform scaling + shear 표현 가능
  * Similarity transformation 에서 Angle / Length-ratio 정보 손실
* Projective 
  * Affine transformation에서 projection 표현 가능
  * Affine transformation에서 incidence, cross-ratio 정보 손실

2. Homogeneous coordinates
* 0이 아닌 scalar lambda 어떤 것을 곱해도 같다
* scale의 값이 추가됨