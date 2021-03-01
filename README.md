# state_estimation
state estimate. observer, kalman filter, etc

## DC-Motor Full State Observer

-DC Motor + Controller + Full State Observer

Full state feedback(Pole Placement)를 적용한 system에 Full state observer를 추가
observer의 pole은 controller의 6배 크기로 설정하였음
process noise, measurement noise 는 gaussian noise
observer는 noise에 취약한 경향을 보여줌. pole을 수정하면 noise의 영향을 덜 받겠으나, 움직임이 느려질 것
일정 주기마다 이동 거리(x1) 데이터를 감소시키면 관측기의 거동이 커지며, 실제 움직임에서 크게 벗어나는 모습을 볼 수 있음

--noise를 입력하지 않음

<img src="https://user-images.githubusercontent.com/54099930/109395502-1d4fd000-7970-11eb-9760-8408f8072539.jpg" width="680">

--process noise + measurement noise

<img src="https://user-images.githubusercontent.com/54099930/109395507-20e35700-7970-11eb-9e79-91d511b4b7fa.jpg" width="680">

--추가로 impulse noise 입력

<img src="https://user-images.githubusercontent.com/54099930/109395510-2476de00-7970-11eb-98bb-7851dff371d3.jpg" width="680">

## DC-Motor Kalman Filter

-DC Motor + Controller + Kalman Filter

Full state feedback(Pole Placement)를 적용한 system에 Kalman filter 적용.
모터의 스펙과 제어기의 pole은  observer_motor.m과 동일하게 구현.
필터 구현을 위한 시스템 모델링에서 DC Motor의 electrical circuit modeling, mechanical system modeling은 theta가 아닌 omega를 기준으로 진행하였음 차분 식을 활용하여 matrix 유도. theta는 등속도 운동으로 가정하였음. 필터의 성능을 그래프로 확인하기 위해 process noise와 measurement noise가 없다고 가정한 시스템의 움직임도 계산함.
process noise, measurement noise 는 gaussian noise.
Kalman Filter는 gaussian noise에 강인한 모습을 보이나, x1에 impulse noise 를 입력하면 발산하는 모습을 보임.

--process noise와 measurement noise를 입력했을 때의 결과

<img src="https://user-images.githubusercontent.com/54099930/109395209-7dde0d80-796e-11eb-9953-f566dbfcbf79.jpg" width="450">
<img src="https://user-images.githubusercontent.com/54099930/109395212-7fa7d100-796e-11eb-9dde-05b79c14357d.jpg" width="450">
<img src="https://user-images.githubusercontent.com/54099930/109395218-859db200-796e-11eb-91a1-10dac20ecc36.jpg" width="450">

--추가로 impulse noise를 입력했을 때의 결과

<img src="https://user-images.githubusercontent.com/54099930/109395223-89313900-796e-11eb-9b7d-ef57ab257662.jpg" width="680">

## Kalman Filter with Constant Velocity Model

2021 캡스톤 설계에서 anti-drone을 만들고자 하는데, 비전을 통해 드론의 위치를 추적하고  한다.
이 때 비전에서 입력받은 드론의 pose를 신뢰할 수 없다. 실시간성이 보장되지 않으며, 알 수 없는 노이즈가 추가되기 때문이다. 정확한 위치를 파악하지 못하면 드론을 타격할 수 없다.
이러한 점을 극복하고자 한 가지 조건을 추가하고, Kalman Filter를 사용한다.
드론의 움직임은 Constant Velocity로 제한한다. 드론이 무작위의 속도로 구동된다면 정확한 모델을 구할 수 없다. 잘못된 모델링은 필터의 잘못된 추정으로 이어질 것이다. 따라서 등속도 운동으로 제한하여 정확한 모델을 구한다.

### 1 Dimension Constant Velocity Model

드론의 velocity는 desired velocity를 바로 만족하지 못한다.
여기서는 tau*v_dot + v = v_des 로 설정하였다.

<img src="https://user-images.githubusercontent.com/54099930/109472729-d8ce4c80-7ab5-11eb-9e54-6ce9ac46f4e7.png" alt="원본 신호와 Filter의 출력 신호를 비교." width="680">

<img src="https://user-images.githubusercontent.com/54099930/109472749-dc61d380-7ab5-11eb-97cf-6ab78f79c5e6.png" alt="noise가 입력된 신호와 Filter의 출력 신호를 비교." width="680">

### 2 Dimension Constant Velocity Model

차원이 증가함에 따라 matrix의 크기 역시 증가해야 한다. 이 때 matrix를 새로 정의할 필요 없이 Jordan form 처럼 y축에 대한 성분을 추가해주면 된다. 1-dim에 추가한 y 성분은 v = v_des로 설정하였다.

<img src="https://user-images.githubusercontent.com/54099930/109473456-a5d88880-7ab6-11eb-8173-fd99d39aaf6e.png" width="680">


