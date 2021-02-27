# state_estimation
state estimate. observer, kalman filter, etc

## DC-Motor Full State Observer

-DC Motor + Controller + Full State Observer

Full state feedback(Pole Placement)를 적용한 system에 Full state observer를 추가
observer의 pole은 controller의 6배 크기로 설정하였음
process noise, measurement noise 는 gaussian noise
observer는 noise에 취약한 경향을 보여줌. pole을 수정하면 noise의 영향을 덜 받겠으나, 움직임이 느려질 것
일정 주기마다 이동 거리(x1) 데이터를 감소시키면 관측기의 거동이 커지며, 실제 움직임에서 크게 벗어나는 모습을 볼 수 있음
