//  main.cpp
//  Created by Byeongil HAM

#include <iostream>
#include <math.h>
#include <fstream>
#include <string>
#include <random>
#include "Eigen/Dense"

#define PI 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679821480865132823066470938446095505822317253594081284811174502841027019385211055596446229489549303819644288109756659334461284756482337867831652712019091456485669234603486104543266482133936072602491412737245870066063155881748815209209628292540917153643678925903600113305305488204665213841469519415116094
#define dt 0.01

using namespace std;
using Eigen::MatrixXd;

int main(void) {
    ios_base::sync_with_stdio(false);
    
    random_device rd;
    mt19937 gen(rd());
    normal_distribution<double> dist_w(0,1);//정규분포 m,s process noie
    normal_distribution<double> dist_v(0,1);//정규분포 m,s measurement noise
    
    double position[2000];
    double position_noise[2000];
    double velocity[2000];
    double velocity_noise[2000];
    double pos[2000];
    double vel[2000];
    double temp = 0;
    MatrixXd A(2,2);
    MatrixXd H(1,2);
    MatrixXd Q(2,2);
    MatrixXd R(1,1);
    MatrixXd x_hat(2,1);
    MatrixXd x_tilde(2,1);
    MatrixXd P_hat(2,2);
    MatrixXd P_tilde(2,2);
    MatrixXd K(2,2);
    MatrixXd z(1,1);

    A << 1, dt, 0, 1;
    H << 1, 0;
    Q << (double)(0.00000001/4), (double)(0.000001/2), (double)(0.000001/2), 0.0001;
    x_hat << 0, 10;
    R << 1;
    P_hat << 1, 0, 0, 1;
    int t = 0;
    
    Q = 9*Q;

    for(t = 0; t < 2000 ; t++) {
        //위치 데이터 생성 및 가우시안 노이즈 추가
        if(t < 400) position[t] = 10*dt*t;
        else if(t < 600)    position[t] = (double)(-1.25*pow(dt*t,2)+20*dt*t-20);
        else if(t < 900) position[t] = 5*dt*t + 25;
        else if(t < 1100) position[t] = (double)(-2*pow(dt*t,2)+41*dt*t-137);
        else position[t] = -3*dt*t + 105;
        
        if(t == 0) {
            temp = dist_w(gen);
            position_noise[t] = position[t] + temp;
        }
        else    position_noise[t] = position[t] + dist_w(gen);
        if(t == 0) {
            velocity[t] = 10;//(double)position[t]/dt;
            velocity_noise[t] = (double)10 + temp/dt;
        }
        else {
            velocity[t] = (double)(position[t]-position[t-1])/dt;
            velocity_noise[t] = (double)(position_noise[t]-position_noise[t-1])/dt;
        }
        z(0,0) = position_noise[t] + 0.1*dist_v(gen);
        //z(1,0) = velocity_noise[t];

        x_tilde = A*x_hat;
        P_tilde = A*P_hat*A.transpose() + Q;
        K = P_tilde*H.transpose()*(H*P_tilde*H.transpose() + R).completeOrthogonalDecomposition().pseudoInverse();
        
        x_hat = x_tilde + K*(z - H*x_tilde);
        P_hat = P_tilde - K*H*P_tilde;
        
        pos[t] = x_hat(0,0);//H*x_hat;
        vel[t] = x_hat(1,0);
        
    }

    string out_line1;
    string out_line2;
    string out_line3;
    string out_line4;
    string out_line5;
    string out_line6;
    string out_line7;
    ofstream out("/Users/byeongilham/Documents/C/cv1.txt");
    for(t = 0 ; t < 2000 ; t++) {

        out_line1 = to_string(0.01*t);
        out_line2 = to_string(position[t]);
        out_line3 = to_string(position_noise[t]);
        out_line4 = to_string(pos[t]);
        out_line5 = to_string(velocity[t]);
        out_line6 = to_string(velocity_noise[t]);
        out_line7 = to_string(vel[t]);
        out<< out_line1 << " " << out_line2 << " " << out_line3 << " " << out_line4 << " " <<out_line5 << " " <<out_line6 << " " <<out_line7 <<'\n';
    }
    out.close();
   
    return 0;
}
