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
    
    double position_x[900];
    double position_x_noise[900];
    double velocity_x[900];
    double velocity_x_noise[900];
    double position_y[900];
    double position_y_noise[900];
    double velocity_y[900];
    double velocity_y_noise[900];
    double pos_x[900];
    double vel_x[900];
    double pos_y[900];
    double vel_y[900];
    double temp = 0;
    MatrixXd A(4,4);
    MatrixXd H(2,4);
    MatrixXd Q(4,4);
    MatrixXd R(2,2);
    MatrixXd x_hat(4,1);
    MatrixXd x_tilde(4,1);
    MatrixXd P_hat(4,4);
    MatrixXd P_tilde(4,4);
    MatrixXd K(4,4);
    MatrixXd z(2,1);

    A << 1, dt, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, dt,
         0, 0, 0, 1;
    H << 1, 0, 0, 0,
         0, 0, 1, 0;
    Q << (double)(0.00000001/4), (double)(0.000001/2), 0, 0,
         (double)(0.000001/2), 0.0001, 0, 0,
         0, 0, (double)(0.00000001/4), (double)(0.000001/2),
         0, 0, (double)(0.000001/2), 0.0001;
    x_hat << 0, 10, 0, 5;
    R << 1, 0, 0, 0.8;
    P_hat << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    
    int t = 0;
    
    Q = 9*Q;

    for(t = 0; t < 900 ; t++) {
        //위치 데이터 생성 및 가우시안 노이즈 추가
        if(t < 400){
            position_x[t] = 10*dt*t;
            position_y[t] = 5*dt*t;
        }
        else if(t < 600){
            position_x[t] = (double)(-1.25*pow(dt*t,2)+20*dt*t-20);
            position_y[t] = 20;
        }
        else{
            position_x[t] = 5*dt*t + 25;
            position_y[t] = (double)(-20*dt/3)*t + 60;
        }
        
        if(t == 0) {
            temp = dist_w(gen);
            position_x_noise[t] = position_x[t] + temp;
            position_y_noise[t] = position_y[t] + temp;
        }
        else{
            position_x_noise[t] = position_x[t] + 1.1*dist_w(gen);
            position_y_noise[t] = position_y[t] + 0.9*dist_w(gen);
        }
        
        if(t == 0) {
            velocity_x[t] = 10;//(double)position[t]/dt;
            velocity_x_noise[t] = (double)10 + temp/dt;
            velocity_y[t] = 5;//(double)position[t]/dt;
            velocity_x_noise[t] = (double)5 + temp/dt;
        }
        else {
            velocity_x[t] = (double)(position_x[t]-position_x[t-1])/dt;
            velocity_x_noise[t] = (double)(position_x_noise[t]-position_x_noise[t-1])/dt;
            velocity_y[t] = (double)(position_y[t]-position_y[t-1])/dt;
            velocity_y_noise[t] = (double)(position_y_noise[t]-position_y_noise[t-1])/dt;
        }
        z(0,0) = position_x_noise[t] + 0.1*dist_v(gen);
        z(1,0) = position_y_noise[t] + 0.09*dist_v(gen);

        x_tilde = A*x_hat;
        P_tilde = A*P_hat*A.transpose() + Q;
        K = P_tilde*H.transpose()*(H*P_tilde*H.transpose() + R).completeOrthogonalDecomposition().pseudoInverse();
        
        x_hat = x_tilde + K*(z - H*x_tilde);
        P_hat = P_tilde - K*H*P_tilde;
        
        pos_x[t] = x_hat(0,0);
        vel_x[t] = x_hat(1,0);
        pos_y[t] = x_hat(2,0);
        vel_y[t] = x_hat(3,0);
        
    }

    string out_line1;
    string out_line2;
    string out_line3;
    string out_line4;
    string out_line5;
    string out_line6;
    string out_line7;
    string out_line8;
    string out_line9;
    string out_line10;
    string out_line11;
    string out_line12;
    string out_line13;
    ofstream out("/Users/byeongilham/Documents/C/cv2.txt");
    for(t = 0 ; t < 900 ; t++) {

        out_line1 = to_string(position_x[t]);
        out_line2 = to_string(position_y[t]);
        out_line3 = to_string(position_x_noise[t]);
        out_line4 = to_string(position_y_noise[t]);
        out_line5 = to_string(pos_x[t]);
        out_line6 = to_string(pos_y[t]);
        out_line7 = to_string(velocity_x[t]);
        out_line8 = to_string(velocity_y[t]);
        out_line9 = to_string(velocity_x_noise[t]);
        out_line10 = to_string(velocity_y_noise[t]);
        out_line11 = to_string(vel_x[t]);
        out_line12 = to_string(vel_y[t]);
        out_line13 = to_string(dt*t);
        out<< out_line1 << " " << out_line2 << " " << out_line3 << " " << out_line4 << " " <<out_line5 << " " <<out_line6 << " " <<out_line7 << " " << out_line8 << " " << out_line9 << " " << out_line10 << " " << out_line11 << " " << out_line12 << " " << out_line13 << " " << '\n';
    }
    out.close();
   
    return 0;
}
