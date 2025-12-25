%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%Satellite Attitude Control%%%%%%%%%%%%%%%%%%%%%%%
clc
clear All;
close all;
EulerIA=[12 31 -40];                                                       %%%%%%% given euler initial angle
quatIA=eul2quat(EulerIA);                                                  %%%%%%% quaternion desired angle
quat0=[quatIA(2) quatIA(3) quatIA(4)].';                                   %%%%%%% Initial Quaternion
EulerDA=[40 -30 90];                                                       %%%%%%% given euler desired angle
quatDA=eul2quat(EulerDA);                                                  %%%%%%% quaternion desired angle
QD=[quatDA(2) quatDA(3) quatDA(4)].';                                      %%%%%%% Quaternion desire 
Wi=[0.1745 -0.3491 0.2094].';                                              %%%%%%% Angular velocity initial rad/sec
Wd=[0 0 0].';                                                              %%%%%%% Angular velocity desired rad/sec

Wu=[0.5 0.5 0.5].';                                                        %%%%%%% The upper Boundary of Angular velocity rad/sec
Wl=[-0.5 -0.5 -0.5].';
QU=[0.7 0.7 0.7].';                                                        %%%%%%% The lower Boundary of Quaternion
QL=[-0.7 -0.7 -0.7].';                               

J=[1.10 0.05 0.00;0.05 1.90 -0.10;0.00 -0.01 1.12];                        %%%%%%% Moment of inertia
initial_hw=J*Wi;                                                           %%%%%%% Initial Angular moment kg.m2.rad/sec
S_Wi=[0 -0.2094 -0.3491;0.2094 0 -0.1745;0.3491 0.1745 0];                 %%%%%%% rotational vector around wi
td=[1 0 0].';                                                              %%%%%%% Torque disturbance due to gravitational force
Qd=[sqrt(1-quatIA(2)^2-quatIA(3)^2-quatIA(4)^2) quatIA(4) -quatIA(3);-quatIA(4) sqrt(1-quatIA(2)^2-quatIA(3)^2-quatIA(4)^2) quatIA(2);quatIA(3) -quatIA(2) sqrt(1-quatIA(2)^2-quatIA(3)^2-quatIA(4)^2)];  %reduced quaternion for kinematic model


A=[zeros(3) zeros(3);0.5*eye(3) zeros(3)];                 
B=[inv(J);zeros(3)];
Q=6*eye(6);
R=3*eye(3);
k=lqr(A,B,Q,R);                                  



w_new=[0 0 0].';
q_new=quat0;

W=[];
Q=[];
T=[];
dt=0.001;



for t=0:dt:50 
    u=-k*[w_new;q_new];                                                    %%%%%%% LQR Feedback control 
    [w_new,q_new]=StateEstiamtion(w_new,q_new,J,S_Wi,Qd,u,dt,initial_hw);  %%%%%%% StateEstimation Function
    W=[W w_new];
    Q=[Q q_new];
    T=[T u];
    obj=Objective(w_new,q_new,quatDA,Wd);                                  
    u_opt=optimize(obj,w_new,q_new,Wl,QL,Wd,QU);
    w_new=[u_opt(1) u_opt(2) u_opt(3)].';                                  %%%%%%% Estimated angular velocity
    q_new=[u_opt(4) u_opt(5) u_opt(6)].';                                  %%%%%%% Estimated quaternion 
    [Ts,AIMU,MagIMU,GroIMU,QIMU]=InertialMeasurement(dt,w_new,q_new);      %%%%%%% IMU sensor measurement
    [Qb,GW,H,D,I]=Quest(Ts,MagIMU,AIMU,GroIMU);                            %%%%%%% Quest soving wahbas 
    [Xkal,P]=ExtendendKalmanFilter(Qb,GW);                                 %%%%%%% Calling extended kalman filter 
end


time=0:dt:50;
figure
plot(time,Q)
xlabel("TIme")
ylabel("Quaternion")
title("Satellite Attitude control")

figure
plot(time,W)
xlabel("Time")
ylabel("ang_rates")
title("Satellite Attitude control")

figure 
plot(time,T)
xlabel("Time")
ylabel("Torque")



