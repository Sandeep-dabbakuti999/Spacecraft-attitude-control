%%The simulink model parameters for spacecraft

J=[1.10 0.05 0.00
   0.05 1.9 -0.01
   0.00 -0.01 1.12];                                                                        %Inertia matrix
Wi=[0.1745 -0.3491 0.2094].';                                                               %Angular velocity
Wf=[0 0 0].';  
quaternion=[-0.8812 0.1687 -0.2926].';                                                          %Quaternion initial
qn=sqrt(1-quaternion(1)^2-quaternion(2)^2-quaternion(3)^2);
q_desired=[-0.06680 0.0481 -0.7296 -0.1385].';                                               %Quaternion final     
ang_rates=[0.1745 -0.3491 0.2094].';
initial_hw=J*Wi;
Qd=[-0.4078 -0.2926 -0.1687
    0.2926 -0.4078 -0.8812
    0.1687 0.8812 -0.4078];
S=[0 0.2094 0.3491
   -0.2094 0 0.1745
   -0.3491 -0.1745 0];
td=[1 0 0].';


A=[zeros(3) zeros(3);0.5*eye(3) zeros(3)];            
B=[inv(J);zeros(3)]; 
Q=0.6*eye(6);
R=0.3*eye(3);  
K=-lqr(A,B,Q,R); 
sim('Satellite_Dynamics_V2')



