function[XF,P]=ExtendendKalmanFilter(QT,WT)
Po=diag([4,2,3,6,1,8]);
Q=diag([2,1.6,4.4,2.3,3.33,5]);
R=diag([5,3,5,6,2,3]);
H=[zeros(3) eye(3)];
Xo=[WT.';QT.'];
simin=[WT.';QT.'];
XF=out.simout;
P=out.simout1;
sim('EKF')
end
