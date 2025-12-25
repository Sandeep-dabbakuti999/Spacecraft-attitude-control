function[tnumsamp,accread,magread,gyroread,Quaternion]=InertialMeasurement(dt,wdot,qnion)
Fs=100000;                                                                 %%%%%%%%% frequency of a sample 
tnumsamp=dt*Fs;                                                            %%%%%%%%% total number of samples
traj = kinematicTrajectory('SampleRate',Fs);
accread = zeros(tnumsamp, 3);                                              %%%%%%%%% Acceleration of a spacecraft to number of samples
angV=repmat(wdot.',tnumsamp,1);                                            %%%%%%%%% The angular velocity for given timesamples                   
[~,orientationNED,~,accNED,angVelNED]=traj(accread,angV);                  %%%%%%%%% Trajectory for estimating orientation 
IMU=imuSensor("accel-gyro-mag",'SampleRate',Fs);                           %%%%%%%%% imusensor with acceleration,gyro,and magnetometer
IMU.Accelerometer=accelparams("MeasurementRange",19.62,"Resolution",0.0023936,"TemperatureScaleFactor",0.08,"ConstantBias",1,"TemperatureBias",0.00147,"NoiseDensity",0.00123);
IMU.Magnetometer=magparams("MeasurementRange",1200,"Resolution",0.1,"TemperatureScaleFactor",0.1,"ConstantBias",1,"TemperatureBias",[0.8 0.8 2.4],"NoiseDensity",[0.6 0.6 0.9]/sqrt(100));
IMU.Gyroscope = gyroparams("MeasurementRange", 250, "Resolution", 0.01, "ConstantBias", [0.1 0.1 0.1], "NoiseDensity", [0.01 0.01 0.01]);
[accread,gyroread,magread]=IMU(accNED,angVelNED,orientationNED);
Quaternion=ecompass(accread,magread);
end