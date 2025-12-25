function[ABQ,Gimu,H,D,I]=Quest(tSamp,MagIMU,AIMU,GyrIMU)

Aimu=sum(AIMU)/tSamp;                                                      %%%%%%%% Processing input IMU values for Acc
Mimu=sum(MagIMU)/tSamp;                                                    %%%%%%%% Processing input IMU values for Mag 
Gimu=sum(GyrIMU)/tSamp;
lla=eci2lla([-6.07 -1.28 0.66]*1e6,[2025 12 24 7 47 52]);                  %%%%%%%% lla caluclates the attitude, latitude and logitude for ground station in a particular time zone
[MagE, H, D ,I ,F]=wrldmagm(1000,lla(1),lla(2),decyear(2025,24,12),'2025');%%%%%%%% MagE gives magnetic vector of a earth at 1000 m altitude default
                                                                           %%%%%%%% The newton law of attraction between two bobies F=((G*m1*m2)/r^3)*r_v where gravitational constant G=6.669*10^-11 and mass of earth 5.97*10^24
AdirE=[((6.669*10^-11)*5.97*10^24*5)/(6.721*10^3)^2 0 0];                  %%%%%%%% mass of satellite is assumed to be 5kg the sateliite is assumed to be at low earth orbit ISS combined withearth radious 6.721*10^3 and others zero as sphere resulting  
t1n=(MagE/norm(MagE));
t2n=cross(t1n,AdirE)/norm(cross(t1n,AdirE));
t3n=cross(t1n,t2n);
TR=[t1n.' t2n t3n];

t1b=Mimu/norm(Mimu);                                                       %%%%%%%% Triad for iniatial 
t2b=cross(t1b,Aimu)/norm(cross(t1b,Aimu));                          
t3b=cross(t1b,t2b);
TB=[t1b t2b t3b];                                                          %%%%%%%% Triad to body frame   
CBR=TB*TR.';                                                               %%%%%%%% Direction cosine matrix 

W1=10;W2=20;                                                               %%%%%%% The weights of the wabbas function
SW=W1+W2;
B =W1*MagE*Mimu+W2*AdirE.'*Aimu;                                           %%%%%%%% Update the bias term based on the direction cosine matrix
[Lopt,S,SIG,Z]=newtonRapsion(SW,B);                                        %%%%%%%% Newton Rapshion iteration method
P=inv((Lopt+SIG)*eye(3)-S)*Z;
AQ=1/sqrt(1+P.'*P)*[P;1];                                                 %%%%%%%% Absolute quaternion from kinmatic we get angular velocity rate 
ABQ=[AQ(2) AQ(3) AQ(4)];
end