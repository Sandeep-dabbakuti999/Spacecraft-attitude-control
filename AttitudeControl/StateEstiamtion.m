function[w_new,q_new]=StateEstiamtion(w_new,q_new,J,S_Wi,Qd,u,dt,initial_hw)
w_new=w_new+inv(J)*(-S_Wi*initial_hw+u).*dt;                               %%%%%%% Dynamic equation of a satellite
q_new=q_new+0.5*Qd*w_new.*dt;                                              %%%%%%% Linematic equation of a satellite 
end
