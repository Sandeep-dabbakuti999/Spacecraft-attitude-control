function [u_optimal_vec]=optimize(obj,w_new,q_new,Wl,QL,Wu,QU)
    options = optimoptions(@fmincon,'Algorithm','sqp');
    u_optimal_vec = fmincon(@(W,Q) obj,[w_new;q_new], [], [], [], [],[Wl;QL], [Wu;QU], [], options);
end