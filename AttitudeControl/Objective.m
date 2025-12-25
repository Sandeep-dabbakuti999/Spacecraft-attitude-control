function [obj]=Objective(w_new,q_new,quatDA,Wd)
 q_0=sqrt(1-q_new(1)^2-q_new(2)^2-q_new(3)^2);
 q_err=(quatmultiply([q_0;q_new].',quatinv(quatDA)));
 obj=(w_new(1)-Wd(1))^2+(w_new(2)-Wd(2))^2+(w_new(3)-Wd(3))^2+q_err(2)^2+q_err(3)^2+q_err(4)^2;
end
