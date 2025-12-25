function [L0,S,SIG,Z]= newtonRapsion(s,B)
 S=B+B.';
 SIG=trace(S);
 Z=[B(2,3)-B(3,2) B(3,1)-B(1,3) B(1,2)-B(2,1)].';
 K=[S-SIG*eye(3) Z;Z.' SIG];

 f=@(s) K-s*eye(4);                                                                                                            

 L0=s;                                                                     %%%%%%% Initial guess
 tol = 1e-10;                                                              %%%%%%% Tolerance and maximum iterations
 max_iter = 100;

 
 for iter = 1:max_iter                                                     %%%%%%% Newton-Raphson iteration
 L1 = L0 - f(L0)/det(f(L0));
 if abs(L1 - L0) < tol
 fprintf('Root found: %.10f\n', L1);
 break;
 end
 L0 = L1;
 end

 if iter == max_iter
 fprintf('Maximum iterations reached without convergence.\n');
 end
end