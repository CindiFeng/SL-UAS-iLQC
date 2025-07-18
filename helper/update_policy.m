function [duff, K, s_k, S_k] = update_policy(A, B, cost, s_k1, S_k1)
% Approximate cost-to-go with a second-order Talor expansion to construct
% l_k, G_k, and H_k; solve the Blemman Equation (DPA) and update policy
% 
% Input:           A,B   discretized A and B matrices of linearized system
%                 cost   approximated stage costs 
%           s_k1, S_k1   previous timestep cost-to-go variables 
% 
% Output:      duff, K   variables needed for policy update 
%              s_k, S_k  new cost-to-go variables

    l_k = cost.rk + B'*s_k1;
    G_k = cost.Pk + B'*S_k1*A;
    H_k = cost.Rk + B'*S_k1*B;
    
    H_k = (H_k+H_k')/2; 

    duff = -H_k\l_k;
    K = -H_k\G_k;

    S_k = cost.Qk + A'*S_k1*A + K'*H_k*K + K'*G_k + G_k'*K;
    s_k = cost.qk + A'*s_k1 + K'*H_k*duff + K'*l_k + G_k'*duff;
end

