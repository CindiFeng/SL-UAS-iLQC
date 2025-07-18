function cost_approx = stage_cost(t_sim, x_sim, u_sim)
%STAGE_COST_QUAD Summary of this function goes here
%   Detailed explanation goes here
    cost_approx.gk = cost_gk(t_sim,x_sim,u_sim);
    cost_approx.qk = cost_qk(t_sim,x_sim,u_sim);
    cost_approx.Qk = cost_Qk2(t_sim,x_sim,u_sim);
    cost_approx.rk = cost_rk(t_sim,x_sim,u_sim);
    cost_approx.Rk = cost_Rk2(t_sim,x_sim,u_sim);
    cost_approx.Pk = cost_Pk(t_sim,x_sim,u_sim);

end

