function cost_approx = terminal_cost(x)
%TERMINAL_COST_QUAD Summary of this function goes here
%   Detailed explanation goes here
    cost_approx.gN = cost_gN(x);
    cost_approx.qN = cost_qN(x);
    cost_approx.QN = cost_QN2(x);
end

