function gencostapprox(params,sim_)
% GENCOSTAPPROX
% Compute second-order approximation of he terminal cost g_N about x_N and
% the second-order approximation of the stage cost g_k about (x_k, u_k)
%
% Output: quadratized terminal and stage costs

    syms t real
    
    % Quadcopter system state x = [pld_rel_vel,uav_vel,pld_rel_pos,uav_pos]
    x = sym('x', [10, 1], 'real'); 
    
    % Quadcopter input u (Forces / Torques)
    u = sym('f', [3, 1], 'real');
    
    % Setup goal state
    vq_goal = params.mission.uav_vel(end,:);
    vp_goal = params.mission.pld_rel_vel(end,:);
    xq_goal = params.mission.uav_pos(end,:);
    rp_goal = params.mission.pld_rel_pos(end,:);
    x_goal = [vp_goal, vq_goal, rp_goal, xq_goal].';
    
    % Waypoints
    num_vp = size(params.mission.uav_pos,1) - 1; % all mission have same # of rows
                                                 % last row is the goal state
    
    % Cost weights
    
    % Q needs to be semi-postive definite
    Q_stage = diag(params.cost.Q_stage);
    Q_term = diag(params.cost.Q_term);
    
    % R needs to be positive definite
    R_lqr = diag(params.cost.R); % penalize control inputs
    
    % Via-point cost term 
    Q_vp = diag(params.cost.Q_viapoint);
    
    % Reference states and input the controller is supposed to track
    % Tip: keep input close to the one necessary for hovering
    f_hover = -params.derived.sys_weight; % [0;0;m*g]
    u_eq = f_hover;
    x_eq = x_goal;

    % iLQC cost functions
    
    % Continous cost terms commonly written as g_N and g_k
    T_k = sim_.Tk; % time interval at timestep k
    
    viapoint_cost = 0;
    rho = params.control.rho_ilqc; % precision
    if num_vp > 0
        for i = 1:num_vp
            t_vp = params.mission.t_interval * i;
            x_vp = [params.mission.pld_rel_vel(i,:),...
                    params.mission.uav_vel(i,:),...
                    params.mission.pld_rel_pos(i,:),...
                    params.mission.uav_pos(i,:)].';
            viapoint_cost = viapoint_cost + viapoint(x,t_vp,x_vp,Q_vp,rho);
        end
    else    
        disp('Only goal specified.');
    end
    
    cost_term = simplify((x - x_goal)' * Q_term * (x - x_goal));
    cost_stage = simplify((x - x_eq)' * Q_stage * (x - x_eq) + ...
                          (u - u_eq)' * R_lqr * (u - u_eq)) * T_k + ...
                          viapoint_cost;
    
    % Save cost functions to files
    
    des_dir = fullfile(pwd,'\generated_fcns\');
    
    % Terminal cost quadratizations
    matlabFunction(cost_term, 'vars', {x}, 'File', [des_dir,'cost_gN']);
    % d/dx
    ct_x = jacobian(cost_term, x)';
    matlabFunction(ct_x, 'vars', {x}, 'File', [des_dir,'cost_qN']);
    % dd/dxdx
    ct_xx = jacobian(ct_x, x);
    matlabFunction(ct_xx, 'vars', {x}, 'File', [des_dir,'cost_QN2']);
                        
    % Stage cost quadratization
    matlabFunction(cost_stage, 'vars', {t, x, u}, 'File', [des_dir,'cost_gk']);
    % d/dx
    cs_x = jacobian(cost_stage, x)' * T_k; % cont -> discr. time
    matlabFunction(cs_x, 'vars', {t, x, u}, 'File', [des_dir,'cost_qk']);
    % dd/dxdx
    cs_xx = jacobian(cs_x, x);
    matlabFunction(cs_xx, 'vars', {t, x, u}, 'File', [des_dir,'cost_Qk2']);
    % d/du
    cs_u = jacobian(cost_stage, u)' * T_k; % cont -> discr. time
    matlabFunction(cs_u, 'vars', {t, x, u}, 'File', [des_dir,'cost_rk']);
    % dd/dudu
    cs_uu = jacobian(cs_u, u);
    matlabFunction(cs_uu, 'vars', {t, x, u}, 'File', [des_dir,'cost_Rk2']);
    % dd/dudx
    cs_xu = jacobian(cs_x, u)';
    matlabFunction(cs_xu, 'vars', {t, x, u}, 'File', [des_dir,'cost_Pk']);

end

function viapoint_cost = viapoint(x,vp_t,vp,Qm_vp,prec)
% WAYPOINT Creates cost depending on deviation of the system state from a
% desired state vp at time t. Doesn't need to be modified. 
% For more details see:
% http://www.adrlab.org/archive/p_14_mlpc_quadrotor_cameraready.pdf
%    vp_t  :   time at which quadrotor should be at viapoint wp
%    vp    :   position where quad should be at given time instance
%    Qm_vp :   the weighting matrix for deviation from the via-point
%    prec  :   how 'punctual' does the quad have to be? The bigger the
                                % number, the harder the time constraint
syms t real
          
viapoint_cost = (x-vp)'*Qm_vp*(x-vp)*sqrt(prec/(2*pi))* ...
                exp(-0.5*prec*(t-vp_t)^2);

end