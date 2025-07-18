%% Run main iLQC simulation
% 2022-08-05 
% Runs the simulation model iLQC_controller using the optimized input from
% iLQC algorithm written in this file. 
% 
% Run genX.m files if parameters/mission change.
% 
% run_init.m called by simulation model upon starting. 

clear; clc;
addpath('generated_fcns','helper','utils','init_params');
run_init; 
genlinmdl(params,sim_);
gendiscreteAB();
gencostapprox(params,sim_);

%% iLQC simulation
max_iter = 50;
eps = 1e-8; % tolerance
cmd = zeros(3, 11, 2001);
% cost_hist = zeros(max_iter,1);

% Outer loop to iterate the simulation to find sol'n
for it = 1:max_iter
    fprintf('Running iteration-%.f ...', it);
    
    % Forward Pass
    sim('iLQCsimulation');
    sim_t = tout;
    sim_x = squeeze(yout.get('all_states').Values.Data);
    sim_u = squeeze(yout.get('F_lift').Values.Data);
    Nt = length(sim_t);
    Nx = size(sim_x,1);
    Nu = size(sim_u,1);
    
    % time-varying variables for Bellman equation approximation
    s_k = zeros(Nx, Nt + 1);
    S_k = zeros(Nx, Nx, Nt + 1);
    
    % variables for finding optimal input 
    du_ff = zeros(Nu, Nt);
    K = zeros(Nu, Nx, Nt);
    theta_ff = zeros(Nu, Nt);
    theta_fb = zeros(Nu, Nx, Nt);
    
    % init backward pass
    cost_approx = terminal_cost(sim_x(:,Nt));
    s_k(:,Nt) = cost_approx.qN; 
    S_k(:,:,Nt) = cost_approx.QN;
    
    % Record cost
    cost_hist(it,1) = cost_approx.gN;
    
    % Backwards Pass
    disp('Starting backward pass loop')
    for k = Nt-1:-1:1
        
        % compute stage cost
        cost_approx = stage_cost(sim_t(k),sim_x(:,k),sim_u(:,k));
        
        % record cost
        cost_hist(it,Nt+1-k) = cost_approx.gk;
        
        % compute continuous linear state and input matrices
        Alin = Mdl_Alin(sim_x(:,k), sim_u(:,k));
        Blin = Mdl_Blin(sim_x(:,k), sim_u(:,k));

        % approximate discrete linear state and input matrices
        A = discretize_A_lin(Alin, sim_.Tk);
        B = discretize_B_lin(Blin, sim_.Tk);

        % compute policy update
        [du_ff(:, k), K(:, :, k), s_k(:, k), S_k(:, :, k)] = ...
        update_policy(A, B, cost_approx, s_k(:, k + 1), S_k(:, :, k + 1));

        % compute feedforward gain of control input
        theta_ff(:, k) = sim_u(:,k) + du_ff(:, k) - K(:, :, k) * sim_x(:,k);

        % compute feedback gain of control input (size: n * m * N-1)
        theta_fb(:, :, k) = K(:, :, k);
        
    end
    
    cmd_prev = cmd;
    cmd = permute(cat(3, theta_ff', permute(theta_fb, [3, 1, 2])), [2, 3, 1]);
    
    % iteration termination conditions
    if any(isnan(sim_u), 'all')
        fprintf('NANs in policy on iteration-%.f\n', it);
        break;
    end

    if it > 1
        nrm_cmd_diff = norm(sum(cmd - cmd_prev, 3), 'fro');

        if nrm_cmd_diff < 1e-4
            fprintf('Converged on iteration-%.f\n', it);
            break;
        end

    end

    if it >= max_iter
        fprintf('Iteration limit reached using convergence condition-%f\n', eps);
    end

end

cost_tot = sum(cost_hist,2);

%% Plot 3D graph
pld_rel_pos = squeeze(yout.get('pld_rel_pos').Values.Data);
uav_pos = squeeze(yout.get('uav_pos').Values.Data);
L_vec = [pld_rel_pos;sqrt(params.cable_len^2-diag(pld_rel_pos.'*pld_rel_pos).')];
pld_abs_pos = uav_pos + L_vec;

figure('Name','Slung-load UAV Trajectory')
plot3(params.mission.uav_pos(:,1),params.mission.uav_pos(:,2),...
      -params.mission.uav_pos(:,3),'dr');
hold on;

pld_rel_goal = [params.mission.pld_rel_pos.';
                sqrt(params.cable_len^2-...
                diag(params.mission.pld_rel_pos*params.mission.pld_rel_pos.')).'];
pld_abs_goal = params.mission.uav_pos.' + pld_rel_goal;
plot3(pld_abs_goal(1,:),pld_abs_goal(2,:),pld_abs_goal(3,:),'or');
grid on;
title('Slung-load UAV Trajectory');
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
plot3(uav_pos(1,:),uav_pos(2,:),-uav_pos(3,:),'b-','linewidth',2)
plot3(pld_abs_pos(1,:),pld_abs_pos(2,:),-pld_abs_pos(3,:),'k--','linewidth',1)

%% Plot payload
figure('Name','Pld Motion'); % plot payload motion
subplot(2,2,1);
Plot2d(params.mission.pld_rel_pos(:,1),params.mission.t_interval,...
            sim_t,sim_x(6,:));
xlabel('t (s)');
ylabel('x (m)','Interpreter','tex');
title('Pld Relative x Pos (r_{x})');

subplot(2,2,2);
Plot2d(params.mission.pld_rel_pos(:,2),params.mission.t_interval,...
            sim_t,sim_x(7,:));
xlabel('t (s)');
ylabel('y (m)','Interpreter','tex');
title('Pld Relative y Pos (r_{y})');

subplot(2,2,3);
plot(sim_x(6,:),sim_x(7,:),'b-','LineWidth',1);
xlabel('x (m)');
ylabel('y (m)');
title('Pld Relative Pos');

subplot(2,2,4);
pld_planar_z = sqrt(sim_x(6,:).^2 + sim_x(7,:).^2);
plot(sim_t,pld_planar_z,'b-','LineWidth',1);
xlabel('t (s)');
ylabel('r_{p} (m)','Interpreter','tex');
title('Planar Pld Relative Dist (radius)');

%% Plot drone
figure('Name','Drone Motion'); % plot quad motion
subplot(2,2,1);
Plot2d(params.mission.uav_pos(:,1),params.mission.t_interval,...
            sim_t,sim_x(8,:));
xlabel('t (s)');
ylabel('x (m)');
title('x Pos');

subplot(2,2,2);
Plot2d(params.mission.uav_pos(:,2),params.mission.t_interval,...
            sim_t,sim_x(9,:));
xlabel('t (s)');
ylabel('y (m)');
title('y Pos')

subplot(2,2,3);
Plot2d(params.mission.uav_pos(:,3),params.mission.t_interval,...
            sim_t,sim_x(10,:));
xlabel('t (s)');
ylabel('z (m)');
title('z Pos');

subplot(2,2,4);
plot(sim_t,sim_u.*[1; 1; -1],'LineWidth',1);
legend('f_{x}','f_{y}','f_{z}');
xlabel('t (s)');
ylabel('Force (N)');
title('Input');

%% Clean up
rmpath('generated_fcns','helper','utils','init_params');