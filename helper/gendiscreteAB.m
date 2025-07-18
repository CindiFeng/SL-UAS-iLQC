function gendiscreteAB()
% Generate helper functions to discretize linearized A and B matrices

    syms dx T_k real
    
    % Linearized A and B matrices for continuous time system ddx = Adx + Bdu
    A_lin = sym('Alin', [10, 10], 'real');
    B_lin = sym('Blin', [10, 3], 'real');
    
    % Quadcopter system state x = [pld_rel_vel,uav_vel,pld_rel_pos,uav_pos]
    x = sym('x', [10, 1], 'real');
    
    % Quadcopter input u (Forces / Torques)
    u = sym('u', [3, 1], 'real');
    
    % RK4 marching scheme
    dx = A_lin*x+B_lin*u; 
    k1 = dx;
    k2 = dx + 0.5*T_k*k1;
    k3 = dx + 0.5*T_k*k2;
    k4 = dx + T_k*k3;
    x_next = simplify(x + 1/6*T_k*(k1 + 2*k2 + 2*k3 + k4));
    
    % Collect terms with x_next on LHS and discretized A and B on RHS
    out = sym(zeros(10,13));
    for i = 1:10
        [cfs, ~] = coeffs(collect(x_next(i), [x', u']), [x', u']);
        out(i,:) = cfs;
    end
    
    des_dir = fullfile(pwd,'\generated_fcns\');
    
    matlabFunction(out(:,1:10), 'var', {A_lin, T_k}, 'File', [des_dir,'discretize_A_lin']);
    matlabFunction(out(:,11:13), 'var', {B_lin, T_k}, 'File', [des_dir,'discretize_B_lin']);