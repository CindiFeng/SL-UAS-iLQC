function genlinmdl(params,sim_)
    m_p = params.pld_mass;
    m_q = params.uav_mass;
    g = sim_.g;
    L = params.cable_len;
    
    % translation states 
    r_L = sym('r_L',[2 1],'real');
    v_L = sym('v_L',[2 1],'real');
    v_q = sym('v_q',[3 1],'real');
    x_q = sym('x_q', [3 1],'real');
    v = [v_L; v_q]; 
    X_trans = [v; r_L; x_q;]; % translational subsystem 
    
    % Forces and input vector
    F_I = sym('f_',[3 1],'real'); 
    F = [zeros(2,1); F_I];
    g_I=[0; 0; g];
    U_trans = F_I; % input vector
    
    % Geometric Relations
    B_hat = [eye(2,2);
             -r_L'/sqrt(L^2-r_L'*r_L)];
    B_hat_dot = [zeros(2,2);
                 ((v_L*(r_L'*r_L-L^2)-r_L*r_L'*v_L)/(L^2-r_L'*r_L)^(3/2))'];
    
    % System Matrices
    G = [m_p*B_hat'*g_I; (m_p+m_q)*g_I];
    C = [m_p*B_hat'*B_hat_dot zeros(2,3);
         m_p*B_hat_dot zeros(3,3)];
    M = [m_p*(B_hat'*B_hat) m_p*B_hat';
         m_p*B_hat (m_q+m_p)*eye(3,3)];
    
    % Nonlinear EOM 
    v_dot = M \ (F + G - C*v);
    X_trans_dot = [v_dot;v];
    
    % Linearize state and input matrices
    A_lin = jacobian(X_trans_dot, X_trans);
    B_lin = jacobian(X_trans_dot, U_trans);
    
    % % Equilibrium points
    % X_e = zeros(10,1);
    % U_e = F_I - ([zeros(3,2),eye(3)]*subs((F + G - C*v),X_trans,X_e));
    % 
    % % Linearize state and input matrices
    % J_X = subs(jacobian((F + G - C*v), X_trans),[X_trans; U_trans],[X_e;U_e]);
    % J_U = subs(jacobian((F + G - C*v), U_trans),[X_trans; U_trans],[X_e;U_e]);
    % 
    % A_lin = [simplify(M \ J_X); [eye(5),zeros(5)]];
    % B_lin = [simplify(M \ J_U); zeros(5,3)];
    
    % Save linearized A and B matrix to path
    
    des_dir = fullfile(pwd,'\generated_fcns\');
    
    matlabFunction(A_lin, 'var', {X_trans, U_trans}, 'File', [des_dir,'Mdl_Alin']);
    matlabFunction(B_lin, 'var', {X_trans, U_trans}, 'File', [des_dir,'Mdl_Blin']);