%% Cleanup
close all;
clear;
clc;

% Symbolic variables
syms q1 q2 q3 real
syms m1 m2 m3 real

% Jacobian of anthropomorpic arm (rotational part)
J_r = [ 0, sin(q1), sin(q1);
        0, -cos(q1), -cos(q1);
       1, 0, 0 ];

J_r_T = J_r';

% Partial derivatives with respect to q
dJr_T_dq1 = diff(J_r_T, q1);
dJr_T_dq2 = diff(J_r_T, q2);
dJr_T_dq3 = diff(J_r_T, q3);

% Kinematic stiffness matrix
m_ext = [m1, m2, m3]';

K_kin = [dJr_T_dq1 * m_ext, dJr_T_dq2 * m_ext, dJr_T_dq3 * m_ext];

% Christoffel symbols
GammaM = [ 0, m3/2, -m2/2; ...
            -m3/2, 0, m1/2; ...
            m2/2, -m1/2, 0  ];

% Kinematic stiffness with correction term
K_kin_corr = K_kin + J_r_T * GammaM * J_r;
disp( 'K_kin_sym')
disp( K_kin_corr )

% Symmetry check
K_kin_corr_asym = 0.5 * ( K_kin_corr - K_kin_corr' );


% Function handles for substitution
J_r_func = matlabFunction(J_r, 'Vars', [q1, q2, q3]);
J_r_T_func = matlabFunction(J_r_T, 'Vars', [q1, q2, q3]);
K_kin_func = matlabFunction(K_kin, 'Vars', [q1, q2, q3, m1, m2, m3]);

% Example usage:
q_values = [pi/4, 0, 0]';   % Substitute values for q1, q2, q3
m_values = [1, 2, 3]';      % Substitute values for m1, m2, m3

J_r_numeric = J_r_func(q_values(1), q_values(2), q_values(3));
J_r_T_numeric = J_r_T_func(q_values(1), q_values(2), q_values(3));
K_kin_numeric = K_kin_func(q_values(1), q_values(2), q_values(3), m_values(1), m_values(2), m_values(3));
GammaM_numeric = [  0,   m_values(3)/2, -m_values(2)/2;
                    -m_values(3)/2,  0,   m_values(1)/2;
                    m_values(2)/2, -m_values(1)/2,  0 ];
               
K_kin_corr_numeric = K_kin_numeric + J_r_T_numeric * GammaM_numeric * J_r_numeric;
disp( 'K_kin')
disp( K_kin_corr_numeric )

K_kin_corr_numeric_asym = 0.5 * ( K_kin_corr_numeric - transpose(K_kin_corr_numeric) );
disp( 'K_kin_asym')
disp(K_kin_corr_numeric_asym)







