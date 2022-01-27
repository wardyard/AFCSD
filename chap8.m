clear all; 
close all; 

load('chapter8.mat')

% 1: construct reduced model with states: h, V, alpha, theta, q

% create 7 x 7 state space with actuator states included 
states_lon = [1 3 4 2 5 6 7];   % h, V, alpha, theta, q + actuator states
inputs_lon = [1 2];             % thrust, elevtor deflection
outputs_lon = [1 3 4 2 5];      % h, V, alpha, theta, q

% determine state space representation 
A_ol = SS_long_lo.A(states_lon, states_lon);
B_ol = SS_long_lo.B(states_lon, inputs_lon);
C_ol = SS_long_lo.C(outputs_lon, states_lon);
D_ol = SS_long_lo.D(outputs_lon, inputs_lon);

% creats state space representation
long_red1 = ss(A_ol, B_ol, C_ol, D_ol, 'StateName', SS_long_lo.StateName(states_lon), ...
    'InputName', SS_long_lo.InputName(inputs_lon), 'OutputName', SS_long_lo.OutputName(outputs_lon))

% 2) remove actuator dynamics from 6X6 SS
states_lon2 = [1 3 4 2 5];      % h, V, alpha, theta, q
inputs_lon2 = inputs_lon;
outputs_lon2 = outputs_lon; 
A_ac1 = long_red1.A([1 2 3 4 5], [1 2 3 4 5]);
B_ac1 = long_red1.A([1 2 3 4 5], [7 6]);
C_ac1 = long_red1.C([1 2 3 4 5], [1 2 3 4 5]);
D_ac1 = long_red1.D;

% reduced state space model: 
ss_red = ss(A_ac1, B_ac1, C_ac1, D_ac1, 'StateName', SS_long_lo.StateName(states_lon2), ...
    'InputName', SS_long_lo.InputName(inputs_lon2), 'OutputName', SS_long_lo.OutputName(outputs_lon2))
%{
% construct pitch attitude hold mode 
% inner loop: pitch damper -> determine K_q via RL
tfs = minreal(tf(ss_red));
H_q_el = tfs(5,2);
pole(H_q_el);
servo = tf([20.2], [1 20.2]);
%sisotool(minreal(-H_q_el*servo))
K_q = -0.16635;

A_new = A_ac1 - B_ac1(:,2)*C_ac1(5,:)*K_q;

ss_red2 = ss(A_new, B_ac1, C_ac1, D_ac1, 'StateName', SS_long_lo.StateName(states_lon2), ...
    'InputName', SS_long_lo.InputName(inputs_lon2), 'OutputName', SS_long_lo.OutputName(outputs_lon2));

% construct pitch attitude hold mode 
tfs2 = minreal(tf(ss_red2)); 
H_theta_el = tfs2(4,2)
%sisotool(-H_theta_el)

% we need PID controller 
K_theta = 1.5104;

% construct airspeed hold mode 
H_v_thr = tfs(2,1);
engine = tf([1],[1 1]);
%rltool(engine*H_v_thr)
% tune PID via Ziegler Nichols 
K_crit = 0.0091311;
T_crit = 175;
K_p_v = 0.6*K_crit;
K_i_V = K_p_v/(0.5*T_crit);
K_d_V = 0.125*T_crit*K_p_v;
%}
%transfer function GS_angle/theta 
tfs = minreal(tf(ss_red))
H_alpha_el = tfs(3,2)
H_theta_el = tfs(4,2)
H_gs_theta = (1 - (H_alpha_el/H_theta_el))
[num_gs_theta, denum_gs_theta] = tfdata(H_gs_theta, 'v')



