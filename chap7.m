
% ---- 7.3 CAP criterion
load('long_red2.mat');

%Reduce SS to short period motion simplification with AOA and q
states_lon_sp = [3 4];
inputs_lon_sp = [2];
%outputs_lon_sp = [1 2]

% Select AOA and q
%long_red2.A
A_ac_sp = long_red2.A([3 4], [3 4]);
%long_red2.B
B_ac_sp = long_red2.B([3 4], [2]);
%long_red2.C
C_ac_sp = long_red2.C([3 4], [3 4]);
%long_red2.D
D_ac_sp = long_red2.D([3 4], [2]);

%Short period motion state space system
long_red_sp = ss(A_ac_sp, B_ac_sp, C_ac_sp, D_ac_sp, 'StateName', long_red2.StateName(states_lon_sp), ...
    'InputName', long_red2.InputName(inputs_lon_sp));

%Select q/delta_e transfer function
long_red_sp_tf = minreal(tf(long_red_sp));
q_el = long_red_sp_tf(2,1);

%Get value of Kq and therefore T_theta
[num, denom] = tfdata(q_el);
kq = num{1}(2);
kq_removed = num{1}/kq;
T_theta = kq_removed(3);
w_sp_squared = denom{1}(3);
w_sp = sqrt(w_sp_squared)
zeta = (denom{1}(2))/(2*sqrt(w_sp_squared))

%For 1. CAP
g = 9.81;
V = 274.32;
CAP = (g/V)*w_sp_squared*T_theta;

%For 2, Compare 4 state system with 2 state system. Answer: zoom in the first few
%seconds, the difference is negligable. Later, phugoid takes over
long_red2_tf = tf(long_red2);
long_red2_tf_q_over_delta_e = long_red2_tf(4,2);
%step(long_red2_tf_q_over_delta_e)
%hold on
%step(q_el)

%For 3 and 4, using gain tuning to get the required level 1 CAP, but first
%determine values that will work for wn and zeta NOTE: w_required seem way too high, impossible in fact:
w_required = 0.03*V
T_theta = 1/(0.75*w_required);
zeta_required = 0.5

%Negative as you need negative gains
rlocus(-q_el)
sgrid


