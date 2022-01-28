
clear all; 
close all; 
% ---- 7.3 CAP criterion
load('long_red2.mat');
syms t;

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


%question 2
long_red_sp_tf4 = minreal(tf(long_red2));
q_el4 = long_red_sp_tf4(4,2);
t1 = 0:0.01:20;
%opt = stepDataOptions('StepAmplitude', -1)
y1 = step(-q_el4, t1);
figure(1)
plot(t1, y1)
hold on
%opt = stepDataOptions('StepAmplitude', -1)
y2 = step(-q_el, t1);
plot(t1, y2)
legend("Pitch rate response (4 states)", "Pitch rate response (2 states)")
xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold')
ylabel('pitch rate [deg/s]', 'FontSize', 12, 'FontWeight', 'bold')
grid on
title('Pitch rate response on a step input', 'FontSize', 13)
hold off

t2 = 0:0.01:200;
%opt = stepDataOptions('StepAmplitude', -1)
y3 = step(-q_el4, t2);
figure(2)
plot(t2, y3)
hold on
%opt = stepDataOptions('StepAmplitude', -1)
y4 = step(-q_el, t2);
plot(t2, y4)
legend("Pitch rate response (4 states)", "Pitch rate response (2 states)")
xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold')
ylabel('pitch rate [deg/s]', 'FontSize', 12, 'FontWeight', 'bold')
grid on
title('Pitch rate response on a step input', 'FontSize', 13)
hold off



%Get value of Kq and therefore T_theta
[num, denom] = tfdata(q_el);
kq = num{1}(2);
kq_removed = num{1}/kq;
T_theta = kq_removed(3);
w_sp_squared = denom{1}(3);
w_sp = sqrt(w_sp_squared);
zeta = (denom{1}(2))/(2*sqrt(w_sp_squared));

%For 1. CAP
g = 9.81;
V = 274.32;


%For 3 and 4, using gain tuning to get the required level 1 CAP, but first
%determine values that will work for wn and zeta NOTE: w_required seem way too high, impossible in fact:
w_required = 0.03*V
T_theta_required = 1/(0.75*w_required)
zeta_required = 0.5

%Negative as you need negative gains
p1 = -w_required*zeta_required + w_required*sqrt(zeta_required^2-1);
p2 = -w_required*zeta_required - w_required*sqrt(zeta_required^2-1);

p = [p1 p2];
K = place(A_ac_sp, B_ac_sp, p)
K_alpha = K(1);
K_q = K(2);

V_gust = 4.572; %m/s
alpha_induced = atan(V_gust/V);
deltac = K_alpha*alpha_induced %degrees, less than 25 degrees, thus it is acceptable

%question 5
%Pre-filter compensation modifies the closed loop transfer function
%directly, while forward path and feedback compensation modify the
%forward path and feedback path transfer functions, which are inputs
%for the root locus.
%zeros in the loop will create unwanted poles in the closed loop transfer
%location and the design will be disorted. Thus, if we want to cancel a
%zero, a pre-filter must be outside the loop.


%question 6
CAP = (w_required)^2/(V/(g*T_theta_required))
DB_over_q = T_theta_required - 2*(zeta_required/w_required);
s = tf('s');
tf_new1 = minreal((K_q*(1+T_theta*s))/(s^2+2*zeta_required*w_required*s+w_required^2));
filter = (1+T_theta_required*s)/(1+T_theta*s)
tf_new2 = minreal(tf_new1*filter)
t = 0:0.01:20;
figure(3)
%opt = stepDataOptions('StepAmplitude', -1);
%[y3, t] = step(tf_new2, opt);
u = -1+1*heaviside(t-10);
y3 = lsim(tf_new2,u,t);
plot(t, y3)
xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold')
ylabel('Pitch rate q(t)', 'FontSize', 12, 'FontWeight', 'bold')
title('Pitch rate tracking response', 'FontSize', 13)
grid on
overshoot = (max(y3)- y3(end))/(y3(end))*100;

figure(4)
tf_pitchangle = tf_new2*(1/s);
y4 = lsim(tf_pitchangle,u,t);
plot(t, y4)
xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold')
ylabel('Pitch angle \Theta (t)', 'FontSize', 12, 'FontWeight', 'bold')
title('Pitch angle tracking response', 'FontSize', 13)
grid on

qmoverqs = max(y3)/y3(t==4);
figure(5)
ptch = patch([0 0.3 0.06 0],[1 1 3 3], 'green', 'FaceAlpha', 0.5)
hold on
scatter(DB_over_q, qmoverqs,30, 'MarkerEdgeColor', [1 0 1],...
    'MarkerFaceColor', [1 0 0], 'LineWidth', 1.5)
xlim([-0.4,0.6])
ylim([1,4])
xlabel('OS/q_s             DB/q_s  [s]', 'FontSize', 10, 'FontWeight', 'bold')
ylabel('q_m/q_s  [-]', 'FontSize', 10, 'FontWeight', 'bold')
grid on
title('Criterion for the tracking task', 'FontSize', 13)



%%
load('20000_600_lin.mat')

% 1) reduce SS_long_lo to a 6X6 state space system 
states_lon_extra = [2 3 4 5 6 7];  % all states except h 
inputs_lon_extra = [1 2];          % all inputs 
outputs_lon_extra = [2 3 4 5];     % theta, Vt, alpha, q

% determine state space representation 
A_ol_extra = SS_long_lo.A(states_lon_extra, states_lon_extra);
B_ol_extra = SS_long_lo.B(states_lon_extra, inputs_lon_extra);
C_ol_extra = SS_long_lo.C(outputs_lon_extra, states_lon_extra);
D_ol_extra = SS_long_lo.D(outputs_lon_extra, inputs_lon_extra);

% creats state space representation
long_red1_extra = ss(A_ol_extra, B_ol_extra, C_ol_extra, D_ol_extra, 'StateName', SS_long_lo.StateName(states_lon_extra), ...
    'InputName', SS_long_lo.InputName(inputs_lon_extra), 'OutputName', SS_long_lo.OutputName(outputs_lon_extra))

% 2) remove actuator dynamics from 6X6 SS
states_lon1_extra = [2 3 4 5];
inputs_lon1_extra = inputs_lon_extra;
outputs_lon1_extra = outputs_lon_extra; 
A_ac1_extra = long_red1_extra.A([1 2 3 4], [1 2 3 4]);
B_ac1_extra = long_red1_extra.A([1 2 3 4], [5 6]);
C_ac1_extra = long_red1_extra.C([1 2 3 4], [1 2 3 4]);
D_ac1_extra = long_red1_extra.D;

long_red2_extra = ss(A_ac1_extra, B_ac1_extra, C_ac1_extra, D_ac1_extra, 'StateName', SS_long_lo.StateName(states_lon1_extra), ...
    'InputName', SS_long_lo.InputName(inputs_lon1_extra), 'OutputName', SS_long_lo.OutputName(outputs_lon1_extra))

%Reduce SS to short period motion simplification with AOA and q
states_lon_sp_extra = [3 4];
inputs_lon_sp_extra = [2];


% Select AOA and q
%long_red2.A
A_ac_sp_extra = long_red2_extra.A([3 4], [3 4]);
%long_red2.B
B_ac_sp_extra = long_red2_extra.B([3 4], [2]);
%long_red2.C
C_ac_sp_extra = long_red2_extra.C([3 4], [3 4]);
%long_red2.D
D_ac_sp_extra = long_red2_extra.D([3 4], [2]);

%Short period motion state space system
long_red_sp_extra = ss(A_ac_sp_extra, B_ac_sp_extra, C_ac_sp_extra, D_ac_sp_extra, 'StateName', long_red2_extra.StateName(states_lon_sp_extra), ...
    'InputName', long_red2_extra.InputName(inputs_lon_sp_extra));

%Select q/delta_e transfer function
long_red_sp_tf_extra = minreal(tf(long_red_sp_extra));
q_el_extra = long_red_sp_tf_extra(2,1);

%question 2
long_red_sp_tf4_extra = minreal(tf(long_red2_extra));
q_el4_extra = long_red_sp_tf4_extra(4,2);
t = 0:0.01:300;
y1_extra = step(q_el4_extra, t);
figure(6)
plot(t, y1_extra)
hold on
y2_extra = step(q_el_extra, t);
plot(t, y2_extra)
legend("Pitch rate response (4 states)", "Pitch rate response(2 states)")
hold off


%Get value of Kq and therefore T_theta
[num, denom] = tfdata(q_el_extra);
kq_extra = num{1}(2);
kq_removed_extra = num{1}/kq_extra;
T_theta_extra = kq_removed_extra(3);
w_sp_squared_extra = denom{1}(3);
w_sp_extra = sqrt(w_sp_squared_extra);
zeta_extra = (denom{1}(2))/(2*sqrt(w_sp_squared_extra));

%For 1. CAP
g_extra = 9.81;
V_extra = 182.88;
%CAP = (g/V)*w_sp_squared*T_theta;

w_required_extra = 0.03*V_extra
T_theta_required_extra = 1/(0.75*w_required_extra)
zeta_required_extra = 0.5

%Negative as you need negative gains
p1_extra = -w_required_extra*zeta_required_extra + w_required_extra*sqrt(zeta_required_extra^2-1);
p2_extra = -w_required_extra*zeta_required_extra - w_required_extra*sqrt(zeta_required_extra^2-1);

p_extra = [p1_extra p2_extra];
K_extra = place(A_ac_sp_extra, B_ac_sp_extra, p_extra)
K_alpha_extra = K_extra(1);
K_q_extra = K_extra(2);

V_gust_extra = 4.572; %m/s
alpha_induced_extra = atan(V_gust_extra/V_extra);
deltac_extra = K_alpha_extra*alpha_induced_extra %degrees, less than 25 degrees, thus it is acceptable


%question 6
CAP_extra = (w_required_extra)^2/(V_extra/(g_extra*T_theta_required_extra))
DB_over_q_extra = T_theta_required_extra - 2*(zeta_required_extra/w_required_extra);
s = tf('s');
tf_new1_extra = minreal((K_q_extra*(1+T_theta_extra*s))/(s^2+2*zeta_required_extra*w_required_extra*s+w_required_extra^2));
filter_extra = (1+T_theta_required_extra*s)/(1+T_theta_extra*s)
tf_new2_extra = minreal(tf_new1_extra*filter_extra)
t = 0:0.001:30;
figure(7)
opt_extra = stepDataOptions('StepAmplitude', -1);
[y3_extra, t] = step(tf_new2_extra, opt_extra);
plot(t, y3_extra)
overshoot_extra = (max(y3_extra)- y3_extra(end))/(y3_extra(end))*100


qmoverqs_extra = max(y3_extra)/y3_extra(end);
figure(8)
ptch_extra = patch([0 0.3 0.06 0],[1 1 3 3], 'green', 'FaceAlpha', 0.5)
hold on
scatter(DB_over_q_extra, qmoverqs_extra,30, 'MarkerEdgeColor', [1 0 1],...
    'MarkerFaceColor', [1 0 0], 'LineWidth', 1.5)
xlim([-0.4,0.6])
ylim([1,4])
xlabel('OS/q_s             DB/q_s  [s]', 'FontSize', 15, 'FontWeight', 'bold')
ylabel('q_m/q_s  [-]', 'FontSize', 15, 'FontWeight', 'bold')
grid on
title('Criterion for the tracking task', 'FontSize', 15)

%%
V_interpol = 213.64;
w_required_interpol = 0.03*V_interpol;
T_theta_required_interpol = 1/(0.75*w_required);
zeta_required_interpol = 0.5;
K_q_interpol = K_q + (V_interpol - V)*((K_q_extra - K_q)/(V_extra - V))

CAP_interpol = (w_required_interpol)^2/(V_interpol/(g*T_theta_required_interpol))
DB_over_q_interpol = T_theta_required_interpol - 2*(zeta_required_interpol/w_required_interpol);
s = tf('s');
tf_new1_interpol = minreal((K_q_extra*(1+T_theta*s))/(s^2+2*zeta_required_interpol*w_required_interpol*s+w_required_interpol^2));
filter_interpol = (1+T_theta_required_interpol*s)/(1+T_theta*s)
tf_new2_interpol = minreal(tf_new1_interpol*filter_interpol)
t = 0:0.001:30;
figure(9)
opt_interpol = stepDataOptions('StepAmplitude', -1);
[y3_interpol, t] = step(tf_new2_interpol, opt_interpol);
plot(t, y3_interpol)
overshoot_interpol = (max(y3_interpol)- y3_interpol(end))/(y3_interpol(end))*100

qmoverqs_interpol = max(y3_interpol)/y3_interpol(end);
figure(10)
ptch_interpol = patch([0 0.3 0.06 0],[1 1 3 3], 'green', 'FaceAlpha', 0.5)
hold on
scatter(DB_over_q_interpol, qmoverqs_interpol,30, 'MarkerEdgeColor', [1 0 1],...
    'MarkerFaceColor', [1 0 0], 'LineWidth', 1.5)
xlim([-0.4,0.6])
ylim([1,4])
xlabel('OS/q_s                      DB/q_s  [s]', 'FontSize', 12, 'FontWeight', 'bold')
ylabel('q_m/q_s  [-]', 'FontSize', 12, 'FontWeight', 'bold')
grid on
title('Criterion for the tracking task', 'FontSize', 13)
