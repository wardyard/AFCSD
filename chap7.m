
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


%question 2
long_red_sp_tf4 = minreal(tf(long_red2));
q_el4 = long_red_sp_tf4(4,2);
t = 0:0.01:300;
y1 = step(q_el4, t);
figure(1)
plot(t, y1)
hold on
y2 = step(q_el, t);
plot(t, y2)
legend("Pitch rate response (4 states)", "Pitch rate response(2 states)")
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
%CAP = (g/V)*w_sp_squared*T_theta;

%For 2, Compare 4 state system with 2 state system. Answer: zoom in the first few
%seconds, the difference is negligable. Later, phugoid takes over
%long_red2_tf = tf(long_red2);
%long_red2_tf_q_over_delta_e = long_red2_tf(4,2);
%step(long_red2_tf_q_over_delta_e)
%hold on
%step(q_el)

%For 3 and 4, using gain tuning to get the required level 1 CAP, but first
%determine values that will work for wn and zeta NOTE: w_required seem way too high, impossible in fact:
w_required = 0.03*V;
T_theta_required = 1/(0.75*w_required);
zeta_required = 0.5;

%Negative as you need negative gains
p1 = -w_required*zeta_required + w_required*sqrt(zeta_required^2-1);
p2 = -w_required*zeta_required - w_required*sqrt(zeta_required^2-1);

p = [p1 p2];
K = place(A_ac_sp, B_ac_sp, p);
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
t = 0:0.001:30;
figure(2)
opt = stepDataOptions('StepAmplitude', -1);
[y3, t] = step(tf_new2, opt);
plot(t, y3)
overshoot = (max(y3)- y3(end))/(y3(end))*100
%overshoot is less than 1, thus the lower part of the "satisfactory" region
%cannot be attained and drobback is not possible.

