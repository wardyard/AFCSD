%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% script for chapter 6: open loop analysis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load('20000_900_lin.mat')
s = tf('s');

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% reducing state space matrices 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%----- Longitudinal model ---------
% 1) reduce SS_long_lo to a 6X6 state space system 
states_lon = [2 3 4 5 6 7];  % all states except h 
inputs_lon = [1 2];          % all inputs 
outputs_lon = [2 3 4 5];     % theta, Vt, alpha, q

% determine state space representation 
A_ol = SS_long_lo.A(states_lon, states_lon);
B_ol = SS_long_lo.B(states_lon, inputs_lon);
C_ol = SS_long_lo.C(outputs_lon, states_lon);
D_ol = SS_long_lo.D(outputs_lon, inputs_lon);

% creats state space representation
long_red1 = ss(A_ol, B_ol, C_ol, D_ol, 'StateName', SS_long_lo.StateName(states_lon), ...
    'InputName', SS_long_lo.InputName(inputs_lon), 'OutputName', SS_long_lo.OutputName(outputs_lon))

% 2) remove actuator dynamics from 6X6 SS
states_lon2 = [2 3 4 5];
inputs_lon2 = inputs_lon;
outputs_lon2 = outputs_lon; 
A_ac1 = long_red1.A([1 2 3 4], [1 2 3 4]);
B_ac1 = long_red1.A([1 2 3 4], [5 6]);
C_ac1 = long_red1.C([1 2 3 4], [1 2 3 4]);
D_ac1 = long_red1.D;

long_red2 = ss(A_ac1, B_ac1, C_ac1, D_ac1, 'StateName', SS_long_lo.StateName(states_lon2), ...
    'InputName', SS_long_lo.InputName(inputs_lon2), 'OutputName', SS_long_lo.OutputName(outputs_lon2))


%----- Lateral model ---------
% 1) reduce SS_lat_lo to a 6X6 state space system 
states_lat = [4 1 5 6 8 9];  % all states except psi, Vt and thrust
inputs_lat = [2 3];          % rudder and aileron
outputs_lat = [4 1 5 6];     % beta, phi, p, r

% determine state space representation 
A_ol2 = SS_lat_lo.A(states_lat, states_lat);
B_ol2 = SS_lat_lo.B(states_lat, inputs_lat);
C_ol2 = SS_lat_lo.C(outputs_lat, states_lat);
D_ol2 = SS_lat_lo.D(outputs_lat, inputs_lat);

% creats state space representation
lat_red1 = ss(A_ol2, B_ol2, C_ol2, D_ol2, 'StateName', SS_lat_lo.StateName(states_lat), ...
    'InputName', SS_lat_lo.InputName(inputs_lat), 'OutputName', SS_lat_lo.OutputName(outputs_lat))

% 2) remove actuator dynamics from 6X6 SS
states_lat2 = [4 1 5 6];    
inputs_lat2 = inputs_lat;
outputs_lat2 = outputs_lat;
A_ac2 = lat_red1.A([1 2 3 4], [1 2 3 4]);
B_ac2 = lat_red1.A([1 2 3 4], [5 6]);
C_ac2 = lat_red1.C([1 2 3 4], [ 1 2 3 4]);
D_ac2 = lat_red1.D;

lat_red2 = ss(A_ac2, B_ac2, C_ac2, D_ac2, 'StateName', SS_lat_lo.StateName(states_lat2), ...
    'InputName', SS_lat_lo.InputName(inputs_lat2), 'OutputName', SS_lat_lo.OutputName(outputs_lat2))

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculation of the inherent motion characteristics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% damping and natural frequency of poles for longitudinal reduced model 
damp(long_red2)
% damping and natural frequency of poles for lateral reduced model 
damp(lat_red2)

% plotting eigenmotion responses 
% first, determine transfer functions 
tfs_lon = tf(long_red2); 
tfs_lat = tf(lat_red2)

% elevator to pitch rate TF (for short period motion)
H_q_el = tfs_lon(4,2)

% elevator to pitch attitude (for phugoid motion)
H_theta_el = tfs_lon(3,2)

% rudder to roll attitude TF (for Dutch roll) 
H_phi_rud = tfs_lat(2,2)

% beta to rudder TF (for Dutch roll) 
H_beta_rud = tfs_lat(1,2)

% roll rate to rudder TF (for Dutch roll)
H_p_rud = tfs_lat(3,2)

% yaw rate to rudder TF (for Dutch roll) 
H_r_rud = tfs_lat(4,2)

% roll rate to aileron TF (for aperiodic roll)
H_p_ail = tfs_lat(3,1)

% roll angle to aileron TF (for spiral) 
H_phi_ail = tfs_lat(2,1)
% input signal for spiral motion: step of 3 sec 

% plot short period 
fig1 = figure(1);
[y , time] = step(H_q_el,15); 
plot(time, y)
xlabel('time [s]'); 
ylabel('q [rad/s]'); 
title('pitch rate step response to elevator input (Short period)'); 

% plot phugoid 
fig2 = figure(2);
[y, time] = step(H_theta_el, 500);
plot(time, y)
xlabel('time [s]'); 
ylabel('\theta [°]'); 
title('pitch attitdue response to elevator step input (Phugoid)');

% plot dutch roll 
fig3 = figure(3);
t = tiledlayout(4,1);
title(t,'Rudder impulse response for \beta, \phi, p and r (Dutch roll)')
xlabel(t,'time [s]')
nexttile;
[y, time] = impulse(H_beta_rud, 10); 
plot(time, y)
ylabel('\beta [°]')
nexttile;
[y, time] = impulse(H_phi_rud, 10); 
plot(time, y)
ylabel('\phi [°]')
nexttile;
[y, time] = impulse(H_p_rud, 10); 
plot(time, y)
ylabel('p [rad_s]');
nexttile; 
[y, time] = impulse(H_r_rud, 10); 
plot(time, y)
ylabel('r [rad/s]'); 

% plot aperiodic roll
fig4 = figure(4); 
[y, time] = step(H_p_ail, 7); 
plot(time, y)
xlabel('time [s]'); 
ylabel('p [rad/s]'); 
title('roll rate response to aileron step input (Aperiodic roll)'); 

% plot spiral
fig5 = figure(5); 
[y, time] = impulse(-H_phi_ail)
plot(time,y)
xlabel('time [s]'); 
ylabel('\phi [°]'); 
title('roll angle reponse to aileron impulse input (spiral)'); 


