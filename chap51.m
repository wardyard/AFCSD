%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% script for exercise 5.1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; 
clear all; 

% adjust lin_f16block

% trim and linearize low fidelity model with updated output and flight 
% condition 15000 ft and 500 ft/s
FindF16Dynamics 

save('15000_500_lin_lo_51', 'SS_lo')

% load obtained results
load('15000_500_lin_lo_51')

% now set the x_a/g_D gain to 0 in lin_f16block to simulate accelerometer
% at cg

% extract transfer functions 
tfs = tf(SS_lo); 

% extract normal acceleration to elevator transfer function 
H_an_el = minreal(tfs(19,2))

% draw normal acceleration response to step elevator command
step(-H_an_el)

% draw pole zero map of the transfer function to hopefully find a zero in
% right half plane, explaining the averse effect of the step input 
pzmap(H_an_el)
% there is a zero at (10,0), explaining the initial opposite direction 

%%%%%%%%%%%%%
% TF zero values for various accelerometer positions 
%%%%%%%%%%%%%

% x_a = 0
pzmap(H_an_el)
hold on

% x_a = 5
% first update the gain in simulink and save the model 
% then, linearize the model again
FindF16Dynamics

% extract corresponding transfer functions 
tfs2 = tf(SS_lo)
H_an_el_5 = minreal(tfs2(19,2))

% draw pole zero map 
fig2 = figure()
pzmap(H_an_el_5)

