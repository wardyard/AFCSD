%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% script for exercise 5.1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%close all; 
clear all; 

% adjust lin_f16block
% trim and linearize low fidelity model with updated output and flight 
% condition 15000 ft and 500 ft/s
%x_a = 0
%FindF16Dynamics 
%save('15000_500_lin_lo_51', 'SS_lo')
% load obtained results
load('15000_500_lin_lo_51');
% now set the x_a/g_D gain to 0 in lin_f16block to simulate accelerometer
% at cg
% extract transfer functions 
tfs = tf(SS_lo);
% extract normal acceleration to elevator transfer function 
H_an_el = minreal(tfs(16,2))%BEWARE OF THE OUTPUT INDEX, MIGHT BE WRONG
% draw normal acceleration response to step elevator command
opt = stepDataOptions('StepAmplitude', -1);
%t = 0:0.0001:1400;
t = 0:0.0001:5;
[y,t] = step(H_an_el, t, opt);
figure(1);
plot(t,y);
xlabel('time [s]');
ylabel('Normal accelaration [g units]');
%title('Negative step input accelarometer response (long run)');
%title('Negative step input accelarometer initial response');
title('Negative step input accelarometer x position 0 ft');

hold on


x_a = 5
FindF16Dynamics 
save('15000_500_lin_lo_51', 'SS_lo')
load('15000_500_lin_lo_51');
tfs = tf(SS_lo);
H_an_el = minreal(tfs(16,2))
opt = stepDataOptions('StepAmplitude', -1);
[y1,t] = step(H_an_el, opt);
figure(2)
plot(t,y1)


x_a = 5.9
FindF16Dynamics 
save('15000_500_lin_lo_51', 'SS_lo')
load('15000_500_lin_lo_51');
tfs = tf(SS_lo);
H_an_el = minreal(tfs(1,2))
opt = stepDataOptions('StepAmplitude', -1);
[y2,t] = step(H_an_el, opt);
figure(8)
plot(t,y2)





x_a = 6
FindF16Dynamics 
save('15000_500_lin_lo_51', 'SS_lo')
load('15000_500_lin_lo_51');
tfs = tf(SS_lo);
H_an_el = minreal(tfs(1,2))
opt = stepDataOptions('StepAmplitude', -1);
[y3,t] = step(H_an_el, opt);
figure(8)
plot(t,y3)



x_a = 7
FindF16Dynamics 
save('15000_500_lin_lo_51', 'SS_lo')
load('15000_500_lin_lo_51');
tfs = tf(SS_lo);
H_an_el = minreal(tfs(1,2))
opt = stepDataOptions('StepAmplitude', -1);
[y4,t] = step(H_an_el, opt);
figure(8)
plot(t,y4)



x_a = 15
FindF16Dynamics 
save('15000_500_lin_lo_51', 'SS_lo')
load('15000_500_lin_lo_51');
tfs = tf(SS_lo);
H_an_el = minreal(tfs(1,2))
opt = stepDataOptions('StepAmplitude', -1);
[y5,t] = step(H_an_el, opt);
figure(8)
plot(t,y5)
legend("x_a = 0", "x_a = 5", "x_a = 5.9", "x_a = 6", "x_a = 7", "x_a = 15")
hold off



% draw pole zero map of the transfer function to hopefully find a zero in
% right half plane, explaining the averse effect of the step input 
pzmap(H_an_el)
title('Pole-zero map of the elevator-normal acceleration transfer function');
% there is a zero at (10,0), explaining the initial opposite direction 

%%%%%%%%%%%%%
% TF zero values for various accelerometer positions 
%%%%%%%%%%%%%

x_a = 0
pzmap(H_an_el)
hold on

x_a = 5
%first update the gain in simulink and save the model 
%then, linearize the model again
FindF16Dynamics

%extract corresponding transfer functions 
tfs2 = tf(SS_lo)
H_an_el_5 = minreal(tfs2(19,2))

%draw pole zero map 
fig2 = figure()
pzmap(H_an_el_5)

