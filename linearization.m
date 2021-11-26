%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% script for linearizing both flight conditions and saving the results 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; 
clear all; 
close all; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% trim and linearization
%
% the results for trimming and linearization in both flight conditions
% are stored in corresponding text files. These files can then be imported
% into the script 
%
% no disturbances were provided to the systems. Linearization in level
% flight
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% flight condition 1 : 15000 ft and 500 ft/s
FindF16Dynamics
% Trim Values and Cost HIFI model:
%cost   = 7.1856e-06
%thrust = 2109.4129 lb
%elev   = -2.2441 deg
%ail    = -0.093578 deg
%rud    = 0.094469 deg
%alpha  = 4.5307 deg
%dLEF   = 6.2816 deg
%Vel.   = 500ft/s 

%Trim Values and Cost LOFI model:
%cost   = 4.2881e-29
%thrust = 2120.6214 lb
%elev   = -2.4607 deg
%ail    = 4.2733e-15 deg
%rud    = 3.6479e-14 deg
%alpha  = 4.4655 deg
%dLEF   = 0 deg
%Vel.   = 500ft/s
save('15000_500_lin')

% flight condition 2: 20000 ft and 900 ft/s
FindF16Dynamics

%Trim Values and Cost HIFI model:
%cost   = 9.1365e-07
%thrust = 3242.1874 lb
%elev   = -1.2202 deg
%ail    = -0.092037 deg
%rud    = -0.050848 deg
%alpha  = 0.66379 deg
%dLEF   = 0 deg
%Vel.   = 900ft/s

%Trim Values and Cost LOFI model:
%cost   = 2.2568e-26
%thrust = 3496.5275 lb
%elev   = -1.5615 deg
%ail    = 5.6461e-14 deg
%rud    = 2.6142e-13 deg
%alpha  = 0.70746 deg
%dLEF   = 0 deg
%Vel.   = 900ft/s
save('20000_900_lin')


