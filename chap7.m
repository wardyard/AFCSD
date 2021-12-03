

load('long_red2.mat')

states_lon_sp = [3 4]
inputs_lon_sp = [2]
%outputs_lon_sp = [1 2]
% Select AOA and q
%long_red2.A
A_ac_sp = long_red2.A([3 4], [3 4])
%long_red2.B
B_ac_sp = long_red2.B([3 4], [2])
%long_red2.C
C_ac_sp = long_red2.C([3 4], [3 4])
long_red2.D
D_ac_sp = long_red2.D([3 4], [2])

long_red_sp = ss(A_ac_sp, B_ac_sp, C_ac_sp, D_ac_sp, 'StateName', long_red2.StateName(states_lon_sp), ...
    'InputName', long_red2.InputName(inputs_lon_sp))