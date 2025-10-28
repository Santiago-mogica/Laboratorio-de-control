close all; 

%planta en continuo
s = sym('s');
s = tf('s')
G = 0.3733 / (s^2 + 104.5*s + 1678);
step(G);

%modelo de espacio de estados
a1 = 104.5;
a2 = 1678;
b = 0.3733;

A = [0 1; -a2 -a1];
B = [0; b];
C = [1 0];
D = 0;

%sistema de espacio de estados
sys = ss(A, B, C, D);


Ts = 0.02;

%polos deseados para el observador (más rápidos que los del sistema)
p_obs = 5 * eig(A);

I = eye(2);
A_d = I + A*Ts;
B_d = B*Ts;
C_d =C;

p_obs_d = exp(p_obs * Ts);
%ganancia del observador L
L = place(A_d', C_d', p_obs_d)'; 

sys_d = c2d(sys, Ts, 'zoh');
[A_d1, B_d1, C_d1, D_d1] = ssdata(sys_d);

p_obs_d = exp(p_obs * Ts);
L1 = place(A_d1', C_d1', p_obs_d)';

%error de estimación dinámico
% A_obs = A - L*C;
% B_obs = [B L];
% C_obs = C;
% D_obs = [0 0];
% 
% sys_obs_c = ss(A_obs, B_obs, C_obs, D_obs);
% 
% Ts = 0.02;
% 
% %discretización
% sys_obs_d = c2d(sys_obs_c, Ts, 'zoh');
% [A_d, B_d, C_d, D_d] = ssdata(sys_obs_d);

% Respuesta del sistema real
%[y, t, x] = lsim(sys_obs_d, u, t);