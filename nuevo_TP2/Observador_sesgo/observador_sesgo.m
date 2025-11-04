close all; 

%planta en continuo
s = sym('s');
s = tf('s')
G= 184.8 / (s^2 + 33.04*s + 538.3);
a1 = 33.04;
a2 = 538.3;
b = 184.8;

%step(G);
% G = 194.5 / (s^2 + 36.56*s + 530.3);
% a1 = 36.56;
% a2 = 530.3;
% b = 194.5;


%modelo de espacio de estados

A = [0 1; -a2 -a1];
B = [0; b];
C = [1 0];
D = 0;

%sistema de espacio de estados
sys = ss(A, B, C, D);


Ts = 0.02;

%polos deseados para el observador (más rápidos que los del sistema)
p_obs = 3 * eig(A);

I = eye(2);
A_d = I + A*Ts;
B_d = B*Ts;
C_d =C;


%Aumentamos la dimension para incluir al sesgo como VE
A_d_aum = [A_d(1,1)  A_d(1,2)   0;
       A_d(2,1)  A_d(2,2)   0;
       0           0            1];

B_d_aum = [B_d(1);
       B_d(2);
       0];

C_d_aum = [1 0 0;
       0 1 1];

p_obs_d = exp(p_obs * Ts);

p_bias = 0.65;             % polo lento para estimar bias
p_obs_aum = [p_obs_d; p_bias];

%ganancia del observador L
L = place(A_d_aum', C_d_aum', p_obs_aum)'; 
%sys_d = c2d(sys, Ts, 'zoh');
%[A_d1, B_d1, C_d1, D_d1] = ssdata(sys_d);

