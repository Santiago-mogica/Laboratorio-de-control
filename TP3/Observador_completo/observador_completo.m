close all; 

%planta en continuo
s = tf('s');

Num = 1811;
Den = [1 35.78 628.7 1474 0];%s^4 + 35.78 s^3 + 628.7 s^2 + 1474 s // m/b 0.3653
% u = servo --> y1/u = posicion / servo
%           --> y2/u = tita / servo
% x1 = x 
% x2 = x punto
% x3 = θ
% x4 = θ punto

b_m = 1/0.3653;

A = [  0      1        0       0;
       0    -b_m     9.8     0;
       0      0        0       1;
       0      0     -538.3  -33.04 ];

B = [ 0; 0; 0; 184.8 ];

C = [ 1  0  0  0;   % Posicion
     0  0  1  0 ];  % Angulo
D = 0;

% comprobar que la TF coincide
sys = ss(A,B,C,D);
tf(sys)
%%
Ts = 0.02;

%polos deseados para el observador (más rápidos que los del sistema)
p_obs = 4.5* eig(A);

I = eye(4);
A_d = I + A*Ts
B_d = B*Ts
C_d =C;

p_obs_d = exp(p_obs * Ts)

%ganancia del observador L
L = place(A_d', C_d', p_obs_d)';

O = obsv(A,C);
rank(O)   % debe ser 4 para que el observador sea posible
