clc;

% -- datos experimentales
% datos_export = [t_exp, posicion_ve,posicion_med, velocidad_carro_ve,
%velocidad_carro_med,tita_ve, tita_med ,vel_angular_ve,vel_angular_med, u];

filename = 'control_v2_1.txt';
T = readtable(filename, 'Delimiter', '\t');
t_exp = T{:, 1};
y_exp = T{:, 3};
y_exp = y_exp +8;
idx_inicio = 22;
t_exp = t_exp(idx_inicio:end);
y_exp = y_exp(idx_inicio:end);
t_exp = t_exp - t_exp(1);

% -- datos teoricos

Num = 1906;
Den = [1 39.1 623.2 1347 0];
Ts = 0.02;

s = tf('s');
z = tf('z', Ts);


% -- modelo teorico del controlador

A_d = [ 1.0000   0.02     0.0       0.0;
       0.0     0.9453   0.196     0.0;
       0.0      0.0      1.0      0.02;
       0.0      0.0    -10.7660   0.3392 ];

B_d = [ 0.0;
       0.0;
       0.0;
       3.696 ];

C_d = [1 0 0 0;
       0 0 1 0];
   
D_d = [0;0] ;

L = [ 
     1.0560,   -0.5249 ;
    4.9376,   -2.8050 ;
    0.0672 ,   0.5643 ;
    -2.6331 , -17.6447 ;
];

F = 1.3276;

%K   = [ 1.327624 0.71637 -0.174823 0.046391 ];
K   = [ 9.027624 3.41637 -0.274823 0.046391 ];

% calculos

% xpunto = (A + BK )x + BF r
A_d =A_d - B_d*K;
B_d = B_d * F;

sys = ss(A_d, B_d, C_d, D_d, Ts);   % sistema en espacio de estado discreto
G = tf(sys);                    % matriz de funciones de transferencia

t_final = 3;             % segundos
t = linspace(0, t_final, 2000);

G_1 = G(1,1) % posicion
G_2 = G(2,1) % tita

[g_imp, t_imp] = impulse(G_1, t_final); 

[g_step, t_step] = step(G_1, t_final)

%  Ajuste manual del offset temporal y de escala ---
offset_tiempo = 0;        % <--  para alinear (en segundos)
escala = 22;             % <--  para ajustar amplitud

t_imp = t_imp + offset_tiempo;
g_imp = g_imp * escala;

t_step = t_step + offset_tiempo;
g_step = g_step * escala;

figure('Color','w','Position',[200 200 800 420]);
%plot(t_step, g_step, 'Color', [0.47,0.67,0.19], 'LineWidth', 1.8); hold on;
plot(t_imp, g_imp, 'Color', [0.47,0.67,0.19], 'LineWidth', 1.8); hold on;
plot(t_exp, y_exp, 'Color', [0.35,0.70,0.90], 'LineWidth',1.5);
grid on;