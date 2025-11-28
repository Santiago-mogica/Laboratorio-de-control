%% ================================
%    CALCULO DE GANANCIA K
%    PARA BALL & BEAM DISCRETO
% =================================

 clc;

%% --- Matrices discretas del modelo ---
Ad = [ 1.0000   0.02     0.0       0.0;
       0.0     0.9453   0.196     0.0;
       0.0      0.0      1.0      0.02;
       0.0      0.0    -10.7660   0.3392 ];

Bd = [ 0.0;
       0.0;
       0.0;
       3.696 ];

%% ================================
%       METODO 1: PLACE
% =================================

% Polos discretos deseados (elegir cómo querés la dinámica)
poles_discretos = [0.89 0.85 0.81 0.6];

K_place = place(Ad, Bd, poles_discretos);

disp('------------------------------------')
disp(' Ganancia K calculada con PLACE:')
disp(K_place)

disp(' Autovalores lazo cerrado (PLACE):')
disp(eig(Ad - Bd*K_place))

%% ================================
%       METODO 2: LQR DISCRETO
% =================================

% Ajustar pesos según prioridad
Q = diag([100, 1, 50, 1]);   % ejemplo: control fuerte sobre p y theta
R = 1;

[K_lqr, S, e_lqr] = dlqr(Ad, Bd, Q, R);

disp('------------------------------------')
disp(' Ganancia K calculada con DLQR:')
disp(K_lqr)

disp(' Autovalores lazo cerrado (DLQR):')
disp(e_lqr)

%% ================================
%       COPIAR A ARDUINO
% =================================

fprintf('\n\n==== Copiar a Arduino (PLACE) ====\n');
fprintf('float K[4] = { %.6f, %.6f, %.6f, %.6f }; \n', K_place);

fprintf('\n==== Copiar a Arduino (DLQR) ====\n');
fprintf('float K[4] = { %.6f, %.6f, %.6f, %.6f }; \n', K_lqr);

%===============================
k_a_mano = [1.327624, 0.71637, -0.174823, 0.056391];
disp('------------------------------------')
disp(' Autovalores lazo cerrado (A MANO):')
disp(eig(Ad - Bd*k_a_mano))

%
posicion_ve = out.pos_ve();
posicion_med = out.pos_med();

velocidad_carro_ve = out.vel_carro_ve();
velocidad_carro_med = out.vel_carro_med();

tita_ve = out.tita_ve();
tita_med = out.tita_med();

vel_angular_ve = out.vel_angular_ve();
vel_angular_med = out.vel_angular_med();

u = out.u();
t_exp = out.tout();
% --- exporto los datos
% datos_export = [t_exp, posicion_ve,posicion_med, velocidad_carro_ve,velocidad_carro_med,tita_ve, tita_med ,vel_angular_ve,vel_angular_med, u];          
% filename = 'control_v1_2.txt'; % Nombre del archivo
% writematrix(datos_export, filename, 'Delimiter','tab');

%%
