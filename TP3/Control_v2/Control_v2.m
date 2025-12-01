%% ================================
%    CALCULO DE GANANCIA K
%    PARA BALL & BEAM DISCRETO
% =================================


%% --- Matrices discretas del modelo ---
Ad = [ 1.0000   0.02     0.0       0.0;
       0.0     0.9453   0.196     0.0;
       0.0      0.0      1.0      0.02;
       0.0      0.0    -10.7660   0.3392 ];

Bd = [ 0.0;
       0.0;
       0.0;
       3.696 ];
   
 
% Usar solo la fila del output controlado
% Si querés regular posición (y = x1):
Cd=[ 1 0 0 0;
    0 0 1 0 ];
C = Cd(1,:); % <-- elegís la salida a unity gain
K = [1.327624, 0.71637, -0.174823, 0.046391];
disp(' Autovalores lazo cerrado (A mano):')
disp(eig(Ad - Bd*K))
% -------------------------
% Calculo de F
% -------------------------
M = C * ((eye(size(Ad)) - (Ad - Bd*K)) \ Bd);
F = 1 / M;       % escalar

disp("F calculado:")
F

%%
% posicion_ve = out.pos_ve();
% posicion_med = out.pos_med();
% 
% velocidad_carro_ve = out.vel_carro_ve();
% velocidad_carro_med = out.vel_carro_med();
% 
% tita_ve = out.tita_ve();
% tita_med = out.tita_med();
% 
% vel_angular_ve = out.vel_angular_ve();
% vel_angular_med = out.vel_angular_med();
% 
% u = out.u();
% t_exp = out.tout();


% --- exporto los datos
%  datos_export = [t_exp, posicion_ve,posicion_med, velocidad_carro_ve,velocidad_carro_med,tita_ve, tita_med ,vel_angular_ve,vel_angular_med, u];          
% filename = 'control_v2_3.txt'; % Nombre del archivo
% writematrix(datos_export, filename, 'Delimiter','tab');
