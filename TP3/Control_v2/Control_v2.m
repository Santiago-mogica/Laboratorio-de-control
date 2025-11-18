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

% -------------------------
% Calculo de F
% -------------------------
M = C * ((eye(size(Ad)) - (Ad - Bd*K)) \ Bd);
F = 1 / M;       % escalar

disp("F calculado:")
F

