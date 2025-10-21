%% ===========================
% Estimación del parámetro m/b
% ============================

clc; close all;

% --- Parámetros físicos ---
g = 9.81;   % [m/s^2]
Ts = 0.02;  % [s] periodo de muestreo


% Leer datos del archivo
datos = load('datos.txt');
x = datos(:,1);
tita_deg = datos(:,2);
%% Modelo en espacio de estados (paramétrico en b/m) y simulación con theta(t)
% Guardá y ejecutá. Cambiá b_over_m manualmente y volvé a correr para iterar.


%% 1. SIMULACIÓN DEL MODELO TEÓRICO (FIGURA 5)
% Este bloque recrea la curva teórica usando los parámetros
% que el documento estimó.

% --- Parámetros del experimento ---
g = 9.81; % m/s^2
theta_deg = 11; % Ángulo de la barra [grados]
theta_rad = deg2rad(theta_deg);

% --- Parámetro estimado del documento ---
q = 0.31; % m/b [s]
b_so_m = 1 / q; % b/m [s^-1]

% --- Condiciones iniciales ---
% Asumimos que parte de la posición x=0 y velocidad v=0
x0 = 0; % [m]

% --- Definición del modelo teórico ---
% q = m/b
% t = vector de tiempo
modelo_x = @(q, t, x0) x0 + q^2*g*sin(theta_rad) * (exp(-t/q) - 1) + q*g*sin(theta_rad) * t;

% --- Generar simulación ---
x_sim = modelo_x(q, t, x0);

% --- Graficar (Esto recrea la línea roja de la Fig. 5) ---
figure;
plot(t, x_sim, 'r-', 'LineWidth', 2);
title('Recreación de la Curva Teórica (Fig. 5)');
xlabel('Tiempo (s)');
ylabel('Posición x (m)');
grid on;
hold on;

%% 1. Carga y Preparación de Datos
% ------------ Datos medidos -------------
tita_deg = [10.802479; 12.448374; 11.972658; 11.731621; 11.983871; ...
            13.005287; 12.475065; 11.489319; 9.8963699; 11.921037; ...
            11.066589; 12.332716];

x_meas = [1.89503872394562; 2.57793545722961; 3.80714988708496; ...
          6.74360609054565; 9.40690422058106; 11.7287530899048; ...
          14.1871814727783; 17.4650859832764; 20.0600948333740; ...
          25.1135311126709; 28.0499877929688; 31.2596015930176];

      
% tita_deg = [4.1134310;
%             4.4334679;
%             3.6728892;
%             2.7826369;
%             3.7954116;
%             3.8190393;
%             4.0064459;
%             2.2692270;
%             2.8757730;
%             4.4247956;
%             3.0187078];      
% x_meas = [1.5535903;
%           4.4900465;
%           6.4021578;
%           8.0411100;
%           10.089801;
%           13.845734;
%           16.099293;
%           19.104038;
%           23.679447;
%           27.503670;
%           30.986444];   


t = (0:0.02:0.22)'; % vector tiempo

% --- Constantes y Conversión de Unidades ---
g = 9.81; % m/s^2

% Convertir posición a metros (asumiendo que x_meas está en cm)
x_m = x_meas / 100;
x0 = x_m(1); % Condición inicial de posición

% ADVERTENCIA: El modelo asume theta constante. Usamos el promedio.
theta_const_rad = mean(deg2rad(tita_deg));
fprintf('Usando ángulo promedio (constante) de: %.2f grados\n', rad2deg(theta_const_rad));

%% 2. Estimación con lsqcurvefit (Punto 2)
% Parámetro a estimar: q = m/b
% Ecuación: x(t) = x0 + q^2*g*sin(th)*(exp(-t/q) - 1) + q*g*sin(th)*t

% Definimos la función del modelo anónima
modelo_fit = @(q, t) x0 + q.^2*g*sin(theta_const_rad) .* (exp(-t./q) - 1) + q*g*sin(theta_const_rad) .* t;

% Suposición inicial para q = m/b (en segundos)
q_inicial = 0.1; % Basado en el valor del paper (0.31)

% Ejecutar el ajuste de curvas
% Nota: Desactivamos el display de la iteración para limpieza
options = optimoptions('lsqcurvefit', 'Display', 'off');
q_estimado = lsqcurvefit(modelo_fit, q_inicial, t, x_m, [], [], options);

% --- Resultados ---
mb_estimado = q_estimado;
bm_estimado = 1 / q_estimado;

fprintf('--- Estimación (Método lsqcurvefit) ---\n');
fprintf('m/b estimado: %.4f s\n', mb_estimado);
fprintf('b/m estimado: %.4f s^-1\n', bm_estimado);

%% 3. Gráfica de Validación (Método lsqcurvefit)
% Generamos la curva del modelo con el parámetro 'q_estimado'
t_fit = linspace(min(t), max(t), 200);
x_fit = modelo_fit(q_estimado, t_fit);

figure;
plot(t, x_m, 'bo', 'LineWidth', 1.5, 'MarkerSize', 8, 'DisplayName', 'Datos Experimentales');
hold on;
plot(t_fit, x_fit, 'r-', 'LineWidth', 2, 'DisplayName', 'Modelo Ajustado (Punto 2)');
title('Estimación (Punto 2) vs. Datos Reales');
xlabel('Tiempo (s)');
ylabel('Posición x (m)');
legend;
grid on;