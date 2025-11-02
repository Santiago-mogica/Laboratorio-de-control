close all; 

tita_deg = out.tita_barra();   % primera columna
x_meas = out.posicion();   % segunda columna
t1 = out.tout();
idx_inicio = 3; 

x_meas = x_meas(idx_inicio:end);
tita_deg = tita_deg(idx_inicio:end);
t1 = t1(idx_inicio:end);

% Crear una máscara lógica que elimine las filas donde haya ceros
mask = (tita_deg ~= 0) & (x_meas ~= 0);

% filtrar cuando supera 35
idx_limite = find(x_meas > 35   , 1);

if ~isempty(idx_limite)
    % Anular todas las muestras desde ese punto en adelante
    mask(idx_limite:end) = false;
end

% Aplicar el filtro
tita_deg = tita_deg(mask);
x_meas = x_meas(mask);
t1 = t1(mask);

Ts = 0.02; g = 9.81;

% Use the measured time vector (baseline at zero). If lengths don't match,
% fall back to a uniform time base using Ts.
t_meas = t1 - t1(1);
if length(t_meas) ~= length(x_meas) || length(t_meas) ~= length(tita_deg)
    t_meas = (0:Ts:Ts*(length(x_meas)-1))'; % Vector de tiempo
end

% Convertir posición a metros (si está en cm)
x_m = x_meas / 100;
x0 = x_m(1);

% Asumimos theta constante (promedio de las mediciones)
theta_const_rad = mean(deg2rad(tita_deg));
fprintf('Usando ángulo promedio (constante) de: %.2f°\n', rad2deg(theta_const_rad));

% -----------------------------
% Ajuste de A y B y q: x = x0 + A*(exp(-t/q)-1) + B*t
model_general = @(p, t, x0) x0 + p(2) .* (exp(-t ./ p(1)) - 1) + p(3) .* t;
err_general = @(p) sum((model_general(p, t_meas, x0) - x_m).^2);

% Inicialización basada en el modelo físico
A0 = 0.4.^2 * g * sin(theta_const_rad);
B0 = 0.4 * g * sin(theta_const_rad);
p0 = [0.4, A0, B0];

% (fminsearch solo acepta double). busco no solo el q, sino que le mando
% los 3 parametros para fittear
t_meas = double(t_meas);
x_m = double(x_m);
tita_deg = double(tita_deg);
x0 = double(x0);
theta_const_rad = double(theta_const_rad);
p0 = double(p0);

p_fit = fminsearch(err_general, p0);
q_g = p_fit(1);
A_g = p_fit(2);
B_g = p_fit(3);
x_fit_general = model_general(p_fit, t_meas, x0);
fprintf('Ajuste libre: q=%.4f, A=%.4e, B=%.4e\n', q_g, A_g, B_g);

figure;
hold on;
grid on;

% colormap
cmap = parula(length(x_m)); % un color por punto
sz = 30; % tamaño de los círculos

% Plot de los datos medidos con círculos huecos y colores del cmap
scatter(t_meas, x_m, sz, 1:length(x_m), 'MarkerEdgeColor','flat', ...
        'MarkerFaceColor','none', 'LineWidth', 1.3);

% Plot del modelo ajustado como línea continua
plot(t_meas, x_fit_general, 'r-', 'LineWidth', 1.1, 'DisplayName', sprintf('Ajuste libre (q=%.3f)', q_g));

xlabel('Tiempo [s]');
ylabel('Posición [m]');
title(sprintf('Estimación de m/b mediante Ajuste (m/b = %.4f s)', q_g));
legend('Datos medidos', 'Modelo ajustado', 'Location', 'best');

hold off;

datos_export = [t_meas, x_m, theta_const_rad*ones(size(t_meas))];

% Guardar en un archivo de texto
T = table(t_meas, x_m, theta_const_rad*ones(size(t_meas)), ...
    'VariableNames', {'Tiempo_s','Posicion_m','Angulo_rad'});

% Guardar en un archivo .txt separado por tabulaciones
filename = 'mb8.txt';
writetable(T, filename, 'Delimiter', '\t');

fprintf('Datos exportados a %s\n', filename);

