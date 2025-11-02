% Cargar datos del archivo .mat
data = load('modelo_ARX_out.mat');

% Extraer variables
t = data.t_sim;
y_real = data.y_real;
y_sim = data.y_sim;

% Gráfico de salida real vs simulada
figure;
plot(t, y_real, 'b', 'LineWidth', 1.5); hold on;
plot(t, y_sim, 'r--', 'LineWidth', 1.5);
xlabel('Tiempo [s]');
ylabel('Salida');
title('Comparación entre salida real y simulada');
legend('y_{real}', 'y_{sim}');
grid on;

% (Opcional) mostrar parámetros del modelo
fprintf('Parámetros del modelo:\n');
fprintf('alfa = %.4f\n', data.alfa);
fprintf('beta = %.4f\n', data.beta);
fprintf('gamma = %.4f\n', data.gamma);
