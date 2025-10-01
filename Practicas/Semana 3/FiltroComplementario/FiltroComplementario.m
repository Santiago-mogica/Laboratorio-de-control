close all;
tita_g = out.tita_diverg;
tita_a = out.tita_a;
tita_estimada = out.tita_estimada;
t = out.tout;

plot(t, tita_g, 'Color', [0.85 0.33 0.10], 'LineWidth', 1, 'DisplayName', 'tita_g corregida'); % naranja quemado
hold on;
plot(t, tita_a, 'Color', [0.00 0.45 0.74], 'LineWidth', 1, 'DisplayName', 'tita_a'); % azul profundo
plot(t, tita_estimada, 'Color', [0.47 0.67 0.19], 'LineWidth', 1, 'DisplayName', 'tita_{estimada}'); % verde oliva
hold off;

xlabel('Tiempo (s)');
ylabel('Ángulo (grados)');
title('Comparación de señales alfa = 0.4');
legend('show');
grid on;