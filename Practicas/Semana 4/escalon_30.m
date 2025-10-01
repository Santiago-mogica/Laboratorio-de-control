close all; 
u = out.x(50:end);    % vector columna
u = u - 90;
y = out.y(50:end);    % vector columna
y = -3*y;
t = out.tout(50:end);    % vector de tiempos
Ts = t(2) - t(1);    % tiempo de muestreo

% Armar matriz con tiempo y señal
data = [t y];

% Guardar en un archivo de texto
writematrix(data, 'angulo_alfa35_v2.txt', 'Delimiter', 'tab');


% Graficar
figure;
plot(t, y, 'r', 'LineWidth', 1.5);
xlabel('Tiempo [s]');
%xlim([1.2, 1.6]);
ylabel('Angulo[grad]');
grid on;
title('Ángulo de la barra ante un escalon de 30 grados');