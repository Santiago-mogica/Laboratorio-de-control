

close all; 
u = out.x(50:end);    % vector columna
u = u - 90;
y = out.y(50:end);    % vector columna
t = out.tout();    % vector de tiempos


% ======================
% 1. Armar regresores
% ======================
N = length(y) - 2;
Y = y(3:end);                              % salida actual
X = [ y(2:end-1), y(1:end-2), u(1:N) ];    % regresores
%}
% ======================
% 2. Resolver con pseudoinversa
% ======================
vectorAlfa = pinv(X) * Y;

alfa  = vectorAlfa(1);
beta  = vectorAlfa(2);
gamma = vectorAlfa(3);

% ======================
% 3. Modelo en diferencias
% y(k) = alfa*y(k-1) + beta*y(k-2) + gamma*u(k-2)
% ======================
%Ts = t(2) - t(1);    % tiempo de muestreo
Ts = 0.02;
num = [0 0 gamma];   % z^-2 → [0 0 γ]
den = [1 -alfa -beta];

Gz = tf(num, den, Ts);   % modelo discreto
Gc = d2c(Gz, 'zoh');     % modelo continuo aproximado

% ======================
% 4. Gráficas comparativas
% ======================
Nsim = N;   % cantidad de muestras a comparar
u_sim = u(1:Nsim);
t_sim = t(1:Nsim);

y_cont = lsim(Gc, u_sim, t_sim);   % salida simulada por el modelo
y_discr = lsim(Gz, u_sim, t_sim);   % salida simulada por el modelo
y_real = y(1:Nsim);               % salida medida real

% ======================
% 5. Graficar comparación
% ======================
cmap = parula(5);       % 5 colores del mapa magma
col1 = cmap(2,:);      % color para y_real
col2 = cmap(4,:);      % color para y_sim
col3 = cmap(3,:);      % color para y_sim

figure;
plot(t_sim, y_real, 'Color', col1, 'LineWidth', 1.8); hold on;
plot(t_sim, y_cont, '--', 'Color', col2, 'LineWidth', 1.8);
plot(t_sim, y_discr, '--', 'Color', col3, 'LineWidth', 1.8);

legend('Salida medida (y)', 'Salida modelo (Gc)', 'Salida modelo (Gz)');
xlabel('Tiempo [s]');
ylabel('Salida');
title('Comparación entre sistema real y modelo ARX');
grid on;

zpk(Gc)
pole(Gc)

% Guardar todo en un archivo .mat
%save('modelo_ARX_out.mat', 'alfa','beta','gamma','y_sim','y_real','t_sim', 'N', 'Y', 'X');

