%% ===========================
% Datos
% ===========================
u = out.x();   % vector columna
u = u - 90;          % centrar en 0
y = out.y();   % vector columna
t = out.tout;        % vector de tiempo
Ts = t(2) - t(1);    % tiempo de muestreo



%% ===========================
% Preparar matriz de regresión
% Modelo: y3 = a*y2 + b*y1 + c*u3
%% ===========================
Y  = y(3:end);       % y3
Y2 = y(2:end-1);     % y2
Y1 = y(1:end-2);     % y1
U3 = u(3:end);       % u3

% Matriz de regresión
X = [Y2, Y1, U3];

%% ===========================
% Estimar coeficientes usando mínimos cuadrados
%% ===========================
theta = X \ Y;       % solución de mínimos cuadrados

a = theta(1);
b = theta(2);
c = theta(3);

fprintf('Coeficientes estimados:\n a = %.4f\n b = %.4f\n c = %.4f\n', a, b, c);

%% ===========================
% Calcular polos
%% ===========================
coef = [1, -a, -b];
raices = roots(coef);

polos_discretos = raices(:).';   % fila
polos_continuos = log(polos_discretos)/Ts;

fprintf('Polos discretos:\n');
disp(polos_discretos);
fprintf('Polos continuos:\n');
disp(polos_continuos);

%% ===========================
% 5. Simular salida con el modelo discreto
%% ===========================
y_pred = a*Y2 + b*Y1 + c*U3;   % vector directamente

%% ===========================
% 6. Calcular K a partir de estado estacionario
%% ===========================
y_ss = mean(y(end-100:end));   % promedio últimas 100 muestras
u_ss = mean(u(end-100:end));

prod_polos = prod(polos_continuos);

K = (y_ss / u_ss) * real(prod_polos);

fprintf('\nValor en estado estacionario (medido) = %.4f\n', y_ss);
fprintf('Entrada (medido) = %.4f\n', u_ss);
fprintf('Ganancia K calculada = %.4f\n', K);

%% ===========================
% 7. Transferencia continua identificada
%% ===========================
num = K;
den = real(poly(polos_continuos));   % asegurar coef reales

G = tf(num, den);

disp('=== Función de transferencia continua identificada ===');
G

disp('=== Ganancia y polos ===');
disp(K);
disp(polos_continuos);

%% ===========================
% 8. Comparación modelo discreto vs datos
%% ===========================
figure;
plot(Y, 'b', 'DisplayName','Medición real'); hold on;
plot(y_pred, 'r--', 'DisplayName','Predicción modelo');
xlabel('Muestra'); ylabel('y');
legend; grid on;
title('Comparación modelo discreto vs datos');

%% ===========================
% 9. Respuesta al escalón del sistema continuo
%% ===========================
t_sim = linspace(0, 1, 1000);
u_step = ones(size(t_sim));
[y_out, t_out] = lsim(G, u_step, t_sim);

y_final = y_out(end);
y_10 = 0.1 * y_final;
y_90 = 0.9 * y_final;

idx_start = find(y_out >= y_10, 1);
idx_end   = find(y_out >= y_90, 1);
t_rise_start = t_out(idx_start);
t_rise_end   = t_out(idx_end);
rise_time = t_rise_end - t_rise_start;

fprintf('\nTiempo de crecimiento (10-90%%): %.4f s\n', rise_time);

figure;
plot(t_out, y_out, 'b', 'DisplayName','Salida'); hold on;
yline(y_10, 'r--', '10% y_{final}');
yline(y_90, 'g--', '90% y_{final}');
xline(t_rise_start, 'r:');
xline(t_rise_end, 'g:');
xlabel('Tiempo (s)'); ylabel('Salida');
grid on; legend;
title(sprintf('Respuesta al escalón - Tiempo de crecimiento = %.3f s', rise_time));
