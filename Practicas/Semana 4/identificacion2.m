%% Datos
u = out.x();      % Entrada
y = out.y();      % Salida
t = out.tout;     % Tiempo
Ts = t(2) - t(1); % Tiempo de muestreo

%% Regresión (ya estimaste theta)
Y  = y(3:end);
Y1 = y(2:end-1);
Y2 = y(1:end-2);
U1 = u(2:end-1);

X = [Y1, Y2, U1];
theta = X \ Y;

a1 = theta(1);
a2 = theta(2);
b1 = theta(3);

fprintf('a1=%.6f, a2=%.6f, b1=%.6f\n', a1, a2, b1);

%% Polos discretos y continuos (OK)
coef = [1, -a1, -a2];
polos_discretos = roots(coef);
polos_continuos = log(polos_discretos) / Ts;

disp('Polos discretos:'); disp(polos_discretos);
disp('Polos continuos (rad/s):'); disp(polos_continuos);

%% Simulación del modelo discreto con condiciones iniciales reales
y_pred = zeros(size(y));
y_pred(1) = y(1);
y_pred(2) = y(2);
for k = 3:length(y)
    y_pred(k) = a1*y_pred(k-1) + a2*y_pred(k-2) + b1*u(k-1);
end

%% Ganancia estática medida y teórica
y_ss = mean(y(end-200:end));
u_ss = mean(u(end-200:end));
K_meas = y_ss / u_ss;
K_model = b1 / (1 - a1 - a2);

fprintf('Medido: y_ss=%.4f, u_ss=%.4f, K_meas=%.4f\n', y_ss, u_ss, K_meas);
fprintf('Model (discrete) K_model= b1/(1-a1-a2) = %.6f\n', K_model);

K = K_meas;                 % Ganancia medida (o K_model si prefieres)
p1 = polos_continuos(1);    % Polo 1 continuo
p2 = polos_continuos(2);    % Polo 2 continuo

%% ===========================
% Planta continua (sin ceros)
% ===========================
num_cont = K;               % Numerador = ganancia
den_cont = conv([1, -p1], [1, -p2]);  % Denominador a partir de polos

% Nota: Si p1 y p2 son complejos conjugados, conv maneja correctamente
G_s = tf(num_cont, den_cont);

disp('Planta continua:');
G_s
step(G_s)
%% ===========================
% Planta discreta usando el mismo Ts
% ===========================
G_z = c2d(G_s, Ts, 'zoh');  % Transformación ZOH

disp('Planta discreta:');
G_z
%% Ploteo comparación
figure;
subplot(2,1,1);
plot(t, y, 'b', 'DisplayName','y medido'); hold on;
plot(t, y_pred, 'r--', 'DisplayName','y predicho (modelo)'); grid on;
legend; xlabel('t [s]'); ylabel('y');

% Si querés comparar respuestas al mismo escalón con el modelo continuo:
subplot(2,1,2);
step(Gc_zoh * u_ss); title('Respuesta al escalón (modelo continuo aproximado)'); grid on;
