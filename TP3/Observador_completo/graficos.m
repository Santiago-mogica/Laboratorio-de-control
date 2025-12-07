
%% Leer archivo exportado
filename = 'mediciones_observador_escalon.txt';
data = readmatrix(filename);

%% Extraer variables según el orden del archivo
t_exp = data(:,1);

% Variables medidas
p_med   = data(:,3);
theta_med = data(:,7);

% Variables estimadas (del modelo / observador)
p_est        = data(:,2);
dp_est       = data(:,4);
theta_est    = data(:,6);
dtheta_est   = data(:,8);

% Medidas restantes
dp_med     = data(:,5);
dtheta_med = data(:,9);

u = data(:,10);


%% ======== Cortar a los primeros 5 segundos ========
idx = t_exp <= 5;

t_exp = t_exp(idx);

p_est      = p_est(idx);
p_med      = p_med(idx);

dp_est     = dp_est(idx);
dp_med     = dp_med(idx);

theta_est  = theta_est(idx);
theta_med  = theta_med(idx);

dtheta_est = dtheta_est(idx);
dtheta_med = dtheta_med(idx);

u = u(idx);
%% ======== FIGURA 1: Posición ========
figure;
plot(t_exp, p_est, 'LineWidth', 1.4); hold on;
plot(t_exp, p_med, '--', 'LineWidth', 1.4);
grid on;
title('Posición');
xlabel('Tiempo [s]');
ylabel('p [m]');
legend('Estimado','Medido');

%% ======== FIGURA 2: Velocidad del carro ========
figure;
plot(t_exp, dp_est, 'LineWidth', 1.4); hold on;
plot(t_exp, dp_med, '--', 'LineWidth', 1.4);
grid on;
title('Velocidad carro');
xlabel('Tiempo [s]');
ylabel('dp/dt [m/s]');
legend('Estimado','Medido');

%% ======== FIGURA 3: Ángulo del péndulo ========
figure;
plot(t_exp, theta_est, 'LineWidth', 1.4); hold on;
plot(t_exp, theta_med, '--', 'LineWidth', 1.4);
grid on;
title('Ángulo \theta');
xlabel('Tiempo [s]');
ylabel('\theta [rad]');
legend('Estimado','Medido');

%% ======== FIGURA 4: Velocidad angular ========
figure;
plot(t_exp, dtheta_est, 'LineWidth', 1.4); hold on;
plot(t_exp, dtheta_med, '--', 'LineWidth', 1.4);
grid on;
title('Velocidad angular ');
xlabel('Tiempo [s]');
ylabel('d\theta/dt [rad/s]');
legend('Estimado','Medido');