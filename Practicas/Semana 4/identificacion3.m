close all;
alfa = 0;
beta = 0;
gamma = 0;
x = out.x(50:end);    % vector columna
y = out.y(50:end);    % vector columna


A = [alfa;beta;gamma];
Y = y(3:end);
X = [y(2:(end-1)), y(1:(end-2)), x(3:(end))];
Ts = 0.02;
A = inv(transpose(X)*X)*transpose(X)*Y

p1 = (A(1)+sqrt(A(1)^2+4*A(2)))/2

p2 = (A(1)-sqrt(A(1)^2+4*A(2)))/2

p_cont1 = log(p1)/Ts
p_cont2 = log(p2)/Ts

% Ganancia medida (ejemplo usando tus datos)
y_ss = mean(out.y(end-50:end));
u_ss = mean(out.x(end-50:end));
%y_ss = mean(y);
%u_ss = mean(x);
K_meas = y_ss / u_ss;    % p.ej. ~ 5.7/15 = 0.38

% Construcción correcta: numerador n0 tal que G(0)=K_meas
prod_p_cont = p_cont1 * p_cont2;
n0 = K_meas * prod_p_cont;      % n0 = K_meas * p_c1 * p_c2
den = real(poly([p_cont1 p_cont2]));   % coeficientes reales del denominador
%no = 5000;
Gc_from_poles = tf(n0, den);    % planta continua con DC = K_meas

% Mostrar comprobaciones
fprintf('K_meas = %.6f\n', K_meas);
fprintf('prod_p_cont = %.6g\n', prod_p_cont);
fprintf('n0 = %.6g\n', n0);
fprintf('dcgain(Gc_from_poles) = %.6g\n', dcgain(Gc_from_poles));

% Simulaciones para comparar
t = (0:length(y)-1)'*Ts;
figure;
plot(t, y, 'b', 'DisplayName','y medida'); hold on;

% 1) lsim con la entrada real (ejemplo: escalón de 15)
y_sim_realinput = lsim(Gc_from_poles, x, t);
plot(t, y_sim_realinput, 'r--', 'DisplayName','sim P (misma entrada)');


legend; grid on; xlabel('t [s]'); ylabel('Angulo barra (grados)');
title('Comparación: medición vs modelo');