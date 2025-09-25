close all;clc;

Gc = zpk([], [-3.167, -36.507], 115.44);

% === Diseño básico: compensador C(s) = Ki / s ===
s = tf('s');
%{
% --- Parámetro del integrador (ajustalo) ---
Ki = 986;
Kp = 9.72;
% --- Compensador integrador ---
C = (Ki *1 /s) +Kp ;
%}

C1 = pidtune(Gc,'PI',13.6);   % donde 13.6 rad/s ≈ ωn deseado
C2 = pidtune(Gc,'PI',10);   % más amortiguado, más lento
C3 = pidtune(Gc,'PI',11);   % más rápido, más sobrepaso


% === Guardar las constantes ===
Kp1 = C1.Kp;   Ki1 = C1.Ki;
Kp2 = C2.Kp;   Ki2 = C2.Ki;
Kp3 = C3.Kp;   Ki3 = C3.Ki;

% Mostrar en consola
fprintf('C1: Kp = %.4f , Ki = %.4f\n',Kp1,Ki1);
fprintf('C2: Kp = %.4f , Ki = %.4f\n',Kp2,Ki2);
fprintf('C3: Kp = %.4f , Ki = %.4f\n',Kp3,Ki3);
% === Lazos abiertos ===
L1 = C1*Gc;
L2 = C2*Gc;
L3 = C3*Gc;


%{
% --- Análisis lazo abierto ---
figure('Name','Bode Lazo Abierto'); bode(L); grid on;
title('Bode de L(s) = C(s) G(s)  (integrador)');

figure('Name','Margenes'); margin(L); grid on;
title('Margen de ganancia y de fase de L(s)');

% Mostrar crossover y margen numéricos
[GM, PM, wcg, wcp] = margin(L);
fprintf('Phase margin = %.2f deg at wc = %.3f rad/s\n', PM, wcp);
fprintf('Gain margin = %.2f dB at wg = %.3f rad/s\n', 20*log10(GM), wcg);
%}
% --- Lazo cerrado (retroalimentación unity) ---
T1 = feedback(L1, 1);   % transferencia referencia -> salida
T2 = feedback(L2, 1);
T3 = feedback(L3, 1);

% === Lazos cerrados acción de control ===
U1 = feedback(C1,Gc);  % referencia -> acción de control
U2 = feedback(C2,Gc);
U3 = feedback(C3,Gc);


% === Simulación escalón ===
t = 0:0.001:1;              % horizonte temporal
r = ones(size(t));          % escalón de amplitud 1

[y1,~] = lsim(T1,r,t);
[y2,~] = lsim(T2,r,t);
[y3,~] = lsim(T3,r,t);

[u1,~] = lsim(U1,r,t);
[u2,~] = lsim(U2,r,t);
[u3,~] = lsim(U3,r,t);

% === Gráficos ===
figure;
subplot(2,1,1)
plot(t,y1,'b','LineWidth',1.5); hold on;
plot(t,y2,'r','LineWidth',1.5);
plot(t,y3,'g','LineWidth',1.5);
legend('\omega_n=13.6','\omega_n=10','\omega_n=11');
title('Respuesta al escalón (salida)'); grid on;

subplot(2,1,2)
plot(t,u1,'b','LineWidth',1.5); hold on;
plot(t,u2,'r','LineWidth',1.5);
plot(t,u3,'g','LineWidth',1.5);
legend('\omega_n=13.6','\omega_n=10','\omega_n=11');
title('Acción de control u(t)'); grid on;
xlabel('Tiempo [s]');
