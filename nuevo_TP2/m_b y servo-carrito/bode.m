close all;
% opciones de Bode
optionss=bodeoptions;
optionss.MagVisible='on';
optionss.PhaseMatching='on';
optionss.PhaseMatchingValue=-180;
optionss.PhaseMatchingFreq=1;
optionss.Grid='on';

%%Análisis de la transferencia del sistema %antes era 2.54 = 1/0.393
b_m = 1/0.3653;

A = [  0      1        0       0;
       0    -b_m     9.8     0;
       0      0        0       1;
       0      0     -538.3  -33.04 ];

B = [ 0; 0; 0; 184.8 ];

C = [ 1  0  0  0 ];
D = 0;

A_eq = double(A);
B_eq = double(B);
C_eq = double(C);
D_eq = double(D);

% funcion transferencia
[Num, Den] = ss2tf(A_eq, B_eq, C_eq, D_eq);
H = tf(Num, Den);

bode(H, optionss);

wc =1.11; % método gráfico, para tener 60° de MF

fprintf('wc: %.3f rad/s\n', wc);

% Diseño P: escogemos Kp para llevar la ganancia a 0 dB en wc ---
[mag_wc, ph_wc] = bode(H, wc); mag_wc = squeeze(mag_wc);
Kp_P = 1 / mag_wc; % Kp para que |Kp*H(jwc)| = 1

%fprintf('P: Kp = %.3f, PM = %.2f deg (en w = %.3f)\n', Kp_P, PM_P, wP_P);

% experimentalmente se obtiene que kp = 1.815
% resulto demasiado elevado y oscila en torno a la referencia
C_P = 3;
L_P = H * C_P;

H_cl = feedback(L_P, 1);  % unity feedback

% --- Tiempo de simulación (ajustalo si hace falta) ---
t_final = 10;                  % segundos (aumentá si la dinámica es lenta)
t = linspace(0, t_final, 2000);

% Obtener respuesta al impulso
[y_imp, t_imp] = impulse(H_cl, t);

% Encontrar pico y tiempo de pico
[y_peak, idx_peak] = max(y_imp);
t_peak = t_imp(idx_peak);

% --- Plot bonito ---
figure('Color','w','Position',[200 200 800 420]);
plot(t_imp, y_imp, 'k', 'LineWidth', 1.8); hold on;
% marcar pico
plot(t_peak, y_peak, 'o', 'MarkerEdgeColor','k', 'MarkerFaceColor',[0.9 0.9 0.9], 'MarkerSize',8);
% línea vertical en tiempo de pico
xline(t_peak, '--k', 'LineWidth', 1);

% anotaciones
txt = sprintf('Peak = %.3g at t = %.3g s', y_peak, t_peak);
text(t_peak*1.02, y_peak, txt, 'FontSize', 10, 'Color','k');

grid on;
xlabel('t [s]', 'FontSize', 12);
ylabel('Respuesta al impulso', 'FontSize', 12);
title(sprintf('Respuesta al impulso del lazo cerrado con K_p = %.2f', C_P), 'FontSize', 13, 'FontWeight','bold');
set(gca, 'FontSize', 11, 'Box','off');






