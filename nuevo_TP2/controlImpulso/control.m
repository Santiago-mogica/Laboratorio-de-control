%close all;

% --- datos experimentales
t_exp = out.tout();
y_exp = out.posicion();

% --- datos teóricos
Num = 1811;
Den = [1 35.58 622.2 1367 0]; %s^4 + 35.58 s^3 + 622.2 s^2 + 1367 s
Den = [1 36.81 662.9 2030 0]; %s^4 + 36.81 s^3 + 662.9 s^2 + 2030 s
Den = [1 35.78 628.7 1474 0];%s^4 + 35.78 s^3 + 628.7 s^2 + 1474 s // m/b 0.3653
kp = 1.35;
ki = 0.0;
kd = 0.0004;
Ts = 0.02;

s = tf('s');
z = tf('z', Ts);

% Controlador PID bilineal (Tustin)
C_s = kp + ki/s + kd*s;
C_z = c2d(C_s, Ts, 'tustin');

% Planta discreta
H1 = tf(Num, Den);
H1_z = c2d(H1, Ts, 'tustin');

% Lazo cerrado discreto
H_cl = feedback(H1_z * C_z, 4);

% --- Simulación teórica
t_final = 10;
t = 0:Ts:t_final;                     
[y_imp, t_imp] = impulse(H_cl, t);

% --- Preparar datos experimentales
idx = 4;                              
t_exp = t_exp(idx:end);
y_exp = y_exp(idx:end);
t_exp = t_exp - t_exp(1);             

% --- exporto los datos
datos_export = [t_exp, y_exp];          
filename = 'kp_3_prueba_3.txt'; % Nombre del archivo
writematrix(datos_export, filename, 'Delimiter','tab');

% --- 🔧 Ajuste manual del offset temporal y de escala ---
offset_tiempo = 0.8;   % <-- EDITÁ ESTE VALOR para alinear (en segundos)
escala = 15;             % <-- EDITÁ ESTE VALOR para ajustar amplitud

% Aplicar desplazamiento y escala
t_imp_shift = t_imp + offset_tiempo;
y_imp_shift = escala * y_imp;

% Interpolar la teórica sobre los tiempos experimentales
y_imp_on_texp = interp1(t_imp_shift, y_imp_shift, t_exp, 'linear', 0);

% --- Gráfico ---
figure('Color','w','Position',[200 200 800 420]);
plot(t_exp, y_exp, 'Color', [0.35,0.70,0.90], 'LineWidth',1.5); hold on;
plot(t_exp, y_imp_on_texp, 'Color', [0.47,0.67,0.19], 'LineWidth',1.8);
legend('Medición','Teórico', 'Location','best');
grid on;
xlabel('t [s]', 'FontSize', 12);
ylabel('Respuesta al impulso', 'FontSize', 12);
%title(sprintf('Comparación (offset = %.2f s, escala = %.1f)', offset_tiempo, escala), ...
 %     'FontSize', 13, 'FontWeight','bold');
set(gca, 'FontSize', 11, 'Box','off');
