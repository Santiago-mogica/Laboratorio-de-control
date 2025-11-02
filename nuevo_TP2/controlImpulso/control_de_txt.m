% --- datos experimentales
filename = 'datos_experimentales_kp_1.8.txt';
T = readtable(filename, 'Delimiter', '\t');
t_exp = T{:, 1};
y_exp = T{:, 2}; 

% --- datos teoricos
Num = 1811;
%Den = [1 59.7 759.1 1559 0];  % planta pre servo
%Den = [1 39.1 623.2 1347 0]; %s^4 + 39.1 s^3 + 623.2 s^2 + 1347 s
%Den = [1 38.56 603.4 1061]; % s^4 + 38.56 s^3 + 603.4 s^2 + 1061 s
Den = [1 35.58 622.2 1367 0]; %s^4 + 35.58 s^3 + 622.2 s^2 + 1367 s
  

idx_inicio = 37;
t_exp = t_exp(idx_inicio:end);
y_exp = y_exp(idx_inicio:end);
t_exp = t_exp - t_exp(1);

C_P = 1.8;
H1 = tf(Num, Den);
H_cl = H1 * C_P;   % Lazo cerrado 
H_cl = feedback(H_cl, 1);

t_final = 3;             % segundos
t = linspace(0, t_final, 2000);

% --- Respuesta al impulso teórica ---
[y_imp, t_imp] = impulse(H_cl, t);
A = 10;  % magnitud del impulso
y_imp = A * y_imp;

% --- Gráfico ---
figure('Color','w','Position',[200 200 800 420]);
plot(t_imp, y_imp, 'Color', [0.47,0.67,0.19], 'LineWidth', 1.8); hold on;
plot(t_exp, y_exp, 'Color', [0.35,0.70,0.90], 'LineWidth',1.5);
legend('Teórico','Medición');

grid on;
xlabel('t [s]', 'FontSize', 12);
ylabel('Respuesta al impulso', 'FontSize', 12);
%title('Respuesta al impulso del lazo cerrado', 'FontSize', 13, 'FontWeight','bold');
set(gca, 'FontSize', 11, 'Box','off');

% --- exporto los datos
%datos_export = [t_exp, y_exp];          
%filename = 'datos_experimentales_kp_2.1_escalon.txt'; % Nombre del archivo
%writematrix(datos_export, filename, 'Delimiter','tab');

%% chequeos teoricos de  ChatGPT

%datos expertimantales
filename = 'datos_experimentales_kp_1.8.txt';
T = readtable(filename, 'Delimiter', '\t');
t_exp = T{:, 1};
y_exp = T{:, 2}; 

%datos teoricos
Num = 1811;
Den = [1 35.58 622.2 1367 0]; %s^4 + 35.58 s^3 + 622.2 s^2 + 1367 s
  
idx_inicio = 37;
t_exp = t_exp(idx_inicio:end);
y_exp = y_exp(idx_inicio:end);
t_exp = t_exp - t_exp(1);

C_P = 1.8;
H1 = tf(Num, Den);
H_cl = H1 * C_P;   % Lazo cerrado 
H_cl = feedback(H_cl, 1);

t_final = 3;             % segundos
t = linspace(0, t_final, 2000);

% --- Respuesta al impulso teórica ---
[y_theo, t_theo] = impulse(H_cl, t);

% supongamos:
% t_exp, y_exp -> de la medición (mismas unidades)
% t_theo, y_theo -> de la simulación

% 1) Interpolar teoría a tiempos experimentales
y_theo_interp = interp1(t_theo, y_theo, t_exp, 'pchip', 'extrap');

% 2) Quitar offsets iniciales (opcional)
y_exp0 = y_exp - y_exp(1);
y_theo0 = y_theo_interp - y_theo_interp(1);

% 3) Plot y error
figure;
plot(t_exp, y_theo0, 'LineWidth',2); hold on;
plot(t_exp, y_exp0, ':');
legend('Teórico (interp)','Medición');
xlabel('t [s]'); ylabel('Respuesta al impulso');
grid on;

% 4) Métricas
err = y_exp0 - y_theo0;
RMS = sqrt(mean(err.^2));
NRMSE = 100*(sqrt(mean(err.^2))/ (max(y_theo0)-min(y_theo0)));
fprintf('RMS error = %.3f, NRMSE = %.2f%%\n', RMS, NRMSE);

% 5) Correlación y retardo por cross-correlation
[c,lags] = xcorr(y_exp0, y_theo0, 200, 'coeff');
[~,idx] = max(abs(c));
lag = lags(idx) * mean(diff(t_exp)); % seconds approx
fprintf('Lag máximo ~ %.4f s\n', lag);

