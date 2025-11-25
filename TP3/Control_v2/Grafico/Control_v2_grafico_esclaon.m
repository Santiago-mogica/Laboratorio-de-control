clc;

% -- datos experimentales
% datos_export = [t_exp, posicion_ve,posicion_med, velocidad_carro_ve = REF,
%velocidad_carro_med,tita_ve, tita_med ,vel_angular_ve,vel_angular_med, u];

filename = 'control_v2_escalon_1.txt';
T = readtable(filename, 'Delimiter', '\t');
t_exp = T{:, 1};
y_exp = T{:, 3};
entradaEscalon = T{:, 5};  % valores en cm 
y_exp = y_exp ;
idx_inicio = 22;
t_exp = t_exp(idx_inicio:end);
y_exp = y_exp(idx_inicio:end);
t_exp = t_exp - t_exp(1);

% Asegurar vectores columna
t_exp = t_exp(:);
entradaEscalon = entradaEscalon(:);
y_exp = y_exp(:);

% Forzar que la referencia empiece en 10 cm
if isempty(entradaEscalon)
	error('entradaEscalon vacío');
end
entradaEscalon(1) = 10;

% -- modelo teorico del controlador

A_d = [ 1.0000   0.02     0.0       0.0;
       0.0     1.01   0.196     0.0;
       0.0      0.0      1.0      0.02;
       0.0      0.0    -10.7660   0.3392 ];

B_d = [ 0.0;
       0.0;
       0.0;
       3.696 ];

C_d = [1 0 0 0;
       0 0 1 0];
   
D_d = [0;0] ;

F = 1.3276;

K   = [ 1.327624 0.71637 -0.174823 0.046391 ];
%K   = [ 9.027624 3.41637 -0.274823 0.046391 ];

% calculos

% xpunto = (A + BK )x + BF r
A_d =A_d - B_d*K;
B_d = B_d * F;

sys = ss(A_d, B_d, C_d, D_d, Ts);   % sistema en espacio de estado discreto
G = tf(sys);                    % matriz de funciones de transferencia

t_final = 3;             % segundos
t = linspace(0, t_final, 2000);

G_1 = G(1,1) % posicion
G_2 = G(2,1) % tita

H_cl = G_1
% Usar el vector de tiempo experimental para la simulación
% lsim admite tiempos no uniformes; así la salida teórica quedará alineada
t = t_exp;

% señal de referencia para la simulación apra seguir la referencia
if length(entradaEscalon) == length(t)
	r = entradaEscalon; % ya alineado
else
	% si no hay vector de tiempo propio, interpolao distribuido en t
	tin = linspace(t(1), t(end), length(entradaEscalon))';
	r = interp1(tin, entradaEscalon, t, 'previous', 'extrap');
end

if max(abs(y_exp)) < 5
	% y_exp probablemente está en metros -> convertir r a metros
	r_sim = double(r) / 100;
	y_teo = lsim(H_cl, r_sim, t);
else
	% mantener en cm
	r_sim = double(r);
	y_teo = lsim(H_cl, r_sim, t);
end

% Alinear la medición (interpolar si es necesario) para evitar desajustes
% Si y_exp no tiene la misma longitud que t, interpolar a t
if length(y_exp) == length(t)
	y_exp_al = y_exp;
else
	y_exp_al = interp1(t_exp, y_exp, t, 'linear', 'extrap');
end

% --- Gráfico (todo sobre el mismo eje temporal t) ---
figure('Color','w','Position',[200 200 900 460]); hold on;
plot(t, r, '--', 'Color', [0.91,0.41,0.17], 'LineWidth', 1.5, 'DisplayName', 'Referencia escalón');
plot(t, y_exp_al, '-', 'Color', [0.35,0.70,0.90], 'LineWidth', 1.5, 'DisplayName', 'Medición');
plot(t, y_teo, 'Color', [0.47,0.67,0.19], 'LineWidth', 1.8, 'DisplayName', 'Respuesta teórica');
legend('Location','best');
grid on;
xlabel('t [s]','FontSize',12);
ylabel('Posición / referencia','FontSize',12);
%title('Referencia escalón vs medición','FontSize',13,'FontWeight','bold');
set(gca,'PlotBoxAspectRatioMode','manual','PlotBoxAspectRatio',[2 1.2 1]);

% --- exporto los datos alineados (t, medición alineada, referencia original) ---
% Guardar en unidades originales: si convertimos r a metros para simular,
% exportaremos la referencia en la misma unidad que la medición alineada.
if max(abs(y_exp)) < 5
	% y_exp está en metros, asegurar r_export en m
	r_export = r_sim; % ya en m
	y_export = y_exp_al;
else
	r_export = r;     % en cm
	y_export = y_exp_al;
end

% --- exporto los datos
%datos_export = [t_exp, y_exp,entradaEscalon];          
%filename = 'datos_experimentales_kp_1.8_escalon_update.txt'; % Nombre del archivo
%writematrix(datos_export, filename, 'Delimiter','tab');