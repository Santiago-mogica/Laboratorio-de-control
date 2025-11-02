% --- datos experimentales  ---
entradaEscalon = out.referencia();  % valores en cm 
y_exp = out.posicion();
t_exp = out.tout();

% Asegurar vectores columna
t_exp = t_exp(:);
entradaEscalon = entradaEscalon(:);
y_exp = y_exp(:);

% Forzar que la referencia empiece en 10 cm
if isempty(entradaEscalon)
	error('entradaEscalon vacío');
end
entradaEscalon(1) = 10;

% --- modelo teórico ---
Num = 1960;
Den = [1 59.7 759.1 1559 0];  % s^4 ... s^0
C_P = 1.8;
H1 = tf(Num, Den);
H_cl = H1 * C_P;  % Lazo cerrado
H_cl = feedback(H_cl, 1);

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
title('Referencia escalón vs medición','FontSize',13,'FontWeight','bold');
set(gca,'FontSize',11,'Box','off');

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