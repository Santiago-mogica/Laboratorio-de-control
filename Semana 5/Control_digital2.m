%% Diseño de Controlador PI para Planta Específica
clear all; close all; clc;

% Parámetros de la planta original
z = [];        % ceros
p = [-19.4578+20.2298i, -19.4578-20.2298i];
k = 12.471;    % ganancia

% Crear la función de transferencia en espacio continuo
Gc = zpk(z, p, k);
fprintf('Planta original:\n');
Gc

% Especificaciones de diseño
ts_deseado = 0.5;      % Tiempo de establecimiento deseado (segundos)
sobrepico_deseado = 10; % Sobrepico deseado (%)

% Convertir especificaciones a parámetros de segundo orden
zeta = -log(sobrepico_deseado/100) / sqrt(pi^2 + log(sobrepico_deseado/100)^2);
wn = 4 / (zeta * ts_deseado);

fprintf('\nEspecificaciones de diseño:\n');
fprintf('Tiempo de establecimiento deseado: %.2f s\n', ts_deseado);
fprintf('Sobrepico deseado: %.1f%%\n', sobrepico_deseado);
fprintf('Zeta calculado: %.4f\n', zeta);
fprintf('Frecuencia natural wn: %.4f rad/s\n', wn);

% Polos deseados para el sistema en lazo cerrado
polos_deseados = [-zeta*wn + 1i*wn*sqrt(1-zeta^2), -zeta*wn - 1i*wn*sqrt(1-zeta^2)];
fprintf('\nPolos deseados: %.4f ± %.4fi\n', real(polos_deseados(1)), imag(polos_deseados(1)));

%% Método práctico: Sintonización empírica basada en la respuesta
% Probaremos diferentes valores y verificaremos las especificaciones

% Rango de valores a probar (optimizado para tu planta)
Kp_range = 3:0.2:7;
Ki_range = 15:1:40;

mejor_Kp = 0;
mejor_Ki = 0;
mejor_ts = inf;
mejor_sobrepico = inf;
mejor_error = inf;

fprintf('\nBuscando mejores parámetros por simulación...\n');

figure(1);
hold on; grid on;

for Kp = Kp_range
    for Ki = Ki_range
        % Controlador PI: C(s) = Kp + Ki/s
        C = tf([Kp Ki], [1 0]);
        
        % Sistema en lazo cerrado
        G_cl = feedback(C * Gc, 1);
        
        % Respuesta al escalón
        [y, t] = step(G_cl, 1);
        
        if ~isempty(y) && max(y) > 0
            % Calcular características de la respuesta
            [sobrepico, ts, tr, tp, ess] = calcularCaracteristicas(y, t);
            
            if ts > 0 && sobrepico >= 0
                % Función de costo que prioriza ts ≈ 0.5s y sobrepico ≈ 10%
                error_ts = abs(ts - ts_deseado) / ts_deseado;
                error_sp = abs(sobrepico - sobrepico_deseado) / sobrepico_deseado;
                error_total = error_ts + error_sp;
                
                if error_total < mejor_error
                    mejor_error = error_total;
                    mejor_Kp = Kp;
                    mejor_Ki = Ki;
                    mejor_ts = ts;
                    mejor_sobrepico = sobrepico;
                end
            end
        end
    end
end

fprintf('\n=== RESULTADOS OPTIMIZADOS ===\n');
fprintf('Kp óptimo = %.4f\n', mejor_Kp);
fprintf('Ki óptimo = %.4f\n', mejor_Ki);
fprintf('Tiempo de establecimiento logrado: %.3f s\n', mejor_ts);
fprintf('Sobrepico logrado: %.2f%%\n', mejor_sobrepico);

%% Simulación final con los mejores parámetros
C_optimo = tf([mejor_Kp mejor_Ki], [1 0]);
G_cl_optimo = feedback(C_optimo * Gc, 1);

figure(2);
step(G_cl_optimo, 1);
grid on;
title(sprintf('Respuesta al Escalón - Kp=%.3f, Ki=%.3f', mejor_Kp, mejor_Ki));
xlabel('Tiempo (s)');
ylabel('Amplitud');

% Características de la respuesta óptima
[y_opt, t_opt] = step(G_cl_optimo, 1);
[sobrepico_opt, ts_opt, tr_opt, tp_opt, ess_opt] = calcularCaracteristicas(y_opt, t_opt);

fprintf('\n=== CARACTERÍSTICAS FINALES ===\n');
fprintf('Tiempo de subida: %.3f s\n', tr_opt);
fprintf('Tiempo pico: %.3f s\n', tp_opt);
fprintf('Tiempo de establecimiento: %.3f s\n', ts_opt);
fprintf('Sobrepico: %.2f%%\n', sobrepico_opt);
fprintf('Error en estado estacionario: %.4f\n', ess_opt);

% Mostrar función de transferencia del sistema controlado
fprintf('\n=== FUNCIONES DE TRANSFERENCIA ===\n');
fprintf('Controlador PI:\n');
C_optimo
fprintf('Sistema en lazo cerrado:\n');
G_cl_optimo

%% Análisis de robustez
figure(3);
subplot(2,1,1);
margin(C_optimo * Gc);
title('Margen de Fase y Ganancia');

subplot(2,1,2);
rlocus(C_optimo * Gc);
title('Lugar de las Raíces');

%% Cálculo de parámetros para implementación discreta en Arduino
Ts_arduino = 0.02; % 20ms

% Coeficientes para implementación Tustin (bilineal)
a0 = mejor_Kp + (mejor_Ki * Ts_arduino / 2);
a1 = -mejor_Kp + (mejor_Ki * Ts_arduino / 2);

fprintf('\n=== IMPLEMENTACIÓN EN ARDUINO ===\n');
fprintf('Periodo de muestreo: Ts = %.3f s\n', Ts_arduino);
fprintf('a0 = Kp + Ki*Ts/2 = %.6f\n', a0);
fprintf('a1 = -Kp + Ki*Ts/2 = %.6f\n', a1);
fprintf('\nEcuación discreta del controlador:\n');
fprintf('u[k] = u[k-1] + %.6f*e[k] + %.6f*e[k-1]\n', a0, a1);

% Verificación con discretización
C_d = c2d(C_optimo, Ts_arduino, 'tustin');
fprintf('\nControlador discreto verificado:\n');
tf(C_d)

% Comparación respuesta continua vs discreta
figure(4);
G_cl_d = feedback(c2d(C_optimo, Ts_arduino, 'tustin') * c2d(Gc, Ts_arduino, 'tustin'), 1);
step(G_cl_optimo, 'r-', G_cl_d, 'b--', 1);
legend('Continuo', 'Discreto', 'Location', 'best');
title('Comparación: Sistema Continuo vs Discreto');
grid on;



%% Código Arduino generado automáticamente
fprintf('\n=== CÓDIGO ARDUINO ===\n');
fprintf('// Parámetros del controlador PI\n');
fprintf('#define TS 0.02f\n');
fprintf('float Kp = %.6ff;\n', mejor_Kp);
fprintf('float Ki = %.6ff;\n', mejor_Ki);
fprintf('float a0 = %.6ff; // Kp + Ki*TS/2\n', a0);
fprintf('float a1 = %.6ff; // -Kp + Ki*TS/2\n\n', a1);

fprintf('// Variables de control\n');
fprintf('float error = 0, error_prev = 0;\n');
fprintf('float u = 0, u_prev = 0;\n\n');

fprintf('void loop() {\n');
fprintf('    // 1. Leer referencia y salida de la planta\n');
fprintf('    float referencia = ...; // tu setpoint\n');
fprintf('    float salida_planta = ...; // lectura del sensor\n\n');
    
fprintf('    // 2. Calcular error\n');
fprintf('    error_prev = error;\n');
fprintf('    error = referencia - salida_planta;\n\n');
    
fprintf('    // 3. Calcular acción de control (forma discreta Tustin)\n');
fprintf('    u = u_prev + a0 * error + a1 * error_prev;\n\n');
    
fprintf('    // 4. Aplicar saturación si es necesario\n');
fprintf('    u = constrain(u, -100, 100); // ajustar límites\n\n');
    
fprintf('    // 5. Aplicar a la planta\n');
fprintf('    aplicarControl(u); // tu función de actuación\n\n');
    
fprintf('    // 6. Actualizar variables\n');
fprintf('    u_prev = u;\n\n');
    
fprintf('    // 7. Esperar periodo de muestreo\n');
fprintf('    delay(TS * 1000);\n');
fprintf('}\n');

%% Función para calcular características de la respuesta
function [sobrepico, ts, tr, tp, ess] = calcularCaracteristicas(y, t)
    % Valor final
    y_final = y(end);
    
    if y_final == 0
        sobrepico = 0; ts = inf; tr = inf; tp = inf; ess = 1;
        return;
    end
    
    % Sobrepico
    [y_max, idx_max] = max(y);
    sobrepico = max(0, (y_max - y_final) / y_final * 100);
    
    % Tiempo pico
    tp = t(idx_max);
    
    % Tiempo de subida (10% a 90%)
    idx_10 = find(y >= 0.1 * y_final, 1);
    idx_90 = find(y >= 0.9 * y_final, 1);
    if ~isempty(idx_10) && ~isempty(idx_90)
        tr = t(idx_90) - t(idx_10);
    else
        tr = NaN;
    end
    
    % Tiempo de establecimiento (±2%)
    y_target_high = 1.02 * y_final;
    y_target_low = 0.98 * y_final;
    
    idx_settle = find(y < y_target_low | y > y_target_high, 1, 'last');
    if ~isempty(idx_settle)
        ts = t(idx_settle);
    else
        ts = t(end);
    end
    
    % Error en estado estacionario
    ess = abs(1 - y_final);
end