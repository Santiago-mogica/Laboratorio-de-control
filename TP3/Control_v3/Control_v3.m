%% ================================
%    CALCULO DE GANANCIA K
%    PARA BALL & BEAM DISCRETO
% =================================


%% --- Matrices discretas del modelo ---
Ad = [ 1.0000   0.02     0.0       0.0;
       0.0     0.9453   0.196     0.0;
       0.0      0.0      1.0      0.02;
       0.0      0.0    -10.7660   0.3392 ];

Bd = [ 0.0;
       0.0;
       0.0;
       3.696 ];
   
 
% Usar solo la fila del output controlado
% Si querés regular posición (y = x1):
Cd=[ 1 0 0 0;
    0 0 1 0 ];
C = Cd(1,:); % <-- elegís la salida a unity gain

K = [1.327624, 0.71637, -0.174823, 0.046391];


Ts = 0.02;    % tiempo de muestreo

nx = size(Ad,1);       % cantidad de estados
ny = size(C,1);        % cantidad de salidas

% ====== MATRICES EXTENDIDAS ======
Ae = [Ad            zeros(nx,ny);
      -C*Ts         eye(ny)]

Be = [Bd;
      zeros(ny, size(Bd,2))]

% Entrada de referencia (para usar si querés feedforward)
Ee = [zeros(nx,ny);
      Ts*eye(ny)];

disp("Ae = "); disp(Ae);
disp("Be = "); disp(Be);
disp("Ee = "); disp(Ee);

% ====== DISEÑO DE K y H ======

% Elegí polos deseados en discreto (ejemplo)

poles_d = [0.4804 0.7360 0.8838 0.9759 0.8];   % tantos polos como nx+ny
K = [1.327624, 0.71637, -0.174823, 0.046391];

% Control por asignación de polos:
Ke = place(Ae, Be, poles_d)

K = Ke(1:nx);        % parte correspondiente a estados
H = Ke(nx+1:end);    % parte correspondiente al integrador

disp(' Autovalores lazo cerrado (A MANO):')
disp(eig(Ae - Be*Ke))
% ====== PRINTS PARA ARDUINO ======

fprintf("\n--- COPIAR A ARDUINO ---\n");

fprintf("\n// K\n");
fprintf("float K[%d] = {", nx);
fprintf("%f, ", K);
fprintf("};\n");

fprintf("\n// H\n");
fprintf("float H[%d] = {", ny);
fprintf("%f, ", H);
fprintf("};\n");

% === Print de Ae para Arduino ===
fprintf("\n// --- Matriz Ae (%dx%d) para Arduino ---\n", size(Ae,1), size(Ae,2));
fprintf("float Ae[%d][%d] = {\n", size(Ae,1), size(Ae,2));

for i = 1:size(Ae,1)
    fprintf("  {");
    for j = 1:size(Ae,2)
        if j == size(Ae,2)
            fprintf("% .4f", Ae(i,j));   % última columna sin coma
        else
            fprintf("% .4f, ", Ae(i,j));
        end
    end
    if i == size(Ae,1)
        fprintf("}\n};\n");
    else
        fprintf("},\n");
    end
end

% === Print de Be para Arduino ===
fprintf("\n// --- Matriz Be (%dx%d) para Arduino ---\n", size(Be,1), size(Be,2));
fprintf("float Be[%d][%d] = {\n", size(Be,1), size(Be,2));

for i = 1:size(Be,1)
    fprintf("  {");
    for j = 1:size(Be,2)
        if j == size(Be,2)
            fprintf("% .15f", Be(i,j));   % última columna sin coma
        else
            fprintf("% .15f, ", Be(i,j));
        end
    end
    if i == size(Be,1)
        fprintf("}\n};\n");
    else
        fprintf("},\n");
    end
end


