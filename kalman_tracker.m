function estimates = kalman_tracker(target_real)

%PASO 0:
T = 4; % Tiempo de muestreo del radar
q = 10; % Varianza del ruido de proceso -> DUDA
N = size(target_real.measure, 1); % Número de medidas

%PASO 1:
%Modelo de velocidad constante
%x_k = [x; y; vx; vy];

%Matriz de transicion del estado:
F = [1 0 T 0;
     0 1 0 T;
     0 0 1 0;
     0 0 0 1];

%DUDA: AQUI PONEMOS Q? - incertidumbre de la aceleracion
%no tengo la desviacion tipica de la aceleracion asiq supongo que no
Q = q * [T^4/4 0 T^3/2 0;
         0 T^4/4 0 T^3/2;
         T^3/2 0 T^2 0;
         0 T^3/2 0 T^2];

%PASO 2: modelo de observación

% medidas-> en coordenadas esterograficas!!
%z_k = [x; y] + ruido;

%matriz observacion H:
H = [1 0 0 0;
     0 1 0 0];
%matriz covarianza R
% creada en real_measurement
%tendre que llamarla pero ahora voy a hacer primero lo basico: que no se me
%olvide

% PASO 3: bucle del filtro de Kalman:
% Matriz de covarianza de medida (ejemplo simple)
R = eye(2) * 100;

% Estado inicial (asumimos que empieza en reposo en la primera medida)
x_hat = [target_real.measure(1,13); target_real.measure(1,14); 0; 0]; % [x y vx vy]
P = eye(4) * 500; % Incertidumbre grande

% Almacenar estimaciones
estimates = zeros(N, 4);

% === Bucle del filtro de Kalman ===
for k = 1:N
    % Medida (posición estereográfica medida)
    z = target_real.measure(k, 13:14)';  % [x, y]

    % Predicción
    x_pred = F * x_hat;
    P_pred = F * P * F' + Q;

    % Actualización
    K = P_pred * H' / (H * P_pred * H' + R);
    x_hat = x_pred + K * (z - H * x_pred);
    P = (eye(4) - K * H) * P_pred;

    % Guardar estimación
    estimates(k, :) = x_hat';
end

% === Visualización (opcional) ===
figure;
plot(target_real.measure(:,13)/1e3, target_real.measure(:,14)/1e3, '+m'); hold on;
plot(estimates(:,1)/1e3, estimates(:,2)/1e3, '-b');
legend('Medidas ruidosas', 'Estimación filtro Kalman');
xlabel('X (km)'); ylabel('Y (km)');
title('Seguimiento con filtro de Kalman');
grid on;

end
