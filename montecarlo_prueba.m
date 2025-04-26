clear; close all; clc;

Nsim = 100;    % Número de simulaciones
T = 4;         % Tiempo entre medidas

% --- Valor de q para el filtro Kalman (provisional, luego lo iremos ajustando) ---
q_value = 1000;   % Puedes ir probando: 10, 50, 100, 200...

% Arrays para guardar errores
longitudinal_error = [];
transversal_error = [];
rumbo_error = [];
velocidad_error = [];

for i = 1:Nsim
    % Mostrar progreso cada 10 simulaciones
    if mod(i,10) == 0
        fprintf('Simulación %d de %d...\n', i, Nsim);
    end

    %% 1. Generar trayectoria nueva
    [track, radar, projection] = generarTrayectoria();

    %% 2. Medidas ideales
    target_ideal = ideal_measurement(track, radar, projection);

    %% 3. Medidas reales con errores
    target_real = real_measurement(target_ideal, radar, 1, 1, 0, 0, 0, projection);

    %% 4. Filtro de Kalman
    [estimates, speed_est, rumbo_est] = kalman_tracker(target_real, track, q_value);

    %% 5. Calcular errores
    % Trayectoria real
    x_real = track(1).posStereo(:,1);
    y_real = track(1).posStereo(:,2);
    vx_real = gradient(x_real, T);
    vy_real = gradient(y_real, T);
    rumbo_real = atan2(vy_real, vx_real); % radianes

    % Estimaciones
    x_est = estimates(:,1);
    y_est = estimates(:,2);
    vx_est = estimates(:,3);
    vy_est = estimates(:,4);

    % Número de muestras estimadas
    Nmed = size(estimates,1);

    % Ajustamos tamaños
    dx = x_est - x_real(1:Nmed);
    dy = y_est - y_real(1:Nmed);

    % Proyección de errores longitudinal y transversal
    longitudinal = dx .* cos(rumbo_real(1:Nmed)) + dy .* sin(rumbo_real(1:Nmed));
    transversal = -dx .* sin(rumbo_real(1:Nmed)) + dy .* cos(rumbo_real(1:Nmed));

    % Error de rumbo
    rumbo_est_calc = atan2(vy_est, vx_est); % radianes
    error_rumbo = wrapToPi(rumbo_est_calc - rumbo_real(1:Nmed));

    % Error de velocidad
    vel_real = sqrt(vx_real.^2 + vy_real.^2);
    vel_est = sqrt(vx_est.^2 + vy_est.^2);
    vel_real_cortado = vel_real(1:Nmed);
    error_velocidad = vel_est - vel_real_cortado;

    %% 6. Guardar errores acumulados
    longitudinal_error = [longitudinal_error; longitudinal];
    transversal_error = [transversal_error; transversal];
    rumbo_error = [rumbo_error; error_rumbo];
    velocidad_error = [velocidad_error; error_velocidad];

end

%% 7. Resultados estadísticos
fprintf('\n--- Resultados Monte Carlo ---\n');
fprintf('Error longitudinal RMS: %.2f m\n', sqrt(mean(longitudinal_error.^2)));
fprintf('Error transversal RMS: %.2f m\n', sqrt(mean(transversal_error.^2)));
fprintf('Error rumbo RMS: %.2f grados\n', sqrt(mean((180/pi*rumbo_error).^2)));
fprintf('Error velocidad RMS: %.2f m/s\n', sqrt(mean(velocidad_error.^2)));

%% 8. Mostrar histogramas
figure;
subplot(2,2,1);
histogram(longitudinal_error, 50);
title('Error Longitudinal');
xlabel('Error [m]');
ylabel('Frecuencia');

subplot(2,2,2);
histogram(transversal_error, 50);
title('Error Transversal');
xlabel('Error [m]');
ylabel('Frecuencia');

subplot(2,2,3);
histogram(rumbo_error*180/pi, 50);
title('Error de Rumbo');
xlabel('Error [°]');
ylabel('Frecuencia');

subplot(2,2,4);
histogram(velocidad_error, 50);
title('Error de Velocidad');
xlabel('Error [m/s]');
ylabel('Frecuencia');

sgtitle('Distribución de errores - Simulación Monte Carlo');
