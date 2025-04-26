clear; close all; clc;

Nsim = 100;    % Número de simulaciones
q_value = 100;   % Valor provisional de q para el filtro Kalman

% Arrays para guardar errores
longitudinal_error = [];
transversal_error = [];
rumbo_error = [];
velocidad_error = [];

for i = 1:Nsim
    % Mostrar progreso
    if mod(i,10) == 0
        fprintf('Simulación %d de %d...\n', i, Nsim);
    end

    %% 1. Generar trayectoria
    [track, radar, projection] = generarTrayectoria();
    Tmedida = radar(1).Tr;  % Tiempo de muestreo del radar (4 segundos)

    %% 2. Medidas ideales
    target_ideal = ideal_measurement(track, radar, projection);

    %% 3. Medidas reales
    target_real = real_measurement(target_ideal, radar, 1, 1, 0, 0, 0, projection);

    %% 4. Filtro de Kalman
    [estimates, speed_est, rumbo_est] = kalman_tracker(target_real, track, q_value);

    %% 5. Ajustar la trayectoria real interpolada
    Nmed = size(estimates,1);  % Número de medidas
    tiempos_medidos = (0:Nmed-1)' * Tmedida;  % instantes donde se mide

    x_real_interp = interp1(track(1).tiempo, track(1).posStereo(:,1), tiempos_medidos);
    y_real_interp = interp1(track(1).tiempo, track(1).posStereo(:,2), tiempos_medidos);

    vx_real = gradient(x_real_interp, Tmedida);
    vy_real = gradient(y_real_interp, Tmedida);
    vel_real = sqrt(vx_real.^2 + vy_real.^2);
    rumbo_real = atan2(vy_real, vx_real);

    %% 6. Comparar posiciones estimadas vs reales
    x_est = estimates(:,1);
    y_est = estimates(:,2);
    vx_est = estimates(:,3);
    vy_est = estimates(:,4);

    dx = x_est - x_real_interp;
    dy = y_est - y_real_interp;

    % Proyección longitudinal y transversal
    longitudinal = dx .* cos(rumbo_real) + dy .* sin(rumbo_real);
    transversal  = -dx .* sin(rumbo_real) + dy .* cos(rumbo_real);

    % Error de rumbo
    rumbo_est_calc = atan2(vy_est, vx_est);
    error_rumbo = wrapToPi(rumbo_est_calc - rumbo_real);

    % Error de velocidad
    vel_est = sqrt(vx_est.^2 + vy_est.^2);
    error_velocidad = vel_est - vel_real;

    %% 7. Guardar errores
    longitudinal_error = [longitudinal_error; longitudinal];
    transversal_error = [transversal_error; transversal];
    rumbo_error = [rumbo_error; error_rumbo];
    velocidad_error = [velocidad_error; error_velocidad];

end

%% 8. Resultados finales
fprintf('\n--- Resultados Monte Carlo ---\n');
fprintf('Error longitudinal RMS: %.2f m\n', sqrt(mean(longitudinal_error.^2)));
fprintf('Error transversal RMS: %.2f m\n', sqrt(mean(transversal_error.^2)));
fprintf('Error rumbo RMS: %.2f grados\n', sqrt(mean((180/pi*rumbo_error).^2)));
fprintf('Error velocidad RMS: %.2f m/s\n', sqrt(mean(velocidad_error.^2)));

%% 9. Mostrar histogramas
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
