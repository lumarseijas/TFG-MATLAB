

Nsim = 100;    % Número de simulaciones
q_value = 0.1; % Valor provisional de q para el filtro Kalman

 % 1. Generar trayectoria
Tmedida = radar(1).Tr;  % Tiempo entre medidas de radar
[track, radar, projection] = generarTrayectoria();

% Nº de scans:
Nscans=track.tiempo(end)/radar(1).Tr;

longitudinal_error = zeros(Nscans,1);
transversal_error = zeros(Nscans,1);
rumbo_error = zeros(Nscans,1);
velocidad_error = zeros(Nscans,1);

for i = 1:Nsim
    if mod(i,10) == 0
        fprintf('Simulación %d de %d...\n', i, Nsim);
    end

   

    % 2. Medidas ideales y reales
    radar(1).Tini = rand(1)*radar(1).Tr; % Tiempo inicial aleator
    target_ideal = ideal_measurement(track, radar, projection);
    target_real = real_measurement(target_ideal, radar, 1, 1, 0, 0, 0, projection);

    % 3. Filtro de Kalman
    [estimates, speed_est, rumbo_est] = kalman_tracker(target_real, track, q_value);

    % 4. Interpolación de trayectoria real para comparar con medidas
    Nmed = size(estimates,1);
    tiempos_medidos = (0:Nmed-1)' * Tmedida;

    x_real_interp = interp1(track(1).tiempo, track(1).posStereo(:,1), tiempos_medidos);
    y_real_interp = interp1(track(1).tiempo, track(1).posStereo(:,2), tiempos_medidos);

    vx_real = gradient(x_real_interp, Tmedida);
    vy_real = gradient(y_real_interp, Tmedida);
    vel_real = sqrt(vx_real.^2 + vy_real.^2);
    rumbo_real = atan2(vy_real, vx_real);

    % 5. Comparar estimaciones con reales
    x_est = estimates(:,1);
    y_est = estimates(:,2);
    vx_est = estimates(:,3);
    vy_est = estimates(:,4);

    dx = x_est - x_real_interp;
    dy = y_est - y_real_interp;

    % Proyección de errores
    longitudinal = dx .* cos(rumbo_real) + dy .* sin(rumbo_real);
    transversal  = -dx .* sin(rumbo_real) + dy .* cos(rumbo_real);
    rumbo_est_calc = atan2(vy_est, vx_est);
    error_rumbo = wrapToPi(rumbo_est_calc - rumbo_real);
    vel_est = sqrt(vx_est.^2 + vy_est.^2);
    error_velocidad = vel_est - vel_real;

    % 6. Guardar errores
    % longitudinal_error = [longitudinal_error; longitudinal];
    longitudinal_error = longitudinal_error + longitudinal(1:Nscans,1).^2/Nsim;
    % transversal_error = [transversal_error; transversal];
    transversal_error = transversal_error + transversal(1:Nscans,1).^2/Nsim;
    % rumbo_error = [rumbo_error; error_rumbo];
    rumbo_error = rumbo_error + error_rumbo(1:Nscans,1).^2/Nsim;
    % velocidad_error = [velocidad_error; error_velocidad];
    velocidad_error = velocidad_error + error_velocidad(1:Nscans,1).^2/Nsim;
end

% 7. Crear vector temporal
dt = radar(1).Tr;
Ntotal = length(longitudinal_error);
tiempo_total = (0:Ntotal-1)' * dt;

% 8. Resultados finales
fprintf('\n--- Resultados Monte Carlo ---\n');
fprintf('Error longitudinal RMS: %.2f m\n', sqrt(mean(longitudinal_error.^2)));
fprintf('Error transversal RMS: %.2f m\n', sqrt(mean(transversal_error.^2)));
fprintf('Error rumbo RMS: %.2f grados\n', sqrt(mean((180/pi*rumbo_error).^2)));
fprintf('Error velocidad RMS: %.2f m/s\n', sqrt(mean(velocidad_error.^2)));

% 9. 
longitudinal_s = sqrt(longitudinal_error);
transversal_s  = sqrt(transversal_error);
rumbo_s        = sqrt(rumbo_error);
velocidad_s    = sqrt(velocidad_error);

% 10. Gráficas de evolución temporal
figure('Position', [100, 100, 800, 600]);  % tamaño personalizado
tiledlayout(2,2);

nexttile;
plot(tiempo_total, longitudinal_s, 'b');
title('Error Longitudinal vs Tiempo', 'FontWeight', 'bold');
xlabel('Tiempo [s]');
ylabel('Error [m]');
grid on;

nexttile;
plot(tiempo_total, transversal_s, 'b');
title('Error Transversal vs Tiempo', 'FontWeight', 'bold');
xlabel('Tiempo [s]');
ylabel('Error [m]');
grid on;

nexttile;
plot(tiempo_total, rumbo_s * 180/pi, 'b');  % en grados
title('Error de Rumbo vs Tiempo', 'FontWeight', 'bold');
xlabel('Tiempo [s]');
ylabel('Error [°]');
grid on;

nexttile;
plot(tiempo_total, velocidad_s, 'b');
title('Error de Velocidad vs Tiempo', 'FontWeight', 'bold');
xlabel('Tiempo [s]');
ylabel('Error [m/s]');
grid on;

sgtitle('Evolución de errores - Simulación Monte Carlo', 'FontSize', 14, 'FontWeight', 'bold');
