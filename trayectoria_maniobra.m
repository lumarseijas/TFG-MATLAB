clear; close all; clc;

% --- 1. Generar una única trayectoria con maniobra ---
[track, radar, projection] = generarTrayectoria();
Tmedida = radar(1).Tr;  % Tiempo entre medidas de radar (4 segundos)

% --- 2. Visualizar la trayectoria real (posición XY) ---
figure;
plot(track(1).posStereo(:,1)/1e3, track(1).posStereo(:,2)/1e3, 'b-');
xlabel('X [km]');
ylabel('Y [km]');
title('Trayectoria real de la aeronave');
grid on;
axis equal;

% --- 3. Visualizar velocidad y rumbo reales ---
Tmuestreo = track(1).tiempo(2) - track(1).tiempo(1);  % tiempo de muestreo fino
vx_real_full = gradient(track(1).posStereo(:,1), Tmuestreo);
vy_real_full = gradient(track(1).posStereo(:,2), Tmuestreo);

velocidad_full = sqrt(vx_real_full.^2 + vy_real_full.^2);
rumbo_full = atan2(vy_real_full, vx_real_full) * 180/pi;  % en grados

figure;
subplot(2,1,1);
plot(track(1).tiempo, velocidad_full);
xlabel('Tiempo [s]');
ylabel('Velocidad [m/s]');
title('Velocidad real vs Tiempo');
grid on;

subplot(2,1,2);
plot(track(1).tiempo, rumbo_full);
xlabel('Tiempo [s]');
ylabel('Rumbo [°]');
title('Rumbo real vs Tiempo');
grid on;

% --- 4. Medidas ideales y reales ---
target_ideal = ideal_measurement(track, radar, projection);
target_real = real_measurement(target_ideal, radar, 1, 1, 0, 0, 0, projection);

% --- 5. Aplicar filtro de Kalman ---
q_value = 0; % Valor provisional de q
[estimates, speed_est, rumbo_est] = kalman_tracker(target_real, track, q_value);

% --- 6. Ajustar la trayectoria real interpolada ---
Nmed = size(estimates,1);
tiempos_medidos = (0:Nmed-1)' * Tmedida;

x_real_interp = interp1(track(1).tiempo, track(1).posStereo(:,1), tiempos_medidos);
y_real_interp = interp1(track(1).tiempo, track(1).posStereo(:,2), tiempos_medidos);

vx_real = gradient(x_real_interp, Tmedida);
vy_real = gradient(y_real_interp, Tmedida);
vel_real = sqrt(vx_real.^2 + vy_real.^2);
rumbo_real = atan2(vy_real, vx_real);

% --- 7. Comparar posiciones estimadas vs reales ---
x_est = estimates(:,1);
y_est = estimates(:,2);
vx_est = estimates(:,3);
vy_est = estimates(:,4);

dx = x_est - x_real_interp;
dy = y_est - y_real_interp;

longitudinal = dx .* cos(rumbo_real) + dy .* sin(rumbo_real);
transversal  = -dx .* sin(rumbo_real) + dy .* cos(rumbo_real);

rumbo_est_calc = atan2(vy_est, vx_est);
error_rumbo = wrapToPi(rumbo_est_calc - rumbo_real);

vel_est = sqrt(vx_est.^2 + vy_est.^2);
error_velocidad = vel_est - vel_real;

% --- 8. Graficar errores vs tiempo ---
figure;
subplot(2,2,1);
plot(tiempos_medidos, longitudinal);
title('Error Longitudinal vs Tiempo');
xlabel('Tiempo [s]');
ylabel('Error [m]');
grid on;

subplot(2,2,2);
plot(tiempos_medidos, transversal);
title('Error Transversal vs Tiempo');
xlabel('Tiempo [s]');
ylabel('Error [m]');
grid on;

subplot(2,2,3);
plot(tiempos_medidos, error_rumbo * 180/pi);  % en grados
title('Error de Rumbo vs Tiempo');
xlabel('Tiempo [s]');
ylabel('Error [°]');
grid on;

subplot(2,2,4);
plot(tiempos_medidos, error_velocidad);
title('Error de Velocidad vs Tiempo');
xlabel('Tiempo [s]');
ylabel('Error [m/s]');
grid on;

sgtitle('Evolución de errores - Trayectoria única con maniobra');
