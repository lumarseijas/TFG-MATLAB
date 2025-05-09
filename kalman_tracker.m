% function estimates = kalman_tracker(target_real)
function [estimates, speed, rumbo_deg] = kalman_tracker(target_real, track, q)
%PASO 0:
T = mean(diff(target_real.measure(:,2))); % Tiempo de muestreo del radar
N = size(target_real.measure, 1); % Número de medidas

%PASO 1:
%Modelo de velocidad constante
%x_k = [x; y; vx; vy];

%Matriz de transicion del estado:
F = [1 0 T 0;
     0 1 0 T;
     0 0 1 0;
     0 0 0 1];

%DUDA: incertidumbre de la aceleracion
%no tengo la desviacion tipica de la aceleracion
%q = 10; % Varianza del ruido de proceso -> INVENTADA
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

% PASO 3: bucle del filtro de Kalman:01
%matriz covarianza R
% creada en real_measurement
%tendre que llamarla pero ahora voy a hacer primero lo basico: que no se me
%olvide

% Matriz de covarianza de medida 
% R = target_real.mcov(:,:,k);

% Estado inicial (asumimos que empieza en reposo en la primera medida)
% x_hat = [target_real.measure(1,13); target_real.measure(1,14); 0; 0]; % [x y vx vy]
% P = eye(4) * 500; % Incertidumbre grande
% === Inicialización ===
x0 = target_real.measure(1,13);
y0 = target_real.measure(1,14);
x1 = target_real.measure(2,13);
y1 = target_real.measure(2,14);
t0 = target_real.measure(1,2);
t1 = target_real.measure(2,2);
T0 = t1 - t0;

vx0 = (x1 - x0) / T0;
vy0 = (y1 - y0) / T0;

x_hat = [x0; y0; vx0; vy0];
P = eye(4) * 500; % incertidumbre inicial grande


% PASO 4: Almacenamiento
estimates = zeros(N, 4);
speed = zeros(N, 1);
rumbo_deg = zeros(N, 1);

% PASO 5: Filtro de Kalman
for k = 1:N
    % Medida actual
    z = target_real.measure(k,13:14)';  % [x y]
    
    % Covarianza real de medida
    R = target_real.mcov(:,:,k);
    
    % Predicción 
    x_pred = F * x_hat;
    P_pred = F * P * F' + Q;
    
    % Actualización 
    K = P_pred * H' / (H * P_pred * H' + R);
    x_hat = x_pred + K * (z - H * x_pred);
    P = (eye(4) - K * H) * P_pred;

    % Guardar estimación
    estimates(k,:) = x_hat';
    
    % Velocidad y rumbo estimado
    vx = x_hat(3);
    vy = x_hat(4);
    speed(k) = sqrt(vx^2 + vy^2);
    rumbo_deg(k) = atan2d(vx, vy);  % ATENCIÓN: vx/vy da rumbo desde Norte
end
% PASO 6: Visualización básica 
% figure;
% plot(target_real.measure(:,13)/1e3, target_real.measure(:,14)/1e3, '+m'); hold on;
% plot(estimates(:,1)/1e3, estimates(:,2)/1e3, '-+b', 'LineWidth', 1.5);
% 
% if nargin > 1  % si paso track como argumento
%     plot(track(1).posStereo(:,1)/1e3, track(1).posStereo(:,2)/1e3, '--g', 'LineWidth', 1.2);
%     legend('Medidas radar', 'Estimación Kalman', 'Trayectoria real');
% else
%     legend('Medidas radar', 'Estimación Kalman');
% end
% 
% xlabel('X (km)');
% ylabel('Y (km)');
% title('Seguimiento de aeronave con filtro de Kalman');
% grid on;
end