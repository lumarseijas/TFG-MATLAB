function [estimates, speed, rumbo_deg] = kalman_tracker_maniobra(target_real, track, q_nominal, q_maniobra)
% Filtro de Kalman con detección de maniobra (modo adaptativo)
% target_real: estructura con medidas del radar
% track: estructura con la trayectoria real (para visualización)
% q_nominal: varianza del ruido de aceleración en modo normal
% q_maniobra: varianza en modo maniobra

% PASO 0: Parámetros temporales
T = mean(diff(target_real.measure(:,2))); % Tiempo entre medidas
N = size(target_real.measure, 1);         % Número de medidas

% PASO 1: Modelo de estado y observación
F = [1 0 T 0;
     0 1 0 T;
     0 0 1 0;
     0 0 0 1];               % Modelo velocidad constante

H = [1 0 0 0;
     0 1 0 0];               % Observamos solo posición [x y]

Gamma = [T^2/2 0;
         0 T^2/2;
         T 0;
         0 T];               % Matriz de entrada del ruido

% PASO 2: Matrices de ruido (Q adaptativa)
Q_nom = q_nominal * (Gamma * Gamma');
Q_mani = q_maniobra * (Gamma * Gamma');
q_current = q_nominal;

% Detector de maniobra
gamma = 5.99;                      % Umbral chi^2 (95% m=2)
contador_no_maniobra = 0;
N_maniobra_persistente = 3;        % nº escaneos para volver a modo normal

% PASO 3: Inicialización del estado
x0 = target_real.measure(1,13);
y0 = target_real.measure(1,14);
x1 = target_real.measure(2,13);
y1 = target_real.measure(2,14);
t0 = target_real.measure(1,2);
t1 = target_real.measure(2,2);
T0 = t1 - t0;

vx0 = (x1 - x0) / T0;
vy0 = (y1 - y0) / T0;

x_hat = [x0; y0; vx0; vy0];       % Estado inicial [x y vx vy]
P = eye(4) * 500;                 % Incertidumbre inicial alta

% PASO 4: Almacenamiento
estimates = zeros(N, 4);
speed = zeros(N, 1);
rumbo_deg = zeros(N, 1);

% PASO 5: Filtro de Kalman con detección de maniobra
for k = 1:N
    z = target_real.measure(k,13:14)';  % Medida actual [x y]
    R = target_real.mcov(:,:,k);        % Covarianza de medida

    % Predicción
    Q = q_current * (Gamma * Gamma');  % Q depende del modo actual
    x_pred = F * x_hat;
    P_pred = F * P * F' + Q;

    % Estadística de innovación (Mahalanobis)
    innov = z - H * x_pred;
    S = H * P_pred * H' + R;
    lambda = innov' / S * innov;

    % Detector de maniobra
    if lambda > gamma
        q_current = q_maniobra;
        contador_no_maniobra = 0;
    else
        contador_no_maniobra = contador_no_maniobra + 1;
        if contador_no_maniobra >= N_maniobra_persistente
            q_current = q_nominal;
        end
    end

    % Actualización
    K = P_pred * H' / (H * P_pred * H' + R);
    x_hat = x_pred + K * (z - H * x_pred);
    P = (eye(4) - K * H) * P_pred;

    % Guardar estimación
    estimates(k,:) = x_hat';
    vx = x_hat(3);
    vy = x_hat(4);
    speed(k) = sqrt(vx^2 + vy^2);
    rumbo_deg(k) = atan2d(vx, vy); % rumbo desde el norte
end

% PASO 6: Visualización básica (opcional)

% figure;
% plot(target_real.measure(:,13)/1e3, target_real.measure(:,14)/1e3, '+m'); hold on;
% plot(estimates(:,1)/1e3, estimates(:,2)/1e3, '-+b', 'LineWidth', 1.5);
% 
% if nargin > 1
%     plot(track(1).posStereo(:,1)/1e3, track(1).posStereo(:,2)/1e3, '--g', 'LineWidth', 1.2);
%     legend('Medidas radar', 'Est. Kalman con maniobra', 'Trayectoria real');
% else
%     legend('Medidas radar', 'Est. Kalman con maniobra');
% end
% 
% xlabel('X (km)');
% ylabel('Y (km)');
% title('Filtro de Kalman con detección de maniobra');
% grid on;

end
