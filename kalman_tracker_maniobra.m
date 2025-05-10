function [estimates, speed, rumbo_deg] = kalman_tracker_maniobras(target_real, track, q_value)
% Filtro de Kalman adaptativo con detección de maniobra
% Usa q_value como valor base para la matriz Q, y lo adapta si detecta maniobras

T = mean(diff(target_real.measure(:,2)));
N = size(target_real.measure, 1);

F = [1 0 T 0;
     0 1 0 T;
     0 0 1 0;
     0 0 0 1];

H = [1 0 0 0;
     0 1 0 0];

% Inicialización del estado
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
P = eye(4) * 500;

estimates = zeros(N, 4);
speed = zeros(N, 1);
rumbo_deg = zeros(N, 1);
maneuvering_flag = false(N,1);

for k = 1:N
    z = target_real.measure(k,13:14)';
    R = target_real.mcov(:,:,k);

    % === Cálculo de Q dinámico ===
    q = q_value;
    Q = q * [T^4/4 0 T^3/2 0;
             0 T^4/4 0 T^3/2;
             T^3/2 0 T^2 0;
             0 T^3/2 0 T^2];

    % Predicción
    x_pred = F * x_hat;
    P_pred = F * P * F' + Q;

    % Evaluar innovación
    nu = z - H * x_pred;
    S = H * P_pred * H' + R;
    D = nu' / S * nu;

    if D > 9  % Umbral de maniobra
        q = 10 * q_value;
        maneuvering_flag(k) = true;
    end

    % Recalcular Q con el nuevo valor si hay maniobra
    Q = q * [T^4/4 0 T^3/2 0;
             0 T^4/4 0 T^3/2;
             T^3/2 0 T^2 0;
             0 T^3/2 0 T^2];

    % Nueva predicción (opcional: ya calculada antes)
    x_pred = F * x_hat;
    P_pred = F * P * F' + Q;

    % Actualización
    K = P_pred * H' / (H * P_pred * H' + R);
    x_hat = x_pred + K * (z - H * x_pred);
    P = (eye(4) - K * H) * P_pred;

    estimates(k,:) = x_hat';
    vx = x_hat(3);
    vy = x_hat(4);
    speed(k) = sqrt(vx^2 + vy^2);
    rumbo_deg(k) = atan2d(vx, vy);
end

% fprintf('Maniobras detectadas en %d instantes.\n', sum(maneuvering_flag));
end
