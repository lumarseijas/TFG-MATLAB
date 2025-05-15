function [estimates, speed, rumbo_deg, Z_vec, modo_maniobra, cambios] = ...
    kalman_tracker_maniobra_eval(target_real, track, q_nominal, q_maniobra, alfa, gamma)


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
%fa = 0.05; %falsa alarma
%gamma = 5.99;                      % Umbral chi^2 (95% m=2)
%M = 2; %dimensionalidad
%alfa = 1/3;
%gamma = chi2inv(1-fa,(1+alfa)*M/(1-alfa)); %umbral comparacion
contador_no_maniobra = 0;
N_maniobra_persistente = 5;        % nº escaneos para volver a modo normal

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

%inicializacion del filtro de residuo
Z = 0;

% PASO 4: Almacenamiento
estimates = zeros(N, 4);
speed = zeros(N, 1);
rumbo_deg = zeros(N, 1);
Z_vec = zeros(N, 1);
modo_maniobra = zeros(N, 1);
modo_actual = 0; % 0: modo normal, 1: modo maniobra


% PASO 5: Filtro de Kalman con detección de maniobra
for k = 1:N
    z = target_real.measure(k,13:14)';  % Medida actual [x y]
    R = target_real.mcov(:,:,k);        % Covarianza de medida

    % Predicción
    x_pred = F * x_hat;
    Q = q_current * (Gamma * Gamma');
    P_pred = F * P * F' + Q;
    S = H * P_pred * H' + R;

    % Estadística de innovación 
    innov = z - H * x_pred;

    Z = alfa*Z + innov.' /S * innov ; 
    Z_vec(k) = Z;



    % Detector de maniobra
    if Z > gamma
        q_current = q_maniobra;
        contador_no_maniobra = 0;
        modo_actual = 1;  % ← IMPORTANTE: activa el flag de modo maniobra
    else
        contador_no_maniobra = contador_no_maniobra + 1;
        if contador_no_maniobra >= N_maniobra_persistente
            q_current = q_nominal;
            modo_actual = 0;  % ← IMPORTANTE: vuelve a modo normal
        end
    end

    modo_maniobra(k) = modo_actual;

    Q = q_current * (Gamma * Gamma');  % Q depende del modo actual

    P_pred = F * P * F' + Q;
    
    S = H * P_pred * H' + R;

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

    

    % Nada más necesario: Z_vec y modo_maniobra ya se devuelven
end
    cambios = sum(abs(diff(modo_maniobra)));

end
