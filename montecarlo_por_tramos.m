Nsim = 100;
q_value = 1;

[track, radar, projection] = generarTrayectoria();
Tmedida = radar(1).Tr;
Nscans = track.tiempo(end)/Tmedida;

% --- Prepara los tramos ---
tramos = track(1).tramos;
limites_tiempo = cumsum([0; tramos(:,4)]);
Ntramos = size(tramos,1);

% Inicializa acumuladores
err_tramos = struct('long', zeros(Ntramos,1), ...
                    'trans', zeros(Ntramos,1), ...
                    'rumbo', zeros(Ntramos,1), ...
                    'vel', zeros(Ntramos,1));

for i = 1:Nsim
    radar(1).Tini = rand()*radar(1).Tr;
    target_ideal = ideal_measurement(track, radar, projection);
    target_real = real_measurement(target_ideal, radar, 1, 1, 0, 0, 0, projection);

    [estimates, ~, ~] = kalman_tracker_maniobra(target_real, track, q_value);

    Nmed = size(estimates,1);
    tiempos = (0:Nmed-1)' * Tmedida;

    x_real = interp1(track(1).tiempo, track(1).posStereo(:,1), tiempos);
    y_real = interp1(track(1).tiempo, track(1).posStereo(:,2), tiempos);
    vx_real = gradient(x_real, Tmedida);
    vy_real = gradient(y_real, Tmedida);
    vel_real = sqrt(vx_real.^2 + vy_real.^2);
    rumbo_real = atan2(vy_real, vx_real);

    dx = estimates(:,1) - x_real;
    dy = estimates(:,2) - y_real;
    vx_est = estimates(:,3);
    vy_est = estimates(:,4);
    vel_est = sqrt(vx_est.^2 + vy_est.^2);
    rumbo_est = atan2(vy_est, vx_est);

    eL = (dx .* cos(rumbo_real) + dy .* sin(rumbo_real)).^2;
    eT = (-dx .* sin(rumbo_real) + dy .* cos(rumbo_real)).^2;
    eR = wrapToPi(rumbo_est - rumbo_real).^2;
    eV = (vel_est - vel_real).^2;

    for k = 1:Nmed
        t = tiempos(k);
        idx = find(t >= limites_tiempo(1:end-1) & t < limites_tiempo(2:end), 1);
        if isempty(idx), continue; end
        err_tramos.long(idx) = err_tramos.long(idx) + eL(k)/Nsim;
        err_tramos.trans(idx)= err_tramos.trans(idx)+ eT(k)/Nsim;
        err_tramos.rumbo(idx)= err_tramos.rumbo(idx)+ eR(k)/Nsim;
        err_tramos.vel(idx)  = err_tramos.vel(idx)  + eV(k)/Nsim;
    end
end

% --- Mostrar resultados por tramo ---
fprintf('\n--- ERRORES POR TRAMO ---\n');
for i = 1:Ntramos
    tipo = "Rectilíneo";
    if tramos(i,2) ~= 0
        tipo = "Giro";
    elseif tramos(i,1) ~= 0
        tipo = "Acelerado";
    end
fprintf('\n---------- TRAMO %d ----------\n', i);
fprintf('Tipo de tramo       : %s\n', tipo);
fprintf('Duración            : %.1f s\n', tramos(i,4));
fprintf('Longitudinal RMS    : %.2f m\n', sqrt(err_tramos.long(i)));
fprintf('Transversal RMS     : %.2f m\n', sqrt(err_tramos.trans(i)));
fprintf('Rumbo RMS           : %.2f º\n', sqrt(err_tramos.rumbo(i)) * 180/pi);
fprintf('Velocidad RMS       : %.2f m/s\n', sqrt(err_tramos.vel(i)));

end
