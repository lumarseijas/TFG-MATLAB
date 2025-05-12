q_value = 2.5;
q_maniobra = 50;
zeta_r = 0.1;  % error relativo permitido
Nsim = Nsim_requerido(zeta_r);
fprintf('Usando zeta_r = %.2f → Nsim = %d simulaciones\n', zeta_r, Nsim);

[track, radar, projection] = generarTrayectoriaAcel();
Tmedida = radar(1).Tr;
Nscans = floor(track(1).tiempo(end) / Tmedida);

tramos = track(1).tramos;
track(1).tramos_tiempos = cumsum([0; tramos(:,4)]);
Ntramos = size(tramos, 1);

err_cv_total = struct('long', zeros(Nscans,1), 'trans', zeros(Nscans,1), ...
                      'rumbo', zeros(Nscans,1), 'vel', zeros(Nscans,1));
err_adapt_total = err_cv_total;

for i = 1:Nsim
    if mod(i, 50) == 0
        fprintf('Simulación %d de %d...\n', i, Nsim);
    end

    radar(1).Tini = rand() * radar(1).Tr; % tiempo inicial aleatorio
    target_ideal = ideal_measurement(track, radar, projection);
    target_real  = real_measurement(target_ideal, radar, 1, 1, 0, 0, 0, projection);

    [est_cv, ~, ~]  = kalman_tracker(target_real, track, q_value);
    [est_ad, ~, ~]  = kalman_tracker_maniobra(target_real, track, q_value, q_maniobra);

    Nmed = size(est_cv, 1);
    tiempos = (0:Nmed-1)' * Tmedida;

    % Asegurar que los tiempos están dentro del dominio
    tmin = track(1).tiempo(1);
    tmax = track(1).tiempo(end);
    tiempos = max(tmin, min(tiempos, tmax));

    x_real = interp1(track(1).tiempo, track(1).posStereo(:,1), tiempos);
    y_real = interp1(track(1).tiempo, track(1).posStereo(:,2), tiempos);
    % vx_real = gradient(x_real, Tmedida);
    % vy_real = gradient(y_real, Tmedida);
    % vel_real = sqrt(vx_real.^2 + vy_real.^2);
    % rumbo_real = atan2(vy_real, vx_real);
    vel_real = interp1(track(1).tiempo, track(1).velocidad, tiempos);
    rumbo_real = interp1(track(1).tiempo, track(1).rumbo, tiempos);

    % Kalman CV
    dx_cv = est_cv(:,1) - x_real;
    dy_cv = est_cv(:,2) - y_real;
    vx_cv = est_cv(:,3); vy_cv = est_cv(:,4);
    vel_cv = sqrt(vx_cv.^2 + vy_cv.^2);
    rumbo_cv = atan2(vy_cv, vx_cv);
    eLcv = (dx_cv .* cos(rumbo_real) + dy_cv .* sin(rumbo_real)).^2;
    eTcv = (-dx_cv .* sin(rumbo_real) + dy_cv .* cos(rumbo_real)).^2;
    eRcv = wrapToPi(rumbo_cv - rumbo_real).^2;
    eVcv = (vel_cv - vel_real).^2;

    % Kalman con maniobras
    dx_ad = est_ad(:,1) - x_real;
    dy_ad = est_ad(:,2) - y_real;
    vx_ad = est_ad(:,3); vy_ad = est_ad(:,4);
    vel_ad = sqrt(vx_ad.^2 + vy_ad.^2);
    rumbo_ad = atan2(vy_ad, vx_ad);
    eLad = (dx_ad .* cos(rumbo_real) + dy_ad .* sin(rumbo_real)).^2;
    eTad = (-dx_ad .* sin(rumbo_real) + dy_ad .* cos(rumbo_real)).^2;
    eRad = wrapToPi(rumbo_ad - rumbo_real).^2;
    eVad = (vel_ad - vel_real).^2;

    Nuse = min(Nscans, Nmed);
    err_cv_total.long(1:Nuse) = err_cv_total.long(1:Nuse) + eLcv(1:Nuse)/Nsim;
    err_cv_total.trans(1:Nuse)= err_cv_total.trans(1:Nuse)+ eTcv(1:Nuse)/Nsim;
    err_cv_total.rumbo(1:Nuse)= err_cv_total.rumbo(1:Nuse)+ eRcv(1:Nuse)/Nsim;
    err_cv_total.vel(1:Nuse)  = err_cv_total.vel(1:Nuse)  + eVcv(1:Nuse)/Nsim;

    err_adapt_total.long(1:Nuse) = err_adapt_total.long(1:Nuse) + eLad(1:Nuse)/Nsim;
    err_adapt_total.trans(1:Nuse)= err_adapt_total.trans(1:Nuse)+ eTad(1:Nuse)/Nsim;
    err_adapt_total.rumbo(1:Nuse)= err_adapt_total.rumbo(1:Nuse)+ eRad(1:Nuse)/Nsim;
    err_adapt_total.vel(1:Nuse)  = err_adapt_total.vel(1:Nuse)  + eVad(1:Nuse)/Nsim;
end

% === Clasificación por tramos ===
err_cv = struct('long', zeros(Ntramos,1), 'trans', zeros(Ntramos,1), ...
                'rumbo', zeros(Ntramos,1), 'vel', zeros(Ntramos,1));
err_adapt = err_cv;

tiempos_sim = (0:Nscans-1)' * Tmedida;

for k = 1:Nscans
    t = tiempos_sim(k);
    idx = find(t >= track(1).tramos_tiempos(1:end-1) & t < track(1).tramos_tiempos(2:end), 1);
    if isempty(idx), continue; end

    err_cv.long(idx)  = err_cv.long(idx)  + err_cv_total.long(k)/Nsim;
    err_cv.trans(idx) = err_cv.trans(idx) + err_cv_total.trans(k)/Nsim;
    err_cv.rumbo(idx) = err_cv.rumbo(idx) + err_cv_total.rumbo(k)/Nsim;
    err_cv.vel(idx)   = err_cv.vel(idx)   + err_cv_total.vel(k)/Nsim;

    err_adapt.long(idx)  = err_adapt.long(idx)  + err_adapt_total.long(k)/Nsim;
    err_adapt.trans(idx) = err_adapt.trans(idx) + err_adapt_total.trans(k)/Nsim;
    err_adapt.rumbo(idx) = err_adapt.rumbo(idx) + err_adapt_total.rumbo(k)/Nsim;
    err_adapt.vel(idx)   = err_adapt.vel(idx)   + err_adapt_total.vel(k)/Nsim;
end

% === Impresión de errores por tramo ===
fprintf('\n--- ERRORES RMS POR TRAMO ---\n');

for i = 1:Ntramos
    tipo = "Rectilíneo";
    if tramos(i,2) ~= 0
        tipo = "Giro";
    elseif tramos(i,1) ~= 0
        tipo = "Acelerado";
    end

    fprintf('\nTramo %d (%s) - Duración: %.1f s\n', i, tipo, tramos(i,4));

    fprintf('  Kalman CV:\n');
    fprintf('    Longitudinal RMS : %.2f m\n', sqrt(err_cv.long(i)));
    fprintf('    Transversal  RMS : %.2f m\n', sqrt(err_cv.trans(i)));
    fprintf('    Rumbo       RMS  : %.2f º\n', sqrt(err_cv.rumbo(i)) * 180/pi);
    fprintf('    Velocidad   RMS  : %.2f m/s\n', sqrt(err_cv.vel(i)));

    fprintf('  Kalman con Maniobras:\n');
    fprintf('    Longitudinal RMS : %.2f m\n', sqrt(err_adapt.long(i)));
    fprintf('    Transversal  RMS : %.2f m\n', sqrt(err_adapt.trans(i)));
    fprintf('    Rumbo       RMS  : %.2f º\n', sqrt(err_adapt.rumbo(i)) * 180/pi);
    fprintf('    Velocidad   RMS  : %.2f m/s\n', sqrt(err_adapt.vel(i)));

    fprintf('  Mejora relativa:\n');
    fprintf('    Longitudinal : %.1f %%\n', 100 * (1 - sqrt(err_adapt.long(i)) / sqrt(err_cv.long(i))) );
    fprintf('    Transversal  : %.1f %%\n', 100 * (1 - sqrt(err_adapt.trans(i)) / sqrt(err_cv.trans(i))) );
    fprintf('    Rumbo        : %.1f %%\n', 100 * (1 - sqrt(err_adapt.rumbo(i)) / sqrt(err_cv.rumbo(i))) );
    fprintf('    Velocidad    : %.1f %%\n', 100 * (1 - sqrt(err_adapt.vel(i)) / sqrt(err_cv.vel(i))) );
end
% === Gráficas temporales de errores ===
t_total = (0:Nscans-1)' * Tmedida;
limites = cumsum([0; tramos(:,4)]);

figure('Name', 'Errores RMS en el Tiempo - Kalman CV vs Detección Maniobra ACEL');
titles = {'Longitudinal RMS [m]', 'Transversal RMS [m]', ...
          'Rumbo RMS [º]', 'Velocidad RMS [m/s]'};
fields = {'long', 'trans', 'rumbo', 'vel'};

for i = 1:4
    subplot(2,2,i); hold on;
    y_cv = sqrt(err_cv_total.(fields{i}));
    y_ad = sqrt(err_adapt_total.(fields{i}));
    if strcmp(fields{i}, 'rumbo')
        y_cv = y_cv * 180/pi;
        y_ad = y_ad * 180/pi;
    end

    plot(t_total, y_cv, '--b', 'DisplayName', 'Kalman CV');
    plot(t_total, y_ad, '-r', 'DisplayName', 'Maniobra');

    for j = 1:length(limites)
        if j == 2
            xline(limites(j), ':k', 'LineWidth', 1, 'DisplayName', 'Cambio de tramo');
        else
            xline(limites(j), ':k', 'LineWidth', 1, 'HandleVisibility', 'off');
        end
    end

    title(titles{i});
    xlabel('Tiempo [s]');
    grid on;
    legend show;
end
