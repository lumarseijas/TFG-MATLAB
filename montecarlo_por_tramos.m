addpath(genpath(pwd));  % Añade todas las subcarpetas del proyecto al path

% === PARÁMETROS ===
q_value = 0.1; %IR PROBANDO
q_maniobra = 5; %IR PROBANDO
zeta_r = 0.1;
Nsim = Nsim_requerido(zeta_r);

fprintf('Usando zeta_r = %.2f → Nsim = %d simulaciones\n', zeta_r, Nsim);

[track, radar, projection] = generarTrayectoria();
Tmedida = radar(1).Tr;
Nscans = floor(track(1).tiempo(end) / Tmedida);

tramos = track(1).tramos;
Ntramos = size(tramos, 1);
track(1).tramos_tiempos = cumsum([0; tramos(:,4)]);

tipos_tramo = strings(Ntramos,1);
for i = 1:Ntramos
    if tramos(i,2) ~= 0
        tipos_tramo(i) = "Giro";
    elseif tramos(i,1) ~= 0
        tipos_tramo(i) = "Acelerado";
    else
        tipos_tramo(i) = "Rectilíneo";
    end
end

% === Inicializar acumuladores ===
err_cv_total = struct('long', zeros(Nscans,1), 'trans', zeros(Nscans,1), ...
                      'rumbo', zeros(Nscans,1), 'vel', zeros(Nscans,1));
err_adapt_total = err_cv_total;

for i = 1:Nsim
    if mod(i, 50) == 0
        fprintf('Simulación %d de %d...\n', i, Nsim);
    end

    radar(1).Tini = rand() * radar(1).Tr;
    target_ideal = ideal_measurement(track, radar, projection);
    target_real  = real_measurement(target_ideal, radar, 1, 1, 0, 0, 0, projection);

    [est_cv, ~, ~]  = kalman_tracker(target_real, track, q_value);
    [est_ad, ~, ~]  = kalman_tracker_maniobra(target_real, track, q_value, q_maniobra);

    Nmed = size(est_cv, 1);
    tiempos = (0:Nmed-1)' * Tmedida;
    tiempos = max(track(1).tiempo(1), min(tiempos, track(1).tiempo(end)));

    x_real = interp1(track(1).tiempo, track(1).posStereo(:,1), tiempos);
    y_real = interp1(track(1).tiempo, track(1).posStereo(:,2), tiempos);
    vel_real = interp1(track(1).tiempo, track(1).velocidad, tiempos);
    rumbo_rad = interp1(track(1).tiempo, unwrap(deg2rad(track(1).rumbo)), tiempos);

    % === Errores Kalman CV ===
    dx = est_cv(:,1) - x_real;
    dy = est_cv(:,2) - y_real;
    vx = est_cv(:,3); vy = est_cv(:,4);
    vel = sqrt(vx.^2 + vy.^2);
    rumbo = atan2(vy, vx);

    eLcv = (dx .* cos(rumbo_rad) + dy .* sin(rumbo_rad)).^2;
    eTcv = (-dx .* sin(rumbo_rad) + dy .* cos(rumbo_rad)).^2;
    eRcv = angdiff(rumbo - rumbo_rad).^2;
    eVcv = (vel - vel_real).^2;

    % === Errores Kalman adaptativo ===
    dx = est_ad(:,1) - x_real;
    dy = est_ad(:,2) - y_real;
    vx = est_ad(:,3); vy = est_ad(:,4);
    vel = sqrt(vx.^2 + vy.^2);
    rumbo = atan2(vy, vx);

    eLad = (dx .* cos(rumbo_rad) + dy .* sin(rumbo_rad)).^2;
    eTad = (-dx .* sin(rumbo_rad) + dy .* cos(rumbo_rad)).^2;
    eRad = angdiff(rumbo - rumbo_rad).^2;
    eVad = (vel - vel_real).^2;

    % Truncar por seguridad
    Nuse = min([Nscans, length(eLcv), length(eTcv), length(eRcv), length(eVcv), ...
                          length(eLad), length(eTad), length(eRad), length(eVad)]);
    err_cv_total.long(1:Nuse)  = err_cv_total.long(1:Nuse)  + eLcv(1:Nuse)/Nsim;
    err_cv_total.trans(1:Nuse) = err_cv_total.trans(1:Nuse) + eTcv(1:Nuse)/Nsim;
    err_cv_total.rumbo(1:Nuse) = err_cv_total.rumbo(1:Nuse) + eRcv(1:Nuse)/Nsim;
    err_cv_total.vel(1:Nuse)   = err_cv_total.vel(1:Nuse)   + eVcv(1:Nuse)/Nsim;

    err_adapt_total.long(1:Nuse)  = err_adapt_total.long(1:Nuse)  + eLad(1:Nuse)/Nsim;
    err_adapt_total.trans(1:Nuse) = err_adapt_total.trans(1:Nuse) + eTad(1:Nuse)/Nsim;
    err_adapt_total.rumbo(1:Nuse) = err_adapt_total.rumbo(1:Nuse) + eRad(1:Nuse)/Nsim;
    err_adapt_total.vel(1:Nuse)   = err_adapt_total.vel(1:Nuse)   + eVad(1:Nuse)/Nsim;
end

% === Clasificación por tramos y transiciones ===
err_cv = struct('long', zeros(Ntramos,1), ...
                'trans', zeros(Ntramos,1), ...
                'rumbo', zeros(Ntramos,1), ...
                'vel', zeros(Ntramos,1));
err_adapt = err_cv;


for i = 1:Ntramos
    t_ini = track(1).tramos_tiempos(i);
    t_fin = track(1).tramos_tiempos(i+1);

    idx = find((0:Nscans-1)'*Tmedida >= t_ini & (0:Nscans-1)'*Tmedida < t_fin);
    
    for campo = ["long","trans","rumbo","vel"]
        err_cv.(campo)(i) = mean(err_cv_total.(campo)(idx));
        err_adapt.(campo)(i) = mean(err_adapt_total.(campo)(idx));
    end
end

% === Transiciones ===

for i = 1:Ntramos-1
    t1 = track(1).tramos_tiempos(i+1);
    idx = find(abs((0:Nscans-1)'*Tmedida - t1) < Tmedida);  % ±1 escaneo

    if isempty(idx), continue; end

    tipoA = tipos_tramo(i);
    tipoB = tipos_tramo(i+1);

    eLcv = sqrt(mean(err_cv_total.long(idx)));
    eTcv = sqrt(mean(err_cv_total.trans(idx)));
    eRcv = sqrt(mean(err_cv_total.rumbo(idx))) * 180/pi;
    eVcv = sqrt(mean(err_cv_total.vel(idx)));

    eLad = sqrt(mean(err_adapt_total.long(idx)));
    eTad = sqrt(mean(err_adapt_total.trans(idx)));
    eRad = sqrt(mean(err_adapt_total.rumbo(idx))) * 180/pi;
    eVad = sqrt(mean(err_adapt_total.vel(idx)));

end



fprintf('\n--- COMPARACIÓN CON LA NORMATIVA EUROCONTROL ---\n');

for i = 1:Ntramos
    tipo = tipos_tramo(i);
    fprintf('\nTramo %d (%s) - Duración: %.1f s\n', i, tipo, tramos(i,4));

    % Umbrales EUROCONTROL por tipo de movimiento
    switch tipo
        case "Rectilíneo"
            umbral = struct('long', 60, 'trans', 60, 'vel', 0.6, 'rumbo', 0.7);
        case "Acelerado"
            umbral = struct('long', 180, 'trans', 60, 'vel', 17, 'rumbo', 1.5);
        case "Giro"
            umbral = struct('long', 100, 'trans', 100, 'vel', 4, 'rumbo', 6);
        otherwise
            warning('Tipo de tramo desconocido');
            continue;
    end

    % Extraer errores RMS
    ecv = structfun(@(x) sqrt(x(i)), err_cv, 'UniformOutput', false);
    ead = structfun(@(x) sqrt(x(i)), err_adapt, 'UniformOutput', false);

    % Comparar y mostrar
    campos = ["long", "trans", "vel", "rumbo"];
    nombres = ["Longitudinal", "Transversal", "Velocidad", "Rumbo"];

    for k = 1:numel(campos)
        campo = campos(k);
        label = nombres(k);

        % Kalman CV
        val_cv = ecv.(campo);
        limite = umbral.(campo);
        cumple_cv = val_cv <= limite;
        marca_cv = "NO CUMPLE"; if cumple_cv, marca_cv = "CUMPLE"; end

        % Kalman Adaptativo
        val_ad = ead.(campo);
        cumple_ad = val_ad <= limite;
        marca_ad = "NO CUMPLE"; if cumple_ad, marca_ad = "CUMPLE"; end

        unidad = "m";
        if campo == "vel", unidad = "m/s";
        elseif campo == "rumbo", unidad = "º"; end

        fprintf('  %s:\n', label);
        fprintf('    Kalman CV         : %.2f %s %s (≤ %.2f %s)\n', val_cv, unidad, marca_cv, limite, unidad);
        fprintf('    Kalman Maniobras  : %.2f %s %s (≤ %.2f %s)\n', val_ad, unidad, marca_ad, limite, unidad);
    end
end

fprintf('\n--- COMPARACIÓN EUROCONTROL EN TRANSICIONES ---\n');

for i = 1:Ntramos-1
    tipoA = tipos_tramo(i);
    tipoB = tipos_tramo(i+1);
    dur = track(1).tramos_tiempos(i+1) - track(1).tramos_tiempos(i);

    fprintf('\nTransición %d (%s → %s) - Duración: %.1f s\n', i, tipoA, tipoB, dur);

    % === Definir umbrales base y condicionales ===
    umbral = struct();

    if tipoA == "Rectilíneo" && tipoB == "Giro"
        umbral.long = 140;
        if dur > 24
            umbral.trans = 215;
            umbral.rumbo = 14.5;
        else
            umbral.trans = 230;
            umbral.rumbo = 17;
        end
        umbral.vel = 6;

    elseif tipoA == "Giro" && tipoB == "Rectilíneo"
        if dur > 65
            umbral.long = 71;
            umbral.trans = 78;
            umbral.vel = 1.1;
            umbral.rumbo = 1.6;
        else
            umbral.long = 110;
            umbral.trans = 180;
            umbral.vel = 5;
            umbral.rumbo = 9;
        end

    elseif tipoA == "Rectilíneo" && tipoB == "Acelerado"
        if dur > 50
            umbral.long = 211;
            umbral.vel = 18.6;
        else
            umbral.long = 310;
            umbral.vel = 26;
        end
        if dur > 60
            umbral.trans = 72;
        else
            umbral.trans = 120;
        end
        if dur > 65
            umbral.rumbo = 1.75;
        else
            umbral.rumbo = 2.5;
        end

    else
        fprintf('  ⚠ Transición no contemplada en normativa.\n');
        continue;
    end

    % === Errores ya calculados (ej. desde antes) ===
    errores_cv = struct('long', eLcv, 'trans', eTcv, 'vel', eVcv, 'rumbo', eRcv);
    errores_ad = struct('long', eLad, 'trans', eTad, 'vel', eVad, 'rumbo', eRad);

    campos = ["long", "trans", "vel", "rumbo"];
    nombres = ["Longitudinal", "Transversal", "Velocidad", "Rumbo"];

    for k = 1:numel(campos)
        campo = campos(k);
        nombre = nombres(k);
        unidad = "m";
        if campo == "vel", unidad = "m/s";
        elseif campo == "rumbo", unidad = "º"; end

        val_cv = errores_cv.(campo);
        val_ad = errores_ad.(campo);
        lim = umbral.(campo);

        cumple_cv = val_cv <= lim;
        cumple_ad = val_ad <= lim;
        marca_cv = "NO CUMPLE"; if cumple_cv, marca_cv = "CUMPLE"; end
        marca_ad = "NO CUMPLE"; if cumple_ad, marca_ad = "CUMPLE"; end

        fprintf('  %s:\n', nombre);
        fprintf('    Kalman CV         : %.2f %s %s (≤ %.2f %s)\n', val_cv, unidad, marca_cv, lim, unidad);
        fprintf('    Kalman Maniobras  : %.2f %s %s (≤ %.2f %s)\n', val_ad, unidad, marca_ad, lim, unidad);
    end
end







% === Gráficas temporales de errores ===
% t_total = (0:Nscans-1)' * Tmedida;
% limites = cumsum([0; tramos(:,4)]);
% 
% figure('Name', 'Errores RMS en el Tiempo - Kalman CV vs Detección Maniobra GIRO');
% titles = {'Longitudinal RMS [m]', 'Transversal RMS [m]', ...
%           'Rumbo RMS [º]', 'Velocidad RMS [m/s]'};
% fields = {'long', 'trans', 'rumbo', 'vel'};
% 
% for i = 1:4
%     subplot(2,2,i); hold on;
%     if strcmp(fields{i}, 'rumbo')
%         y_cv = sqrt(err_cv_total.(fields{i})) * (180/pi);
%         y_ad = sqrt(err_adapt_total.(fields{i})) * (180/pi);
%     else
%         y_cv = sqrt(err_cv_total.(fields{i}));
%         y_ad = sqrt(err_adapt_total.(fields{i}));
%     end
% 
% 
%     plot(t_total, y_cv, '--b', 'DisplayName', 'Kalman CV');
%     plot(t_total, y_ad, '-r', 'DisplayName', 'Maniobra ');
% 
%     for j = 1:length(limites)
%         if j == 2
%             xline(limites(j), ':k', 'LineWidth', 1, 'DisplayName', 'Cambio de tramo');
%         else
%             xline(limites(j), ':k', 'LineWidth', 1, 'HandleVisibility', 'off');
%         end
%     end
% 
%     title(titles{i});
%     xlabel('Tiempo [s]');
%     grid on;
%     legend show;
% end
