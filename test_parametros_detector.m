% Generar trayectoria con o sin maniobra antes de empezar:
% Asegúrate de que existen las variables: target_real y track
% 1. Generar la trayectoria con giro
[track, radar, projection] = generarTrayectoria();

% 2. Generar las medidas ideales y reales
target_ideal = ideal_measurement(track, radar, projection);
target_real = real_measurement(target_ideal, radar, 1, 1, 0, 0, 0, projection);


% Parámetros de prueba
alfas = [0.1, 0.3, 0.5, 0.7, 0.9];
pfas = [0.01, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3];

q_nominal = 0.1;
q_maniobra = 5;

M = 2;  % dimensionalidad de la medida (x, y)
T = mean(diff(target_real.measure(:,2))); % Tiempo entre medidas
N = size(target_real.measure, 1);

resultados = [];  % Aquí se guardarán todas las pruebas

% Barrido de combinaciones alfa y PFA
for a = 1:length(alfas)
    for p = 1:length(pfas)
        alfa = alfas(a);
        PFA = pfas(p);
        Neq = (1 + alfa) / (1 - alfa) * M;
        gamma = chi2inv(1 - PFA, Neq);

        % Ejecutar el filtro con esta configuración
        [~, ~, ~, Z_vec, modo_maniobra, cambios] = kalman_tracker_maniobra_eval(...
            target_real, track, q_nominal, q_maniobra, alfa, gamma);

        % Métricas
        duracion_maniobra = sum(modo_maniobra) * T;
        n_falsas = sum(modo_maniobra);
        primer_k = find(modo_maniobra, 1);
        if isempty(primer_k)
            t_deteccion = NaN;
        else
            t_deteccion = primer_k * T;
        end

        % Guardar resultado
        resultados = [resultados;
            alfa, PFA, gamma, n_falsas, t_deteccion, duracion_maniobra, cambios];
    end
end


% Mostrar tabla
Tresult = array2table(resultados, ...
    'VariableNames', {'alfa', 'PFA', 'gamma', ...
                      'FalsasAlarmas', 'TiempoDeteccion_s', 'DuracionManiobra_s', 'CambiosModo'});

disp(Tresult)

% Crear etiquetas tipo 'a=0.3, PFA=0.05'
labels = strcat("a=", string(Tresult.alfa), ", PFA=", string(Tresult.PFA));

% Gráfico
figure;
hold on;
plot(Tresult.FalsasAlarmas, '-o', 'LineWidth', 1.5);
plot(Tresult.TiempoDeteccion_s, '-s', 'LineWidth', 1.5);
plot(Tresult.DuracionManiobra_s, '-^', 'LineWidth', 1.5);
plot(Tresult.CambiosModo, '-d', 'LineWidth', 1.5);
hold off;

legend('Falsas alarmas', 'Tiempo detección (s)', ...
       'Duración modo maniobra (s)', 'Cambios de modo', ...
       'Location', 'northwest');

xlabel('Configuración (alfa y PFA)');
xticks(1:height(Tresult));
xticklabels(labels);
xtickangle(45);
ylabel('Valor');
title('Comparación de métricas del detector de maniobra');
grid on;
