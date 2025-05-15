function mejoresValoresQ()
% Busca los mejores valores de q_value y q_maniobra que cumplen Eurocontrol

q_vals = [0.1, 0.2, 0.3, 0.4, 0.5, 0.7, 1.0];
q_manis = [1, 2, 3, 3.5, 4, 4.5, 5];

resultados = [];

fprintf('Probando combinaciones de q_value y q_maniobra...\n\n');

for q = q_vals
    for qm = q_manis
        % Asignar los valores a las variables del script base
        assignin('base', 'q_value', q);
        assignin('base', 'q_maniobra', qm);

        % Ejecutar el script y capturar salida
        salida = evalc('montecarlo_por_tramos');

        % Contar ocurrencias reales de líneas que contienen solo lo necesario
        n_cumple = count(salida, ' CUMPLE ');
        n_nocumple = count(salida, ' NO CUMPLE ');

        % Guardar resultados
        resultados = [resultados; q, qm, n_cumple, n_nocumple];
        fprintf('q = %.2f, q_maniobra = %.2f → si %d  no %d\n', q, qm, n_cumple, n_nocumple);
    end
end

% Ordenar por máximo número de cumplimientos
[~, idx] = sort(resultados(:,3), 'descend');
mejores = resultados(idx,:);

fprintf('\n--- MEJORES COMBINACIONES ---\n');
for i = 1:min(5, size(mejores,1))
    fprintf('q = %.2f, q_maniobra = %.2f → si %d  no %d\n', ...
        mejores(i,1), mejores(i,2), mejores(i,3), mejores(i,4));
end

% Comprobar si alguna combinación cumple TODO
idx_todo = find(mejores(:,4) == 0, 1);
if isempty(idx_todo)
    fprintf('\n Ninguna combinación ha cumplido completamente todos los requisitos.\n');
else
    fprintf('\n ¡Combinación que CUMPLE TODO!: q = %.2f, q_maniobra = %.2f\n', ...
        mejores(idx_todo,1), mejores(idx_todo,2));
end
end
