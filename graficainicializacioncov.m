% Parámetros de la medida en coordenadas polares
rho = 5;                    % Distancia medida (m)
theta = deg2rad(40);        % Ángulo en radianes

% Incertidumbres
sigma_theta = deg2rad(2);   % Incertidumbre en theta (rad)
sigma_rho = 0.8;            % Incertidumbre en rho (m)

% Coordenadas cartesianas de la medida
x = rho * cos(theta);
y = rho * sin(theta);

% Crear la figura
figure;
hold on;
axis equal;
axis([-1 7 -1 5]);
xlabel('x');
ylabel('y');
title('Transformación de covarianza de (\rho, \theta) a (x, y)');

% Ejes
quiver(0, 0, 6, 0, 0, 'k', 'LineWidth', 1.5); % eje x
quiver(0, 0, 0, 4, 0, 'k', 'LineWidth', 1.5); % eje y
text(6.2, 0, 'x', 'FontSize', 12);
text(0.2, 4.2, 'y', 'FontSize', 12);

% Vector de medida (rho)
plot([0, x], [0, y], 'k', 'LineWidth', 2);
text(2.2, 0.3, '\rho', 'FontSize', 12);
text(0.8, 0.6, '\theta', 'FontSize', 12);

% Arco para el ángulo theta
t = linspace(0, theta, 100);
r = 1.3;
plot(r*cos(t), r*sin(t), 'k');

% === Círculo de incertidumbre con radio sigma_theta * rho ===
r_circ = sigma_theta * rho;
theta_circ = linspace(0, 2*pi, 100);
plot(x + r_circ*cos(theta_circ), y + r_circ*sin(theta_circ), 'k', 'LineWidth', 1.2);

% Vectores de incertidumbre desde el extremo
quiver(x, y, sigma_rho*cos(theta), sigma_rho*sin(theta), 0, 'k', 'LineWidth', 1);
text(x + sigma_rho*cos(theta) + 0.2, y + sigma_rho*sin(theta), '\sigma_\rho', 'FontSize', 12);

quiver(x, y, -r_circ*sin(theta), r_circ*cos(theta), 0, 'k', 'LineWidth', 1);
text(x - r_circ*sin(theta) - 0.2, y + r_circ*cos(theta) + 0.1, '\sigma_\theta \cdot \rho', 'FontSize', 12);

hold off;
