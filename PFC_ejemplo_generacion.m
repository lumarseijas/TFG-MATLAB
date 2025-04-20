% Ejemplo de generación de trayectoria (aquí utilizaremos coordenadas
% geodésicas)

%%Datos de la tierra, del centro de proyección y de la proyección

%Elipsoide de la tierra
%geoide = almanac('earth','wgs84','meters');
geoide  = referenceEllipsoid("wgs84","meter");

%Centro de proyeccion de la proyección estereográfica (sobre España)
latP = 40.5;
longP = 0.5;
altP = 0;

projection = defaultm('stereo');
origen = [latP longP 0];
projection.origin = origen;
projection.geoid = geoide;
projection = defaultm(projection);

%Tiempos de muestreo de la trayectoria
Ts=0.001;            %Intervalo de muestreo de la trayectoria


%% Especificación de la posición y datos del radar (se especifican en el plano estereográfico). En este caso está situado en el centro de proyección
[radar(1).posGeod(1), radar(1).posGeod(2)] = stereo(...
         projection,0,0,'surface','inverse');   %Posición del radar 1 (lat º, lon º, h en m)
radar(1).posGeod(3)=0;      %Altura del radar
radar(1).id = 1;
radar(1).range = 400e3;
radar(1).resDist = 70;%Desviación típica en distancia del radar 1 (m)
radar(1).resAzim =0.08;%Desviación típica en acimut del radar 1 (º)
radar(1).Tr = 4;%Tiempo de scan radar1 (segundos)
radar(1).Tini = 0;
radar(1).VelMS = 0;
radar(1).StVel = 0;
    
%% Datos iniciales de la trayectoria (se convierte la posición inicial del plano estereográfico a coordenadas geodésicas)
[yini, xini] = stereo(...
    projection,-26.305e3,150e3+26.305e3,'surface','inverse');  %X,Y iniciales en lat lon
zini=10e3;
vini=155;
rini=135;
cini=0;
tramos=[0 0 0 240;0 11 0 98;0 0 0 262];         % Los tramos se especifican como Acel longitudinal, Acel Transversal, Acel vertical y duración.
    
    
%% Generación de la trayectoria ideal sobremuestreada

% Generación en coordenadas geodésicas
o = 0; %Tiempo de inicio de la trayectoria

[track(1).posGeod, track(1).tiempo, track(1).velocidad, track(1).rumbo, track(1).velascen] = ...
    trayectMia(tramos, [yini xini zini], vini, rini, To, Ts, geoide);

% Proyección de la trayectoria en el plano estereográfico y de la posición
% del radar

[radar(1).posStereo(1), radar(1).posStereo(2)] = stereo(...
    projection,radar(1).posGeod(1),radar(1).posGeod(2),'surface','forward');
radar(1).posStereo(3)=radar(1).posGeod(3);

[track(1).posStereo(:,1), track(1).posStereo(:,2)] = stereo(...
    projection,track(1).posGeod(:,1),track(1).posGeod(:,2),'surface','forward');
track(1).posStereo(:,3)=track(1).posGeod(:,3);

%% Medidas del radar

radar(1).Tini=rand(1,1)*radar(1).Tr;  %Aleatorizacion del tiempo inicial del radar
target_ideal = ideal_measurement( track, radar, projection );   %Posición ideal de avistamiento
target_real = real_measurement(target_ideal, radar,1,1,0,0,0,projection);   %GENERACIÓN DE LA MEDIDA CON LOS ERRORES

plot(target_real.measure(:,13)/1e3,target_real.measure(:,14)/1e3,'+m')

%% A partir de aquí iría el tracker

%********************TRACKER DE LA FUSION TOOLBOX*******
%estimates = kalman_tracker(target_real);
[estimates, speed, rumbo] = kalman_tracker(target_real, track);
%%  Gráficas de ajuste de filtros
% === Variables necesarias ===
x_real = track(1).posStereo(:,1);
y_real = track(1).posStereo(:,2);
x_est = estimates(:,1);
y_est = estimates(:,2);

T = 4;  % Tiempo entre medidas
tiempo = (0:length(x_est)-1)' * T;

% Errores
dx = x_est - x_real(1:length(x_est));
dy = y_est - y_real(1:length(y_est));
err_total = [dx'; dy'];

% Dirección real
vx_real = gradient(x_real, T);
vy_real = gradient(y_real, T);
rumbo = atan2(vy_real, vx_real);  % en radianes

% Proyección de errores
longitudinal = dx .* cos(rumbo(1:length(dx))) + dy .* sin(rumbo(1:length(dy)));
transversal  = -dx .* sin(rumbo(1:length(dx))) + dy .* cos(rumbo(1:length(dy)));

% Rumbo estimado y real
vx_est = gradient(x_est, T);
vy_est = gradient(y_est, T);
rumbo_est = atan2(vy_est, vx_est) * 180/pi;
rumbo_real = rumbo(1:length(rumbo_est)) * 180/pi;

% Velocidad sobre el suelo
velocidad = sqrt(vx_est.^2 + vy_est.^2);

% === Cálculo de errores medios y RMS ===
mean_long = mean(longitudinal);
mean_trans = mean(transversal);
rms_long = sqrt(mean(longitudinal.^2));
rms_trans = sqrt(mean(transversal.^2));

fprintf('\n--- Errores de seguimiento ---\n');
fprintf('Error medio longitudinal: %.2f m\n', mean_long);
fprintf('Error medio transversal:  %.2f m\n', mean_trans);
fprintf('RMS longitudinal: %.2f m\n', rms_long);
fprintf('RMS transversal:  %.2f m\n', rms_trans);

% === PLOTS ===
figure;

subplot(4,1,1);
plot(tiempo, longitudinal.^2, 'b');
title('1. Error cuadrático longitudinal');
xlabel('Tiempo (s)'); ylabel('Error^2 (m²)'); grid on;

subplot(4,1,2);
plot(tiempo, transversal.^2, 'r');
title('2. Error cuadrático transversal');
xlabel('Tiempo (s)'); ylabel('Error^2 (m²)'); grid on;

subplot(4,1,3);
plot(tiempo, rumbo_est, 'b', tiempo, rumbo_real, '--g');
title('3. Rumbo');
xlabel('Tiempo (s)'); ylabel('Rumbo (º)');
legend('Estimado', 'Real'); grid on;

subplot(4,1,4);
plot(tiempo, velocidad, 'k');
title('4. Velocidad');
xlabel('Tiempo (s)'); ylabel('Velocidad (m/s)'); grid on;