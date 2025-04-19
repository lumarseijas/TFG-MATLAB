function posGeod = radar2geodetic(dist,azim,elev, posRadar, geoide)


xlv = dist .* cos(elev*pi/180) .* sin(azim*pi/180);
ylv = dist .* cos(elev*pi/180) .* cos(azim*pi/180);
zlv = dist .* sin(elev*pi/180);
    
[xecef yecef zecef ] = lv2ecef (xlv,ylv,zlv,posRadar(1)*pi/180,...
    posRadar(2)*pi/180,posRadar(3),geoide);

[lat long alt] = ecef2geodetic(xecef, yecef, zecef, geoide);

posGeod = [ lat*180/pi long*180/pi alt ];
