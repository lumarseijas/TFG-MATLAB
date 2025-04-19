function elev = elevation2 (dist,alt, posRadar, geoide)
% posRadar(:,1) = latitud
% posRadar(:,3) = altura

N = length(dist);

a = geoide(1);
e = geoide(2);
R = a * (1-e^2) ./ sqrt( (1-e^2*sin(posRadar(1)*pi/180).^2).^3 );
elev = 180/pi * asin ( (2*R.*(alt-ones(N,1)*posRadar(3)) + alt.^2 -...
    ones(N,1)*posRadar(3)^2 - dist.^2 ) ./ (2*(dist*(R+posRadar(3)))) );