function target = ideal_measurement( track, radar, projection )

Ntracks = length(track);
Nradar = length(radar);
target(Ntracks) = struct('measure',[]);
geoide = projection.geoid;

for j=1:Ntracks
    for k=1:Nradar
        [elev dist azim] = elevation(...
            radar(k).posGeod(1),radar(k).posGeod(2),radar(k).posGeod(3),...
            track(j).posGeod(:,1),track(j).posGeod(:,2),track(j).posGeod(:,3),...
            'degrees',geoide);
        lat = track(j).posGeod(:,1);
        long = track(j).posGeod(:,2);
        alt = track(j).posGeod(:,3);
        tiempo = track(j).tiempo;
        velocidad=track(j).velocidad;
        rumbo=track(j).rumbo;
        velascen=track(j).velascen;
        
        unwrapedAntenaAzim = 360*(tiempo-radar(k).Tini)/radar(k).Tr;
        unwrapedTargetAzim = unwrap(azim*pi/180)*180/pi;
        difAzim = mod(unwrapedAntenaAzim - unwrapedTargetAzim,360);
        difAzim=mod(difAzim+180,360)-180;
%         comp = find(difAzim>180);
%         difAzim(comp) = 360 - difAzim(comp);
        
%         figure;
%         plot(difAzim)
        
        ind = find ( difAzim < radar(k).resAzim/2);
        extremo1 = 0;
        sobrantes = [];
        for i=1:length(ind)-1
            if  ( ind(i+1) == ind(i)+1 )  
                if extremo1 == 0
                    extremo1 = i;
                else
                    continue;
                end                
            else
                if extremo1 == 0;
                    continue;
                else
                    extremo2 = i;
                    indCentral = round((extremo2+extremo1)/2);
                    if(indCentral>extremo1)
                        sobrantes = [ sobrantes  extremo1:indCentral-1 ];
                    end
                    if(indCentral<extremo2)
                        sobrantes = [sobrantes  indCentral+1:extremo2 ];
                    end
                    extremo1 = 0;
                end                    
            end
        end
        
        if(extremo1 ~= 0)
            extremo2 = length(ind);
            indCentral = round((extremo2+extremo1)/2);
            if(indCentral>extremo1)
                sobrantes = [ sobrantes  extremo1:indCentral-1 ];
            end
            if(indCentral<extremo2)
                sobrantes = [sobrantes  indCentral+1:extremo2 ];
            end
        end
        
        ind(sobrantes) = [];
        
        lat = lat(ind);
        long = long(ind);
        dist = dist(ind);
        azim = azim(ind);
        alt = alt(ind);
        tiempo = tiempo(ind);
        velocidad = velocidad(ind);
        rumbo = rumbo(ind);
        velascen=velascen(ind);
        
                       
        i = find (dist > radar(k).range);
        lat(i) = [];
        long(i) = [];
        dist(i) = [];
        azim(i) = [];
        alt(i) = [];
        tiempo(i) = [];
        velocidad(i)=  [];
        rumbo(i)=  [];
        velascen(i)= [];
               
        %elevacion
        elev = elevation2(dist, alt, radar(k).posGeod, geoide) ;
              
        %coordenadas esteriográficas
        [x y] = stereo(projection,lat,long, 'surface','forward');
        
        target(j).measure = [ target(j).measure ;...
            radar(k).id*ones(length(dist),1) tiempo dist azim alt...
            elev lat long alt x y velocidad rumbo velascen] ;
        
    end
end

% Devuelve el control

return

        