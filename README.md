# TFG
## Generación principal
### `PFC_ejemplo_generacion.m`
Script principal que orquesta todo el sistema:
- Define el geoide WGS84 y la proyección estereográfica.
- Configura los parámetros del radar.
- Genera una trayectoria con `trayectMia.m`.
- Calcula las medidas ideales (`ideal_measurement.m`) y reales (`real_measurement.m`) simulando errores.
- Representa visualmente las medidas en el plano estereográfico.
---

## Trayectoria de la aeronave

### `trayectMia.m`
Genera la trayectoria de la aeronave en coordenadas geodésicas a partir de:
- Tramos con aceleración longitudinal, transversal y vertical.
- Parámetros iniciales: posición, velocidad, rumbo y tiempo.
- Soporta trayectorias rectilíneas, curvas (giros) y aceleradas.

**Salida:** Posición, tiempo, velocidad, rumbo y velocidad vertical.

---

## Medidas radar ideales

### `ideal_measurement.m`
Simula las medidas de un radar ideal (sin errores):
- Calcula distancia, acimut y elevación.
- Filtra detecciones según barrido del radar.
- Añade posición geodésica, altitud y proyección estereográfica.
- Calcula velocidad, rumbo y velocidad vertical de cada muestra.

---

## Medidas radar con errores

### `real_measurement.m`
Añade errores a las medidas ideales para simular un radar realista:
- Errores sistemáticos (offsets) y aleatorios en distancia, acimut y altitud.
- Cuantización de altitud en pasos de 100 pies.
- Calcula elevación con `elevation2.m`.
- Convierte a coordenadas geodésicas con `radar2geodetic.m`.
- Proyecta a coordenadas estereográficas.
- Calcula matriz de covarianza para medidas en 2D.
- Estima velocidades con ruido (opcional).

---

## Transformaciones y utilidades

### `elevation2.m`
Calcula el ángulo de elevación entre radar y blanco a partir de distancia y altitud.

### `radar2geodetic.m`
Transforma coordenadas polares (distancia, acimut, elevación) en coordenadas geodésicas (latitud, longitud, altitud) usando conversiones ECEF/ENU.

