# TFG
## Que llevo:
- Generacion de trayectorias realistas
- Simulacion de mediciones radar con errores
- Implementacion del FK sencillo
- Simulacion MonteCarlo con errores
---
## Estructura de archivos

| Archivo | Descripción |
|:--------|:------------|
| `generarTrayectoria.m` | Genera una trayectoria sintética de avión, radar y proyección estereográfica. |
| `ideal_measurement.m` | Calcula las medidas ideales (sin errores) que recibiría el radar. |
| `real_measurement.m` | Introduce errores aleatorios y sistemáticos realistas en las medidas. |
| `kalman_tracker.m` | Aplica el filtro de Kalman de velocidad constante. `q` es parámetro de entrada. |
| `trayectMia.m` | Genera la trayectoria segmentada a partir de aceleraciones longitudinales, transversales y verticales. |
| `elevation2.m` | Calcula la elevación radar hacia la aeronave. |
| `radar2geodetic.m` | Convierte coordenadas radar a geodésicas (latitud, longitud, altura). |
| `montecarlo_prueba.m` | Ejecuta simulaciones Monte Carlo y calcula errores estadísticos. |

---

## Resultados

- Monte Carlo ejecutado con 100 simulaciones.
- Resultados representativos:
  - Error longitudinal RMS ≈ 41 km.
  - Error transversal RMS ≈ 10 km.
  - Error rumbo RMS ≈ 50°.
  - Error velocidad RMS ≈ 150 m/s.
*esta un poco mal*
---