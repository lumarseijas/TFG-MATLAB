# TFG

## Que llevo:

- Generación de trayectorias: con giro (sin aceleracion) y con aceleración (sin giro).

- Implementación del filtro de Kalman CV.

- Implementación del filtro con detección de maniobra.

- Evaluación Monte Carlo.

- Análisis de errores RMS por tramo y globales.

- Gráficas comparativas.

- Calculo del número de ensayos.

- Comparación con Eurocontrol.

## Que voy a hacer ahora:

- que detecte el movimiento y cambie el filtro

---

## Resultados
### Trayectoria 1: CON GIRO
- trayectoria: 

     ![TRAYECTORIA1](img/trayectoria_unica.jpg)

- en el tiempo: 

     ![TRAYECTORIA1tiempo](img/trayectoria_unica_tiempo.jpg)

- sigma_a=0.1: errores: 

     ![TRAYECTORIA1errores](img/errores_trayectoria_unica_q=0.1.jpg)

### montecarlo_por_tramos
- Nsim = 200 (error relativo un 10%)
- q=0.1
- ambos kalman con cv y con detección de maniobra

     ![KALMAN_Y_MANIOBRA_MONTECARLO](img/montecarlo_por_tramos.jpg)

#### Errores RMS por Tramo-Trayectoria sin aceleración con giro


---
### Trayectoria 2:CON ACELERACIÓN

- trayectoria:

     ![TRAYECTORIA2](img/trayectoria_unica_acel.jpg)

- en el tiempo:

     ![TRAYECTORIA2tiempo](img/trayectoria_unica_tiempo_acel.jpg)

- sigma_a=0.1: errores:

     ![TRAYECTORIA2errores](img/errores_trayectoria_unica_q=0.1_acel.jpg)

### montecarlo_por_tramos_acel

- Nsim = 200 (error relativo un 10%)
- q=0.1
- ambos kalman con cv y con detección de maniobra

     ![KALMAN_Y_MANIOBRA_MONTECARLO](img/montecarlo_por_tramos_acel.jpg)

### Errores RMS por Tramo – Trayectoria con Aceleración sin giro


## COMPARACIÓN CON EUROCONTROL:

### PRIMERO GIRO:

**Parámetros utilizados**:  
- `q_value = 0.1` (filtro CV)  
- `q_maniobra = 5` (modo maniobra)

| Tramo | Tipo       | Duración \[s] | Métrica      | Kalman CV | Maniobra  | Umbral EUROCONTROL | ¿CV Cumple? | ¿Maniobra Cumple? |
| ----- | ---------- | ------------- | ------------ | --------- | --------- | ------------------ | ----------- | ----------------- |
| 1     | Rectilíneo | 240.0         | Longitudinal | 263.54 m  | 268.44 m  | 60.0 m             | No          | No                |
|       |            |               | Transversal  | 138.44 m  | 123.13 m  | 60.0 m             | No          | No                |
|       |            |               | Velocidad    | 17.90 m/s | 15.48 m/s | 0.6 m/s            | No          | No                |
|       |            |               | Rumbo        | 0.05 º    | 0.06 º    | 0.7 º              | **Sí**      | **Sí**            |
| 2     | Giro       | 98.0          | Longitudinal | 740.37 m  | 270.83 m  | 100.0 m            | No          | No                |
|       |            |               | Transversal  | 1034.44 m | 221.93 m  | 100.0 m            | No          | No                |
|       |            |               | Velocidad    | 21.00 m/s | 10.85 m/s | 4.0 m/s            | No          | No                |
|       |            |               | Rumbo        | 0.19 º    | 0.21 º    | 6.0 º              | **Sí**      | **Sí**            |
| 3     | Rectilíneo | 262.0         | Longitudinal | 213.84 m  | 89.76 m   | 60.0 m             | No          | No                |
|       |            |               | Transversal  | 330.57 m  | 241.79 m  | 60.0 m             | No          | No                |
|       |            |               | Velocidad    | 8.71 m/s  | 3.61 m/s  | 0.6 m/s            | No          | No                |
|       |            |               | Rumbo        | 0.03 º    | 0.02 º    | 0.7 º              | **Sí**      | **Sí**            |
---
| Transición        | Métrica      | Kalman CV | Maniobra | Umbral EUROCONTROL | ¿CV Cumple? | ¿Maniobra Cumple? |
| ----------------- | ------------ | --------- | -------- | ------------------ | ----------- | ----------------- |
| Rectilíneo → Giro | Longitudinal | 777.13 m  | 93.93 m  | 140.0 m            | No          | **Sí**            |
|                   | Transversal  | 1115.40 m | 255.45 m | 215.0 m            | No          | No                |
|                   | Velocidad    | 39.46 m/s | 7.88 m/s | 6.0 m/s            | No          | No                |
|                   | Rumbo        | 9.85 º    | 6.79 º   | 14.5 º             | **Sí**      | **Sí**            |
| Giro → Rectilíneo | Longitudinal | 777.13 m  | 93.93 m  | 71.0 m             | No          | No                |
|                   | Transversal  | 1115.40 m | 255.45 m | 78.0 m             | No          | No                |
|                   | Velocidad    | 39.46 m/s | 7.88 m/s | 1.1 m/s            | No          | No                |
|                   | Rumbo        | 9.85 º    | 6.79 º   | 1.6 º              | No          | No                |

**Parámetros utilizados**:  
- `q_value = 0.3` (filtro CV)  
- `q_maniobra = 4` (modo maniobra)
| Tramo | Tipo       | Duración \[s] | Métrica      | Kalman CV | Maniobra  | Umbral EUROCONTROL | ¿CV Cumple? | ¿Maniobra Cumple? |
| ----- | ---------- | ------------- | ------------ | --------- | --------- | ------------------ | ----------- | ----------------- |
| 1     | Rectilíneo | 240.0         | Longitudinal | 255.65 m  | 257.09 m  | 60.0 m             | No          | No                |
|       |            |               | Transversal  | 132.99 m  | 123.63 m  | 60.0 m             | No          | No                |
|       |            |               | Velocidad    | 18.53 m/s | 16.70 m/s | 0.6 m/s            | No          | No                |
|       |            |               | Rumbo        | 0.05 º    | 0.06 º    | 0.7 º              | **Sí**      | **Sí**            |
| 2     | Giro       | 98.0          | Longitudinal | 513.68 m  | 278.58 m  | 100.0 m            | No          | No                |
|       |            |               | Transversal  | 610.40 m  | 218.30 m  | 100.0 m            | No          | No                |
|       |            |               | Velocidad    | 16.17 m/s | 10.07 m/s | 4.0 m/s            | No          | No                |
|       |            |               | Rumbo        | 0.20 º    | 0.21 º    | 6.0 º              | **Sí**      | **Sí**            |
| 3     | Rectilíneo | 262.0         | Longitudinal | 119.70 m  | 87.90 m   | 60.0 m             | No          | No                |
|       |            |               | Transversal  | 239.33 m  | 235.99 m  | 60.0 m             | No          | No                |
|       |            |               | Velocidad    | 4.53 m/s  | 3.76 m/s  | 0.6 m/s            | No          | No                |
|       |            |               | Rumbo        | 0.02 º    | 0.02 º    | 0.7 º              | **Sí**      | **Sí**            |
---
| Transición        | Métrica      | Kalman CV | Maniobra | Umbral EUROCONTROL | ¿CV Cumple? | ¿Maniobra Cumple? |
| ----------------- | ------------ | --------- | -------- | ------------------ | ----------- | ----------------- |
| Rectilíneo → Giro | Longitudinal | 404.73 m  | 92.27 m  | 140.0 m            | No          | **Sí**            |
|                   | Transversal  | 440.88 m  | 228.80 m | 215.0 m            | No          | No                |
|                   | Velocidad    | 19.31 m/s | 7.29 m/s | 6.0 m/s            | No          | No                |
|                   | Rumbo        | 8.11 º    | 6.71 º   | 14.5 º             | **Sí**      | **Sí**            |
| Giro → Rectilíneo | Longitudinal | 404.73 m  | 92.27 m  | 71.0 m             | No          | No                |
|                   | Transversal  | 440.88 m  | 228.80 m | 78.0 m             | No          | No                |
|                   | Velocidad    | 19.31 m/s | 7.29 m/s | 1.1 m/s            | No          | No                |
|                   | Rumbo        | 8.11 º    | 6.71 º   | 1.6 º              | No          | No                |
**Parámetros utilizados**:  
- `q_value = 0` (filtro CV)  
- `q_maniobra = 10` (modo maniobra)
| Tramo | Tipo       | Duración \[s] | Métrica      | Kalman CV  | Maniobra  | Umbral EUROCONTROL | ¿CV Cumple? | ¿Maniobra Cumple? |
| ----- | ---------- | ------------- | ------------ | ---------- | --------- | ------------------ | ----------- | ----------------- |
| 1     | Rectilíneo | 240.0         | Longitudinal | 244.46 m   | 259.38 m  | 60.0 m             | No          | No                |
|       |            |               | Transversal  | 155.14 m   | 115.21 m  | 60.0 m             | No          | No                |
|       |            |               | Velocidad    | 20.29 m/s  | 16.09 m/s | 0.6 m/s            | No          | No                |
|       |            |               | Rumbo        | 0.05 º     | 0.07 º    | 0.7 º              | **Sí**      | **Sí**            |
| 2     | Giro       | 98.0          | Longitudinal | 2486.59 m  | 244.05 m  | 100.0 m            | No          | No                |
|       |            |               | Transversal  | 4528.16 m  | 224.46 m  | 100.0 m            | No          | No                |
|       |            |               | Velocidad    | 4.81 m/s   | 12.16 m/s | 4.0 m/s            | No          | No                |
|       |            |               | Rumbo        | 0.11 º     | 0.21 º    | 6.0 º              | **Sí**      | **Sí**            |
| 3     | Rectilíneo | 262.0         | Longitudinal | 11466.77 m | 88.55 m   | 60.0 m             | No          | No                |
|       |            |               | Transversal  | 14765.44 m | 240.06 m  | 60.0 m             | No          | No                |
|       |            |               | Velocidad    | 66.72 m/s  | 4.04 m/s  | 0.6 m/s            | No          | No                |
|       |            |               | Rumbo        | 0.02 º     | 0.02 º    | 0.7 º              | **Sí**      | **Sí**            |
---
| Transición        | Métrica      | Kalman CV  | Maniobra | Umbral EUROCONTROL | ¿CV Cumple? | ¿Maniobra Cumple? |
| ----------------- | ------------ | ---------- | -------- | ------------------ | ----------- | ----------------- |
| Rectilíneo → Giro | Longitudinal | 3027.83 m  | 92.37 m  | 140.0 m            | No          | **Sí**            |
|                   | Transversal  | 10060.51 m | 261.97 m | 215.0 m            | No          | No                |
|                   | Velocidad    | 11.30 m/s  | 8.72 m/s | 6.0 m/s            | No          | No                |
|                   | Rumbo        | 2.38 º     | 7.58 º   | 14.5 º             | **Sí**      | **Sí**            |
| Giro → Rectilíneo | Longitudinal | 3027.83 m  | 92.37 m  | 71.0 m             | No          | No                |
|                   | Transversal  | 10060.51 m | 261.97 m | 78.0 m             | No          | No                |
|                   | Velocidad    | 11.30 m/s  | 8.72 m/s | 1.1 m/s            | No          | No                |
|                   | Rumbo        | 2.38 º     | 7.58 º   | 1.6 º              | No          | No                |

#### OBSERVACIONES ENTRE Q's:

- ninguna va igual
