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
- `q_value = 2` (filtro CV)  
- `q_maniobra = 5` (modo maniobra)

| Tramo | Tipo        | Duración [s] | Métrica       | Kalman CV        | Maniobra         | Umbral EUROCONTROL |
|-------|-------------|--------------|---------------|------------------|------------------|---------------------|
| 1     | Rectilíneo  | 240.0        | Longitudinal  | <span style="color:red">132.88 m</span> | <span style="color:red">132.05 m</span> | 60.0 m              |
|       |             |              | Transversal   | <span style="color:red">96.42 m</span>  | <span style="color:red">96.65 m</span>  | 60.0 m              |
|       |             |              | Rumbo         | <span style="color:red">73.31 º</span>  | <span style="color:red">73.29 º</span>  | 0.7 º               |
|       |             |              | Velocidad     | <span style="color:red">9.21 m/s</span> | <span style="color:red">8.86 m/s</span> | 0.6 m/s             |
| 2     | Giro        | 98.0         | Longitudinal  | <span style="color:green">93.34 m</span>  | <span style="color:green">81.32 m</span>  | 140.0 m             |
|       |             |              | Transversal   | <span style="color:green">92.14 m</span>  | <span style="color:green">80.65 m</span>  | 230.0 m             |
|       |             |              | Rumbo         | <span style="color:red">36.83 º</span>  | <span style="color:red">36.66 º</span>  | 17.0 º              |
|       |             |              | Velocidad     | <span style="color:green">3.58 m/s</span> | <span style="color:green">3.33 m/s</span> | 6.0 m/s             |
| 3     | Rectilíneo  | 262.0        | Longitudinal  | <span style="color:red">127.91 m</span> | <span style="color:red">128.20 m</span> | 110.0 m             |
|       |             |              | Transversal   | <span style="color:green">72.12 m</span>  | <span style="color:green">72.35 m</span>  | 180.0 m             |
|       |             |              | Rumbo         | <span style="color:red">17.36 º</span>  | <span style="color:red">17.42 º</span>  | 9.0 º               |
|       |             |              | Velocidad     | <span style="color:green">3.32 m/s</span> | <span style="color:green">3.53 m/s</span> | 5.0 m/s             |


#### OBSERVACIONES ENTRE Q's:

