# Práctica 2 - Ampliación de Robótica: Filtro de Kalman

El filtro de Kalman implementado en este proyecto está diseñado para estimar la posición y orientación (x, y, θ) de un robot (y en algunos casos también su velocidad: vx, vy, ω) basándose en mediciones ruidosas. Se han estudiado tres casos con diferentes configuraciones de ruido.

## Estructura del Filtro

Se implementaron dos versiones del filtro de Kalman:

1. **KalmanFilter**: Modelo básico que estima solo posición/orientación [x, y, θ]. Su implementación se encuentra en `/p2_kf_jflr/filters/kalman_filter.py`
2. **KalmanFilter_2**: Modelo extendido que estima posición/orientación y velocidad [x, y, θ, vx, vy, ω]. Su implementación se encuentra en `/p2_kf_jflr/filters/kalman_filter.py`

### Modelos matemáticos implementados:

- **Modelo de movimiento (predicción)**:
  - Para el modelo básico: Matriz de transición A y matriz de control B.
  - Para el modelo extendido: Incluye términos para la velocidad lineal y angular

- **Modelo de observación (corrección)**:
  - Matrices C que mapean el estado interno a las observaciones medidas

Todos los modelos se encuentran en `/p2_kf_jflr/motion_models.py` (predicción) y `/p2_kf_jflr/observation_models.py` (observación).

## Casos Estudiados

Todas las imágenes discutidas se encuentran `/media`.

### 1. Caso de Bajo Ruido (LowNoiseKalman1.png y LowNoiseKalman2.png)

**Configuración**:
- Ruido de proceso (R) bajo: `[0.02, 0.02, 0.01]` para el modelo de posición y `[0.02, 0.02, 0.01, 0.02, 0.02, 0.01]` para el modelo extendido. Se escogieron de forma arbitraria, en función de la fiabilidad del modelo.
- Ruido de observación (Q) bajo: valores similares a R. Se escogieron valores idénticos al ruido de medición.

**Resultados**:
- Se observa una convergencia rápida hacia los valores reales (ground truth).
- Error muy pequeño (~0.01) después de pocas iteraciones.
- Covarianza que disminuye rápidamente (valores muy pequeños como 0.000248 para iteración 3 de Kalman1).
- Las estimaciones son muy cercanas a los valores reales, con errores mínimos.
- Se observa una convergencia más veloz en el modelo extendido.

### 2. Caso de Alto Ruido en Mediciones (HighMeasureNoiseKalman1.png y HighMeasureNoiseKalman2.png)

**Configuración**:
- Ruido de proceso bajo, como en el experimento previo.
- Ruido de observación alto: `[0.1, 0.1, 0.05]` para posición y `[0.1, 0.1, 0.05, 0.1, 0.1, 0.05]` para el modelo extendido, 5 veces más alto que el caso 1.

**Resultados**:
- Mayor divergencia inicial entre estimaciones y valores reales.
- La covarianza disminuye más lentamente.
- Los errores son notablemente mayores, especialmente al principio.
- El filtro tarda más iteraciones en converger.
- Valores de covarianza más altos (0.002780 para iteración 3 de Kalman1) que en el caso de bajo ruido.

### 3. Caso de Alto Ruido en Proceso (HighProcessNoiseKalman1.png y HighProcessNoiseKalman2.png)

**Configuración**:
- Ruido de proceso alto: `[0.1, 0.1, 0.05]` para posición y `[0.1, 0.1, 0.05, 0.1, 0.1, 0.05]` para el modelo extendido, 5 veces más alto que el caso 1.
- Ruido de medición bajo, como en el primer experimento.

**Resultados**:
- El filtro confía más en las mediciones que en las predicciones.
- La covarianza tiende a mantenerse más elevada que en el caso de bajo ruido.
- Al final de las iteraciones, las estimaciones siguen mejorando pero no alcanzan la precisión del caso de bajo ruido (covarianza de 0.000385 para iteración 3 de Kalman1).

## Comparación de los Modelos

1. **Modelo básico**:
   - Más simple y ligero, estima solo posición.
   - Menos resistente a ruido alto en las mediciones.

2. **Modelo extendido**:
   - Incluye estimación de velocidades (vx, vy, ω).
   - Más robusto ante perturbaciones.
   - Puede producir mejores estimaciones a largo plazo.

El comportamiento observado corresponde a la teoría del filtro de Kalman: cuando el ruido de medición es alto, el filtro confía más en su modelo de predicción, mientras que cuando el ruido de proceso es alto, confía más en las mediciones.

El código de los nodos principales se encuentra en `/p2_kf_jflr/kf_estimation.py` (modelo de posición) y `/p2_kf_jflr/kf_estimation_vel.py` (modelo extendido).
