# Computación del Espacio de Configuración para un Robot Plano 3-DOF

Este repositorio contiene el código fuente para la implementación y simulación del espacio de configuración (C-Space) de un robot plano de 3 grados de libertad (3-DOF). Se incluyen dos escenarios:  
- **Escenario con un único obstáculo:** Se analiza el efecto de un obstáculo fijo en el espacio de trabajo y se computa el C-Space que marca las configuraciones prohibidas.  
- **Escenario con múltiples obstáculos:** Se extiende la metodología para manejar múltiples obstáculos generados aleatoriamente, evaluando la intersección de cada uno de los 3 eslabones del robot con todos los obstáculos simultáneamente.

---

## Descripción

El proyecto implementa las siguientes funcionalidades:

- **Cinemática Directa:**  
  La función `forward_kinematics` calcula la posición de cada articulación y del efector final a partir de los ángulos θ1, θ2 y θ3 usando las ecuaciones:
x1=l1cos(θ1),				y1=l1sin(θ1)
x2=x1+l2cos(θ1+θ2),			y2=y1+l2sin(θ1+θ2)
x3=x2+l3cos(θ1+θ2+θ3), 		x3=y2+l3sin(θ1+θ2+θ3)
  
- **Detección de Colisiones:**  
  Se evalúa la intersección de cada uno de los 3 eslabones del robot contra obstáculos definidos como círculos en el espacio de trabajo. La función `is_collision` recorre cada segmento del robot para detectar colisiones.

- **C-Space:**  
  Se computa un espacio tridimensional (cubos en θ1, θ2 y θ3) donde se marca con valores especiales las configuraciones que generan colisión.

- **Planificación de Trayectorias:**  
  Se implementa la función `interpolate_path` para generar una secuencia de configuraciones intermedias mediante interpolación lineal en el espacio articular. Además, la función `compute_via_point` calcula un punto vía en el espacio de trabajo que, a partir de los ajustes acumulados para evitar obstáculos, permite replanificar la trayectoria.

- **Simulación del Movimiento:**  
  La función `simulate_arm_motion` anima el movimiento del robot a lo largo de la trayectoria planificada. Durante la simulación se verifica en tiempo real si alguna configuración está en colisión, dibujando la trayectoria en rojo (si hay colisión) o en azul (configuración segura).

---

## Estructura del Proyecto

```
├── README.md
├── simulacion_un_obstaculo.py         # Código para el escenario con un único obstáculo.
└── simulacion_varios_obstaculos.py      # Código para el escenario con múltiples obstáculos.
```

## Requisitos

- **Python 3.x**
- **Numpy**
- **Matplotlib**
- **Scipy**

Instala las dependencias con:

```bash
pip install numpy matplotlib scipy
```

---

## Cómo Ejecutar el Código

Para ejecutar el escenario con un único obstáculo:

```bash
python simulacion_un_obstaculo.py
```

Para ejecutar el escenario con múltiples obstáculos:

```bash
python simulacion_varios_obstaculos.py
```

Cada script generará:
- La visualización del C-Space, mostrando las regiones en las que se producen colisiones.
- Una animación del movimiento del robot desde la configuración inicial hasta la final, indicando en rojo aquellas configuraciones que presentan colisión y en azul las configuraciones seguras.
- Una gráfica de la configuración inicial del brazo.
- Una gráfica de la configuracion final del brazo.

---

## Uso e Interpretación

- **Cinemática Directa:** Calcula las posiciones de cada articulación del robot en función de los ángulos.
- **Detección de Colisiones:** Verifica si alguno de los 3 eslabones colisiona con los obstáculos, lo que se utiliza para penalizar configuraciones inseguras.
- **Planificación de Trayectorias:**  
  - *Interpolación de Trayectorias:* Genera configuraciones intermedias para una transición suave.  
  - *Punto Vía:* Ajusta la trayectoria para evitar zonas de colisión cuando la línea directa se acerca demasiado a un obstáculo o a varios simultáneamente.
- **Simulación:** La función `simulate_arm_motion` anima el movimiento del robot, permitiendo observar en tiempo real la evasión de obstáculos y la seguridad de cada configuración.

---

## Contribuciones

Si deseas contribuir a este proyecto, abre un *issue* para discutir tus propuestas o envía un *pull request* con tus mejoras. Se valorarán aportaciones que mejoren la optimización, la planificación de trayectorias o la visualización del C-Space.

---

## Licencia

Este proyecto se distribuye bajo la Licencia MIT. Consulta el archivo `LICENSE` para más detalles.

