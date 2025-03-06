import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import differential_evolution
import math
from math import pi

# --- Cinemática directa ---
def forward_kinematics(theta, link_lengths):
    l1, l2, l3 = link_lengths
    # La base se fija en (0,0)
    x0, y0 = 0, 0
    x1 = x0 + l1 * np.cos(theta[0])
    y1 = y0 + l1 * np.sin(theta[0])
    x2 = x1 + l2 * np.cos(theta[0] + theta[1])
    y2 = y1 + l2 * np.sin(theta[0] + theta[1])
    x3 = x2 + l3 * np.cos(theta[0] + theta[1] + theta[2])
    y3 = y2 + l3 * np.sin(theta[0] + theta[1] + theta[2])
    positions = [(x0, y0), (x1, y1), (x2, y2), (x3, y3)]
    return positions

# --- Generación de obstáculos aleatorios ---
def generate_random_obstacles(num_obstacles, xlim, ylim, radius_range):
    obstacles = []
    for _ in range(num_obstacles):
        x = np.random.uniform(xlim[0], xlim[1])
        y = np.random.uniform(ylim[0], ylim[1])
        r = np.random.uniform(radius_range[0], radius_range[1])
        obstacles.append((x, y, r))
    return obstacles

# --- Funciones de colisión ---
def segment_circle_collision(p1, p2, obstacle):
    """
    Verifica si el segmento definido por p1 y p2 (un eslabón) interseca el círculo del obstáculo.
    Se aplica para cada uno de los 3 eslabones.
    """
    x1, y1 = p1
    x2, y2 = p2
    xc, yc, r = obstacle
    dx, dy = x2 - x1, y2 - y1
    fx, fy = x1 - xc, y1 - yc
    a = dx**2 + dy**2
    b = 2 * (fx * dx + fy * dy)
    c = (fx**2 + fy**2) - r**2
    discriminant = b**2 - 4 * a * c
    if discriminant < 0:
        return False  # No hay intersección
    discriminant = np.sqrt(discriminant)
    t1 = (-b - discriminant) / (2 * a)
    t2 = (-b + discriminant) / (2 * a)
    return (0 <= t1 <= 1) or (0 <= t2 <= 1)

def is_collision(positions, obstacles):
    """
    Revisa cada segmento (de la base a la articulación 1, de la articulación 1 a la 2,
    y de la articulación 2 al efector) contra cada uno de los obstáculos.
    """
    if not positions:
        return True
    for i in range(len(positions) - 1):
        for obs in obstacles:
            if segment_circle_collision(positions[i], positions[i+1], obs):
                return True
    return False

def check_trajectory_collision(trajectory, link_lengths, obstacles):
    """
    Verifica si alguna configuración intermedia en la trayectoria presenta colisión
    en alguno de sus 3 eslabones.
    """
    for theta in trajectory:
        positions = forward_kinematics(theta, link_lengths)
        if is_collision(positions, obstacles):
            return True
    return False

# --- Espacio de configuraciones (C-space) ---
def generate_cspace(resolution, link_lengths, obstacles):
    theta_range = np.linspace(-np.pi, np.pi, resolution)
    cspace = np.zeros((resolution, resolution, resolution))
    for i, t1 in enumerate(theta_range):
        for j, t2 in enumerate(theta_range):
            for k, t3 in enumerate(theta_range):
                positions = forward_kinematics([t1, t2, t3], link_lengths)
                if is_collision(positions, obstacles):
                    cspace[i, j, k] = 1
    return cspace

# --- Cinemática inversa con penalización ---
def inverse_kinematics(target, link_lengths, obstacles, cspace, resolution, initial_theta):
    best_solution = None
    best_cost = np.inf
    trajectory = []
    
    def objective(theta):
        positions = forward_kinematics(theta, link_lengths)
        if not positions:
            return np.inf
        # Penaliza fuertemente si alguno de los 3 eslabones colisiona con algún obstáculo
        if is_collision(positions, obstacles):
            return 1e8
        end_effector = positions[-1]
        dist_to_target = np.linalg.norm(np.array(end_effector) - np.array(target))
        # Penalización extra por proximidad a cualquier obstáculo
        safety_margin = 0.15
        penalty = 0
        for obs in obstacles:
            for (x, y) in positions:
                distance = np.linalg.norm([x - obs[0], y - obs[1]])
                if distance < obs[2] + safety_margin:
                    penalty += (obs[2] + safety_margin - distance) * 50
        return dist_to_target + penalty

    initial_guesses = [
        initial_theta,
        [np.pi/4, -np.pi/4, np.pi/4],
        [-np.pi/4, np.pi/4, -np.pi/4],
        [0, np.pi/2, -np.pi/2],
        [np.pi/2, 0, -np.pi/2]
    ]
    
    for guess in initial_guesses:
        result = differential_evolution(
            objective,
            bounds=[(-np.pi, np.pi)] * 3,
            strategy='best1bin',
            maxiter=2000,      # Se aumenta el número máximo de iteraciones
            popsize=15,        # Se aumenta el tamaño de la población
            tol=1e-6,          # Tolerancia más fina para la convergencia
            polish=True,
            updating='deferred'
        )
        if result.success:
            final_positions = forward_kinematics(result.x, link_lengths)
            cost = np.linalg.norm(np.array(final_positions[-1]) - np.array(target))
            if cost < best_cost and not is_collision(final_positions, obstacles):
                best_solution = result.x
                best_cost = cost
            trajectory.append(result.x)
    
    if best_solution is not None:
        return best_solution, np.array(trajectory)
    else:
        print("⚠ No se encontró una solución válida sin colisión.")
        return None, np.array([])

# --- Funciones para graficar ---
def plot_initial_configuration(initial_theta, link_lengths, obstacles):
    positions = forward_kinematics(initial_theta, link_lengths)
    if not positions:
        print("⚠ No se pudo graficar la configuración inicial del brazo.")
        return
    x_values, y_values = zip(*positions)
    plt.figure()
    plt.plot(x_values, y_values, 'bo-', linewidth=3, label='Configuración Inicial')
    for obs in obstacles:
        plt.scatter(obs[0], obs[1], color='k', s=100, label='Obstáculo')
    plt.xlim(-3, 3)
    plt.ylim(-3, 3)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Configuración Inicial del Brazo con Obstáculos')
    plt.grid()
    plt.show(block=False)
    
def plot_cspace(cspace, trajectory):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    theta_range = np.linspace(-np.pi, np.pi, cspace.shape[0])
    x, y, z = np.meshgrid(theta_range, theta_range, theta_range)
    ax.scatter(x[cspace == 1], y[cspace == 1], z[cspace == 1],
               c='r', marker='o', alpha=0.3, label='Colisión')
    if trajectory.shape[0] > 1:
        ax.plot(trajectory[:,0], trajectory[:,1], trajectory[:,2],
                'b-', linewidth=2, label='Cambio de Ángulos')
        ax.scatter(trajectory[0,0], trajectory[0,1], trajectory[0,2],
                   c='g', marker='o', s=100, label='Inicio')
        ax.scatter(trajectory[-1,0], trajectory[-1,1], trajectory[-1,2],
                   c='y', marker='o', s=100, label='Final')
    ax.set_xlabel('Theta1')
    ax.set_ylabel('Theta2')
    ax.set_zlabel('Theta3')
    plt.title('C-space con Trayectoria de Ángulos')
    plt.legend()
    plt.show()
    
def plot_robot(theta, link_lengths, obstacles):
    fig, ax = plt.subplots()
    positions = forward_kinematics(theta, link_lengths)
    if not positions:
        print("⚠ Error en forward_kinematics.")
        return
    x_values, y_values = zip(*positions)
    ax.plot(x_values, y_values, 'bo-', linewidth=3, label='Brazo Robótico')
    for obs in obstacles:
        ax.scatter(obs[0], obs[1], color='k', s=100, label='Obstáculo')
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.legend()
    ax.set_title('Configuración final del Brazo con Obstáculos')
    ax.grid()
    plt.show(block=False)

# --- Funciones para la simulación del movimiento ---

def interpolate_path(theta_start, theta_end, steps):
    """Genera una trayectoria lineal entre dos configuraciones en el espacio articular."""
    return np.linspace(theta_start, theta_end, steps)

def compute_via_point(initial_ee, target, obstacles, margin=0.3):
    """
    Calcula un punto vía en el espacio (x,y) si la línea directa entre la posición inicial y
    el objetivo se acerca demasiado a alguno (o varios) de los obstáculos.
    Para cada obstáculo se calcula el ajuste requerido; se promedian los ajustes de todos los
    obstáculos que infrinjan el margen y se retorna el nuevo punto.
    """
    P0 = np.array(initial_ee)
    P1 = np.array(target)
    v = P1 - P0
    if np.linalg.norm(v) == 0:
        return None
    v_norm = v / np.linalg.norm(v)
    adjustments = []
    for obs in obstacles:
        O = np.array([obs[0], obs[1]])
        t = np.dot(O - P0, v_norm)
        projection = P0 + t * v_norm
        d = np.linalg.norm(O - projection)
        if d < obs[2] + margin:
            offset = obs[2] + margin - d
            # Calcular dos candidatos desplazando perpendicularmente
            perp = np.array([-v_norm[1], v_norm[0]])
            candidate1 = projection + perp * offset
            candidate2 = projection - perp * offset
            # Escoger el candidato que mayor distancia genere respecto al obstáculo
            chosen = candidate1 - projection if np.linalg.norm(candidate1 - O) > np.linalg.norm(candidate2 - O) else candidate2 - projection
            adjustments.append(chosen)
    if adjustments:
        avg_adjustment = np.mean(adjustments, axis=0)
        return P0 + avg_adjustment
    else:
        return None

def simulate_arm_motion(path, link_lengths, obstacles, target):
    """
    Anima el movimiento del brazo a lo largo de la trayectoria (en espacio articular).
    En cada frame se dibuja el brazo y todos los obstáculos; si en la configuración actual
    se detecta colisión (en alguno de los 3 eslabones contra cualquier obstáculo), el brazo se dibuja en rojo.
    """
    fig, ax = plt.subplots()
    for theta in path:
        ax.clear()
        positions = forward_kinematics(theta, link_lengths)
        x, y = zip(*positions)
        if is_collision(positions, obstacles):
            # Si hay colisión, dibuja el brazo en rojo
            ax.plot(x, y, 'ro-', linewidth=3, label='Brazo (colisión)')
        else:
            ax.plot(x, y, 'bo-', linewidth=3, label='Brazo')
        for obs in obstacles:
            circle = plt.Circle((obs[0], obs[1]), obs[2], color='r', alpha=0.5)
            ax.add_patch(circle)
        ax.scatter(target[0], target[1], color='g', s=100, label='Objetivo')
        ax.set_xlim(-3, 3)
        ax.set_ylim(-3, 3)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Simulación del movimiento del brazo')
        ax.grid()
        plt.pause(0.05)
    plt.show()

# --- Parámetros y ejecución principal ---
# link_lengths = [1, 1, 1]
# num_obstacles = 3
# obstacles = generate_random_obstacles(num_obstacles, xlim=(-2, 2), ylim=(-2, 2), radius_range=(0.1, 0.3))
# resolution = 20
# target_xy = (0.5, 1)
# initial_theta = [-np.pi/4, np.pi/2, -np.pi/3]  # Configuración inicial

l1=float(input("Longitud del eslabón 1: "))
l2=float(input("Longitud del eslabón 2: "))
l3=float(input("Longitud del eslabón 3: "))
t1=eval(input("Valor de tetha 1 en radianes [usar pi] "))
t2=eval(input("Valor de tetha 2 en radianes [usar pi] "))
t3=eval(input("Valor de tetha 3 en radianes [usar pi] "))
n_o=int(input("Cantidad de obstáculos: "))
tx=float(input("Coordenada en x del target: "))
ty=float(input("Coordenada en y del target: "))


link_lengths = [l1, l2, l3]
num_obstacles = n_o
obstacles = generate_random_obstacles(num_obstacles, xlim=(-2, 2), ylim=(-2, 2), radius_range=(0.1, 0.3))
resolution = 20
target_xy = (tx, ty)
initial_theta = [t1, t2, t3]

# Generar y graficar el C-space
cspace = generate_cspace(resolution, link_lengths, obstacles)
angles, trajectory = inverse_kinematics(target_xy, link_lengths, obstacles, cspace, resolution, initial_theta)


if angles is not None:
    
    print("Ángulos finales:", angles)
    
    # Obtener la posición inicial del efector final
    initial_ee = forward_kinematics(initial_theta, link_lengths)[-1]
    # Calcular un punto vía si la línea directa se acerca a alguno (o varios) de los obstáculos
    via_point = compute_via_point(initial_ee, target_xy, obstacles, margin=0.3)
    
    if via_point is not None:
        print("Se usará un punto vía para evitar los obstáculos:", via_point)
        # Resolver la cinemática inversa para el punto vía
        config_via, _ = inverse_kinematics(via_point, link_lengths, obstacles, cspace, resolution, initial_theta)
        if config_via is not None:
            path1 = interpolate_path(initial_theta, config_via, steps=50)
            path2 = interpolate_path(config_via, angles, steps=50)
            full_path = np.vstack((path1, path2))
        else:
            full_path = interpolate_path(initial_theta, angles, steps=100)
    else:
        full_path = interpolate_path(initial_theta, angles, steps=100)
    
    # Animar la simulación del movimiento
    simulate_arm_motion(full_path, link_lengths, obstacles, target_xy)
    plot_robot(angles, link_lengths, obstacles)
else:
    print("⚠ No se encontró una solución válida sin colisión.")

# Graficar la configuración inicial del robot con obstáculos
plot_initial_configuration(initial_theta, link_lengths, obstacles)
# (Opcional) Graficar el C-space y la trayectoria en el espacio articular
plot_cspace(cspace, trajectory)
