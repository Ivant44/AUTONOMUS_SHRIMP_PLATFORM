#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
import numpy as np
import math

# ==================================================
# WAYPOINTS BASE
# ==================================================
WAYPOINT_TRAJECTORY = np.array([
    [0.0, 0.0],
    [2.0, 0.0],
    [4.0, 1.0],
    [5.0, 3.0],
    [4.0, 5.0],
    [2.0, 6.0],
    [0.0, 5.0],
    [-1.0, 3.0],
    [-2.0, 1.0],
], dtype=float)


# ==================================================
# 1) WAYPOINTS LINEALES
# ==================================================
def build_waypoint_path(samples_per_segment=50):
    waypoints = WAYPOINT_TRAJECTORY
    path_x = []
    path_y = []

    for i in range(len(waypoints)-1):
        p0 = waypoints[i]
        p1 = waypoints[i+1]

        t = np.linspace(0,1,samples_per_segment)

        path_x.extend(p0[0] + (p1[0]-p0[0])*t)
        path_y.extend(p0[1] + (p1[1]-p0[1])*t)

    return np.array(path_x), np.array(path_y)


# ==================================================
# 2) TRAYECTORIA EN S
# ==================================================
def build_s_curve():
    x = np.linspace(0, 8, 400)
    y = 2*np.sin(0.8*x)
    return x, y


# ==================================================
# 3) FIGURA DE 8
# ==================================================
def build_figure8():
    t = np.linspace(0, 2*np.pi, 500)

    x = 3*np.sin(t)
    y = 2*np.sin(t)*np.cos(t)

    return x, y


# ==================================================
# 4) CIRCULAR
# ==================================================
def build_circle():
    t = np.linspace(0, 2*np.pi, 500)

    x = 2*np.cos(t)
    y = 2*np.sin(t)

    return x, y


# ==================================================
# 5) SPLINE SUAVE
# ==================================================
def build_spline():
    from scipy.interpolate import CubicSpline

    pts = np.array([
        [0,0],
        [2,1],
        [4,4],
        [6,2],
        [8,5],
        [10,0]
    ])

    t = np.arange(len(pts))

    cs_x = CubicSpline(t, pts[:,0])
    cs_y = CubicSpline(t, pts[:,1])

    t_new = np.linspace(0, len(pts)-1, 500)

    x = cs_x(t_new)
    y = cs_y(t_new)

    return x, y


# ==================================================
# SELECTOR GENERAL
# ==================================================
AVAILABLE_TRAJECTORIES = ("waypoints", "s_curve", "figure8", "circle", "spline")


def build_controller_reference_path(mode="waypoints"):
    if mode == "waypoints":
        return WAYPOINT_TRAJECTORY, *build_waypoint_path()
    if mode == "s_curve":
        x, y = build_s_curve()
        return None, x, y
    if mode == "figure8":
        x, y = build_figure8()
        return None, x, y
    if mode == "circle":
        x, y = build_circle()
        return None, x, y
    if mode == "spline":
        x, y = build_spline()
        return None, x, y

    raise ValueError(
        f"Trayectoria desconocida: {mode}. Opciones: {', '.join(AVAILABLE_TRAJECTORIES)}"
    )


class PurePursuitAckermann(Node):
    def __init__(self):
        super().__init__('pure_pursuit_ackermann')

        # =========================
        # PARÁMETROS DEL ROBOT (Ackermann SHRIMP)
        # =========================
        self.declare_parameter('wheelbase', 0.175)
        self.L = self.get_parameter('wheelbase').value
        self.delta_max = np.radians(40)   # Máximo ángulo de dirección [rad]
        self.declare_parameter('heading_offset_deg', 0.0)
        self.heading_offset = np.radians(
            self.get_parameter('heading_offset_deg').value
        )
        self.declare_parameter('control_point_offset_x', 0.0)
        self.declare_parameter('control_point_offset_y', 0.0)
        self.control_point_offset_x = self.get_parameter('control_point_offset_x').value
        self.control_point_offset_y = self.get_parameter('control_point_offset_y').value
        
        # =========================
        # PARÁMETROS PURE PURSUIT (RE-SINTONIZADOS)
        # =========================
        self.lookahead_distance = 0.1 # Reducido de 0.5 [m]
        self.k_velocity = 0.3           # Reducido de 0.8 para más suavidad
        self.k_steering = 1.0     # Aumentado de 1.5 para más giro
        self.declare_parameter('rear_steering_ratio', -0.4)
        self.rear_steering_ratio = self.get_parameter('rear_steering_ratio').value
        
        # Límites
        self.max_speed = 0.3           # Reducido de 1.0 [m/s]
        self.max_omega = 2.5             # Velocidad angular máxima [rad/s]
        
        # Parámetros de seguridad
        self.min_speed = 0.05            # Velocidad mínima [m/s]
        
        # =========================
        # TRAYECTORIA
        # =========================
        self.declare_parameter('trajectory_type', 'figure8')
        self.trajectory_type = self.get_parameter('trajectory_type').value
        self.trajectory_waypoints, self.path_x, self.path_y = build_controller_reference_path(
            self.trajectory_type
        )
        self.path_s = np.zeros(len(self.path_x))
        if len(self.path_x) > 1:
            segment_lengths = np.hypot(np.diff(self.path_x), np.diff(self.path_y))
            self.path_s[1:] = np.cumsum(segment_lengths)
        
        # =========================
        # ESTADO DEL ROBOT
        # =========================
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.base_x = 0.0
        self.base_y = 0.0
        self.base_theta = 0.0
        self.v_meas = 0.0
        self.closest_point_idx = 0
        self.pose_initialized = False
        self.closest_point_initialized = False
        
        # =========================
        # NODOS ROS2
        # =========================
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        self.declare_parameter('odom_topic', '/odom_custom')
        self.odom_topic = self.get_parameter('odom_topic').value
        self.declare_parameter('steering_cmd_topic', '/steering_cmd')
        self.steering_cmd_topic = self.get_parameter('steering_cmd_topic').value
        
        # Suscripciones y publicaciones
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, odom_qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.steering_pub = self.create_publisher(Float64MultiArray, self.steering_cmd_topic, 10)
        
        # Timers
        self.timer = self.create_timer(0.05, self.update)  # 20 Hz
        
        # Logging
        self.log = []
        self.debug_count = 0
        self.odom_msg_count = 0
        
        self.get_logger().info(
            f"Pure Pursuit Ackermann CORREGIDO iniciado | "
            f"L={self.L}m | lookahead={self.lookahead_distance}m | "
            f"trayectoria={self.trajectory_type} | "
            f"steering_topic={self.steering_cmd_topic} | "
            f"rear_ratio={self.rear_steering_ratio} | "
            f"heading_offset_deg={np.degrees(self.heading_offset):.1f}"
        )

    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

        steering_msg = Float64MultiArray()
        steering_msg.data = [0.0, 0.0]
        self.steering_pub.publish(steering_msg)

    def find_target_point(self):
        """
        Pure Pursuit CORREGIDO: 
        1. Encuentra el punto MÁS CERCANO al robot
        2. Busca hacia adelante el punto a lookahead_distance
        """
        n_points = len(self.path_x)
        
        # PASO 1: Encontrar el punto más cercano.
        # Al inicio se busca en toda la trayectoria para enganchar el tramo correcto.
        min_dist = float('inf')
        closest_idx = 0

        if not self.closest_point_initialized:
            start_idx = 0
            end_idx = n_points
        else:
            # Luego se usa una ventana local para mantener eficiencia.
            search_window = 200
            start_idx = max(0, self.closest_point_idx - search_window)
            end_idx = min(n_points, self.closest_point_idx + search_window)
        
        for j in range(start_idx, end_idx):
            dx = self.path_x[j] - self.x
            dy = self.path_y[j] - self.y
            dist = math.hypot(dx, dy)
            if dist < min_dist:
                min_dist = dist
                closest_idx = j
        
        # Actualizar punto más cercano
        self.closest_point_idx = closest_idx
        self.closest_point_initialized = True

        # PASO 2: Buscar el target por avance sobre la trayectoria.
        # No filtramos por "punto delante del robot" aquí: si el yaw de odometría
        # viene rotado, ese filtro salta el primer tramo y el robot corta por dentro.
        target_s = self.path_s[self.closest_point_idx] + self.lookahead_distance
        target_idx = int(np.searchsorted(self.path_s, target_s, side="left"))
        target_idx = min(target_idx, n_points - 1)
        
        return self.path_x[target_idx], self.path_y[target_idx], target_idx
    
    def update(self):
        """
        Control Pure Pursuit con mejor manejo de errores
        """
        if not self.pose_initialized:
            self.publish_stop()
            return

        # =========================
        # 1. ENCONTRAR PUNTO OBJETIVO
        # =========================
        tx, ty, target_idx = self.find_target_point()
        
        # =========================
        # 2. CALCULAR ERRORES
        # =========================
        dx = tx - self.x
        dy = ty - self.y
        distance_to_target = math.hypot(dx, dy)
        
        # Si estamos muy cerca del punto objetivo final, detener
        if self.closest_point_idx >= len(self.path_x) - 10 and distance_to_target < 0.15:
            self.publish_stop()
            self.get_logger().info("Objetivo alcanzado - Robot detenido")
            return
        
        # Transformar el target al marco del robot para la geometría clásica
        local_x = math.cos(self.theta) * dx + math.sin(self.theta) * dy
        local_y = -math.sin(self.theta) * dx + math.cos(self.theta) * dy

        # Ángulo deseado hacia el punto objetivo
        theta_d = math.atan2(dy, dx)
        
        # Error angular (normalizado a [-pi, pi])
        ew = math.atan2(math.sin(theta_d - self.theta), 
                        math.cos(theta_d - self.theta))
        
        # =========================
        # 3. CONTROL DE VELOCIDAD LINEAL
        # =========================
        # Velocidad proporcional a la distancia, con mínimo
        v = self.k_velocity * distance_to_target
        v = np.clip(v, self.min_speed, self.max_speed)
        
        # Reducir velocidad si el error angular es grande (curvas cerradas)
        v = v * (1.0 - min(0.7, abs(ew) / (math.pi/2)))
        
        # =========================
        # 4. CONTROL DE DIRECCIÓN ACKERMANN
        # =========================
        # Pure pursuit clásico:
        # delta = atan(2 * L * sin(alpha) / Ld)
        lookahead_geom = max(distance_to_target, self.lookahead_distance)
        alpha = math.atan2(local_y, local_x)
        delta_pp = math.atan2(2.0 * self.L * math.sin(alpha), lookahead_geom)

        # Ganancia fina opcional para ajustar agresividad sin perder la geometría.
        delta_f = self.k_steering * delta_pp
        
        # Limitar ángulo máximo
        delta_f = np.clip(delta_f, -self.delta_max, self.delta_max)
        
        # Ángulo de las ruedas traseras (opuesto y reducido)
        delta_r = self.rear_steering_ratio * delta_f
        delta_r = np.clip(delta_r, -self.delta_max, self.delta_max)
        
        # =========================
        # 5. COMANDOS A TRACCIÓN Y SERVOS
        # =========================
        # Si el ángulo es muy grande, bajar velocidad para no exigir los servos.
        steering_severity = max(abs(delta_f), abs(delta_r)) / self.delta_max
        v = v * (1.0 - 0.5 * min(1.0, steering_severity))
        if distance_to_target < 0.05:
            v = 0.0

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

        steering_msg = Float64MultiArray()
        steering_msg.data = [float(delta_f), float(delta_r)]
        self.steering_pub.publish(steering_msg)
        
        # =========================
        # 6. DEBUG Y LOGGING (Reducido para no saturar)
        # =========================
        self.debug_count += 1
        if self.debug_count % 20 == 0:  # Cada 1 segundo aproximadamente
            # Calcular error a trayectoria (distancia mínima)
            min_error = float('inf')
            for j in range(max(0, self.closest_point_idx-100), 
                          min(len(self.path_x), self.closest_point_idx+100)):
                dx_err = self.path_x[j] - self.x
                dy_err = self.path_y[j] - self.y
                err = math.hypot(dx_err, dy_err)
                if err < min_error:
                    min_error = err
            
            self.get_logger().info(
                f"📍 Pos=({self.x:.2f},{self.y:.2f}) | "
                f"Target=({tx:.2f},{ty:.2f}) | "
                f"idx={self.closest_point_idx}->{target_idx} | "
                f"θ={np.degrees(self.theta):.1f}° | "
                f"local=({local_x:.2f},{local_y:.2f}) | "
                f"Error trayectoria={min_error:.3f}m | "
                f"Err ang={np.degrees(ew):.1f}° | "
                f"v={v:.2f} m/s | "
                f"δ_f={np.degrees(delta_f):.1f}° | "
                f"δ_r={np.degrees(delta_r):.1f}°"
            )
        
        # Guardar datos para análisis (cada 2 iteraciones para no saturar)
        if self.debug_count % 2 == 0:
            # Calcular error mínimo a trayectoria
            min_error = float('inf')
            for j in range(max(0, self.closest_point_idx-100), 
                          min(len(self.path_x), self.closest_point_idx+100)):
                dx_err = self.path_x[j] - self.x
                dy_err = self.path_y[j] - self.y
                err = math.hypot(dx_err, dy_err)
                if err < min_error:
                    min_error = err
                    
            self.log.append([self.x, self.y, min_error, ew, delta_f, delta_r, v])
    
    def odom_callback(self, msg):
        """
        Callback de odometría
        """
        self.odom_msg_count += 1
        self.base_x = msg.pose.pose.position.x
        self.base_y = msg.pose.pose.position.y
        self.v_meas = msg.twist.twist.linear.x
        self.pose_initialized = True
        
        # Extraer orientación (quaternion a yaw)
        q = msg.pose.pose.orientation
        self.base_theta = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )
        self.theta = self.base_theta + self.heading_offset
        self.x = self.base_x
        self.y = self.base_y


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitAckermann()
    
    # Timer para verificar odometría periódicamente
    def check_odom():
        if node.odom_msg_count == 0:
            node.get_logger().warning(f"No se reciben mensajes de odometría en {node.odom_topic}")
        else:
            node.get_logger().info(f"✅ Recibiendo odometría correctamente")
        node.odom_msg_count = 0
    
    odom_check = node.create_timer(5.0, check_odom)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Guardando log...")
    finally:
        if node.log:
            log_array = np.array(node.log)
            np.savetxt(
                "log_pp_corregido.txt",
                log_array,
                header=(
                    "trajectory_type="
                    f"{node.trajectory_type} "
                    "x y error_trayectoria error_ang_rad delta_f_rad delta_r_rad v_cmd_mps"
                ),
                comments="# ",
            )
            
            # Estadísticas
            errors_dist = log_array[:, 2]
            print("=" * 50)
            print("ESTADISTICAS FINALES:")
            print(f"  Error RMS: {np.sqrt(np.mean(errors_dist**2)):.4f} m")
            print(f"  Error maximo: {np.max(errors_dist):.4f} m")
            print(f"  Error promedio: {np.mean(errors_dist):.4f} m")
            print("=" * 50)
        
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
