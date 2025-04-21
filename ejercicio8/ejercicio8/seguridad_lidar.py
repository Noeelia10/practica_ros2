import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SeguridadLidar(Node):

    def __init__(self):
        super().__init__('filtro_seguridad')

        self.umbral_seguridad = 0.4  # metros
        self.rangos_visibles = 10    # muestras de cada lado frontal

        self.lidar = None
        self.cmd_actual = Twist()

        # Suscripciones
        self.create_subscription(LaserScan, '/scan', self.callback_lidar, 10)
        self.create_subscription(Twist, '/cmd_vel', self.callback_cmd, 10)

        # Publicador a cmd_vel
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # Temporizador
        self.create_timer(0.1, self.controlar_movimiento)

        self.get_logger().info('Nodo de seguridad LIDAR activado.')

    def callback_lidar(self, msg):
        self.lidar = msg

    def callback_cmd(self, msg):
        self.cmd_actual = msg

    def controlar_movimiento(self):
        if self.lidar is None:
            return

        ranges = self.lidar.ranges
        frontal_izq = ranges[:self.rangos_visibles]
        frontal_der = ranges[-self.rangos_visibles:]
        frontal = [r for r in (frontal_izq + frontal_der) if 0.0 < r < float('inf')]

        peligro = frontal and min(frontal) < self.umbral_seguridad

        if peligro and self.cmd_actual.linear.x > 0:
            stop = Twist()
            self.pub_cmd.publish(stop)
            self.get_logger().warn(f"⚠️ Obstáculo a {min(frontal):.2f} m. Movimiento cancelado.")
        else:
            self.pub_cmd.publish(self.cmd_actual)
            self.get_logger().info(f"✅ Movimiento permitido: avance {self.cmd_actual.linear.x:.2f} m/s")


def main(args=None):
    rclpy.init(args=args)
    nodo = SeguridadLidar()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
