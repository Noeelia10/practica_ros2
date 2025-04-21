import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from acciones_pkg.action import MoverTriangulo
import math
import time


class TrianguloActionServer(Node):

    def __init__(self):
        super().__init__('servidor_triangulo')
        self._action_server = ActionServer(
            self,
            MoverTriangulo,
            'mover_triangulo',
            self.execute_callback
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Servidor de acción de triángulo listo.')

    def move_for_duration(self, linear, angular, duration):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular

        start = time.time()
        while time.time() - start < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)

        # Detener movimiento
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.1)

    def execute_callback(self, goal_handle):
        lado = goal_handle.request.lado
        self.get_logger().info(f'Recibido objetivo: triángulo de lado {lado:.2f} m')

        # Configuración de velocidades
        velocidad_lineal = 0.1  # m/s
        velocidad_angular = 0.5  # rad/s
        tiempo_recto = lado / velocidad_lineal
        angulo_rad = 2 * math.pi / 3  # 120 grados
        tiempo_giro = angulo_rad / velocidad_angular

        for i in range(3):
            self.get_logger().info(f'Lado {i + 1}/3: Avanzando...')
            self.move_for_duration(velocidad_lineal, 0.0, tiempo_recto)

            self.get_logger().info(f'Lado {i + 1}/3: Girando...')
            self.move_for_duration(0.0, velocidad_angular, tiempo_giro)

            # Feedback
            feedback = MoverTriangulo.Feedback()
            feedback.progreso = (i + 1) / 3.0
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'Progreso: {(feedback.progreso * 100):.1f}%')

        # Finalización
        result = MoverTriangulo.Result()
        result.success = True
        result.mensaje = f'Triángulo completado con lado {lado:.2f} m.'
        self.get_logger().info(result.mensaje)
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    servidor = TrianguloActionServer()
    rclpy.spin(servidor)
    servidor.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



