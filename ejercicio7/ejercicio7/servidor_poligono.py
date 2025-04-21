import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from acciones_pkg.action import DibujarPoligono
import math
import time

class ServidorPoligono(Node):

    def __init__(self):
        super().__init__('servidor_poligono')
        self._action_server = ActionServer(
            self,
            DibujarPoligono,
            'dibujar_poligono',
            self.execute_callback
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Servidor de acción "dibujar_poligono" listo.')

    def move_for_duration(self, linear, angular, duration):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular

        start = time.time()
        while time.time() - start < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)

        # Parar
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.1)

    def execute_callback(self, goal_handle):
        n = goal_handle.request.n_lados
        lado = goal_handle.request.longitud

        if n < 3 or lado <= 0.0:
            self.get_logger().warn('Parámetros inválidos')
            goal_handle.abort()
            result = DibujarPoligono.Result()
            result.success = False
            result.mensaje = 'Número de lados debe ser >= 3 y longitud > 0.'
            return result

        self.get_logger().info(f'Recibido objetivo: {n} lados de {lado:.2f} m')

        vel_lineal = 0.1
        vel_angular = 0.5

        if n >= 11:
            # === MODO CÍRCULO ===
            self.get_logger().info('→ Entrando en modo CÍRCULO')
            radio = lado
            duracion = (2 * math.pi * radio) / vel_lineal  # Tiempo total para completar el círculo

            twist = Twist()
            twist.linear.x = vel_lineal
            twist.angular.z = vel_lineal / radio

            start = time.time()
            while time.time() - start < duracion:
                self.cmd_vel_pub.publish(twist)

                feedback = DibujarPoligono.Feedback()
                feedback.progreso = (time.time() - start) / duracion
                goal_handle.publish_feedback(feedback)

                time.sleep(0.1)

            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

        else:
            # === MODO POLÍGONO REGULAR ===
            self.get_logger().info('→ Entrando en modo POLÍGONO')
            angulo_rad = (2 * math.pi) / n
            tiempo_recto = lado / vel_lineal
            tiempo_giro = angulo_rad / vel_angular

            for i in range(n):
                self.get_logger().info(f'Lado {i+1}/{n}: avanzando...')
                self.move_for_duration(vel_lineal, 0.0, tiempo_recto)

                self.get_logger().info(f'Lado {i+1}/{n}: girando {math.degrees(angulo_rad):.1f} grados...')
                self.move_for_duration(0.0, vel_angular, tiempo_giro)

                feedback = DibujarPoligono.Feedback()
                feedback.progreso = (i + 1) / n
                goal_handle.publish_feedback(feedback)

        # RESULTADO FINAL
        result = DibujarPoligono.Result()
        result.success = True
        result.mensaje = f'Figura completada: {"círculo" if n >= 11 else f"{n} lados"}.'
        goal_handle.succeed()
        self.get_logger().info(result.mensaje)
        return result


def main(args=None):
    rclpy.init(args=args)
    servidor = ServidorPoligono()
    rclpy.spin(servidor)
    servidor.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


