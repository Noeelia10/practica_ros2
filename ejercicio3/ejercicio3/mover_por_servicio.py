import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from math import sqrt


class MovimientoServicio(Node):
    def __init__(self):
        super().__init__('movimiento_por_servicio')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.srv = self.create_service(Trigger, 'mover_robot', self.callback_servicio)

        self.inicio = None
        self.mover = False
        self.get_logger().info('Nodo listo. Esperando llamadas al servicio...')

    def callback_servicio(self, request, response):
        if self.mover:
            response.success = False
            response.message = 'El robot ya está en movimiento.'
        else:
            self.inicio = None
            self.mover = True
            response.success = True
            response.message = 'Movimiento iniciado.'
            self.get_logger().info('Servicio recibido: ¡moviendo 1 metro!')

        return response

    def odom_callback(self, msg):
        if not self.mover:
            return

        posicion_actual = msg.pose.pose.position

        if self.inicio is None:
            self.inicio = posicion_actual
            return

        dx = posicion_actual.x - self.inicio.x
        dy = posicion_actual.y - self.inicio.y
        distancia = sqrt(dx**2 + dy**2)

        if distancia < 1.0:
            self.enviar_velocidad(0.05)
        else:
            self.enviar_velocidad(0.0)
            self.mover = False
            self.get_logger().info('Movimiento completo. Esperando otra llamada.')

    def enviar_velocidad(self, velocidad):
        msg = Twist()
        msg.linear.x = velocidad
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    nodo = MovimientoServicio()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
