import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, fabs
from interfaces_pkg.srv import Movimiento

class MovimientoConfigurable(Node):
    def __init__(self):
        super().__init__('movimiento_configurable')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.srv = self.create_service(Movimiento, 'mover_robot', self.callback_servicio)

        self.tipo = None
        self.valor = 0.0
        self.inicio = None
        self.en_movimiento = False
        self.get_logger().info('Nodo iniciado. Esperando comandos...')

    def callback_servicio(self, request, response):
        if self.en_movimiento:
            response.success = False
            response.message = 'Ya en movimiento.'
        else:
            self.tipo = request.tipo
            self.valor = request.valor
            self.inicio = None
            self.en_movimiento = True
            response.success = True
            response.message = f'Movimiento {self.tipo} iniciado con valor {self.valor}'
            self.get_logger().info(response.message)

        return response

    def odom_callback(self, msg):
        if not self.en_movimiento:
            return

        if self.inicio is None:
            self.inicio = msg.pose.pose
            return

        if self.tipo == 'lineal':
            dx = msg.pose.pose.position.x - self.inicio.position.x
            dy = msg.pose.pose.position.y - self.inicio.position.y
            distancia = sqrt(dx**2 + dy**2)

            if distancia < self.valor:
                self.publicar_velocidad(lineal=0.05)
            else:
                self.publicar_velocidad(lineal=0.0)
                self.en_movimiento = False
                self.get_logger().info('Movimiento lineal completo.')

        elif self.tipo == 'rotacional':
            ang_actual = self.get_yaw(msg.pose.pose.orientation)
            ang_inicio = self.get_yaw(self.inicio.orientation)
            delta_ang = fabs(ang_actual - ang_inicio)

            if delta_ang < self.valor:
                self.publicar_velocidad(angular=0.3)
            else:
                self.publicar_velocidad(angular=0.0)
                self.en_movimiento = False
                self.get_logger().info('Rotación completa.')

    def publicar_velocidad(self, lineal=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = lineal
        msg.angular.z = angular
        self.publisher_.publish(msg)

    def get_yaw(self, orientation):
        import tf_transformations
        quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(quat)
        return yaw


def main(args=None):
    rclpy.init(args=args)
    nodo = MovimientoConfigurable()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
