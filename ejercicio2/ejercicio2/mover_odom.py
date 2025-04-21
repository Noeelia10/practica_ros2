import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt


class MovimientoOdometrico(Node):
    def __init__(self):
        super().__init__('movimiento_odom')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.inicio = None
        self.en_movimiento = True
        self.get_logger().info('Nodo iniciado. Avanzando hasta recorrer 1 metro...')

    def odom_callback(self, msg):
        posicion_actual = msg.pose.pose.position

        if self.inicio is None:
            self.inicio = posicion_actual
            return

        dx = posicion_actual.x - self.inicio.x
        dy = posicion_actual.y - self.inicio.y
        distancia = sqrt(dx**2 + dy**2)

        if distancia < 1.0:
            self.mover()
        elif self.en_movimiento:
            self.detener()
            self.get_logger().info('Distancia alcanzada. Robot detenido.')

    def mover(self):
        msg = Twist()
        msg.linear.x = 0.05
        self.publisher_.publish(msg)

    def detener(self):
        msg = Twist()
        msg.linear.x = 0.0
        self.publisher_.publish(msg)
        self.en_movimiento = False


def main(args=None):
    rclpy.init(args=args)
    nodo = MovimientoOdometrico()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
