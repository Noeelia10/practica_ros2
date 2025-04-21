import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from interfaces_pkg.srv import Movimiento


class ServidorMovimiento(Node):

    def __init__(self):
        super().__init__('servidor_movimiento')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.srv = self.create_service(Movimiento, '/mover_robot', self.callback_servicio)
        self.get_logger().info('Servidor /mover_robot listo.')

    def callback_servicio(self, request, response):
        msg = Twist()

        if request.tipo == "lineal":
            msg.linear.x = request.valor
            msg.angular.z = 0.0
            response.message = f"Moviendo lineal a velocidad {request.valor}"
            response.success = True

        elif request.tipo == "rotacional":
            msg.linear.x = 0.0
            msg.angular.z = request.valor
            response.message = f"Rotando a velocidad {request.valor}"
            response.success = True

        else:
            response.message = f"Tipo '{request.tipo}' no reconocido"
            response.success = False
            return response

        self.publisher_.publish(msg)
        return response


def main(args=None):
    rclpy.init(args=args)
    nodo = ServidorMovimiento()
    rclpy.spin(nodo)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
