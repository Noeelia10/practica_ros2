import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MovimientoLineal(Node):
    def __init__(self):
        super().__init__('movimiento_lineal')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.mover_robot)  # Llama cada 0.1s
        self.start_time = self.get_clock().now()
        self.get_logger().info('Nodo iniciado: avanzando 1 metro...')

    def mover_robot(self):
        tiempo_transcurrido = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        msg = Twist()
        if tiempo_transcurrido < 20.0:
            msg.linear.x = 0.05  # Velocidad lineal en x
        else:
            msg.linear.x = 0.0  # Detenerse

        self.publisher_.publish(msg)

        if tiempo_transcurrido >= 20.0:
            self.get_logger().info('Movimiento completado. Nodo detenido.')
            self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    nodo = MovimientoLineal()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
