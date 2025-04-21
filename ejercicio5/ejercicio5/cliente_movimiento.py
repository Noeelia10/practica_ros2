import rclpy
from rclpy.node import Node
from interfaces_pkg.srv import Movimiento


class ClienteMovimiento(Node):

    def __init__(self):
        super().__init__('cliente_movimiento')
        self.cli = self.create_client(Movimiento, '/mover_robot')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio /mover_robot...')

        self.solicitar_datos()

    def solicitar_datos(self):
        tipo = input("Introduce el tipo de movimiento (lineal/rotacional): ")
        valor = float(input("Introduce el valor del movimiento: "))

        request = Movimiento.Request()
        request.tipo = tipo
        request.valor = valor

        future = self.cli.call_async(request)
        future.add_done_callback(self.respuesta_servicio)

    def respuesta_servicio(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Respuesta: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Error al llamar al servicio: {e}")


def main(args=None):
    rclpy.init(args=args)
    cliente = ClienteMovimiento()
    rclpy.spin(cliente)
    cliente.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
