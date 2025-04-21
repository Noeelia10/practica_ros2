import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from acciones_pkg.action import DibujarPoligono

class ClientePoligono(Node):

    def __init__(self):
        super().__init__('cliente_poligono')
        self._action_client = ActionClient(self, DibujarPoligono, 'dibujar_poligono')

    def send_goal(self, n_lados, longitud):
        goal_msg = DibujarPoligono.Goal()
        goal_msg.n_lados = n_lados
        goal_msg.longitud = longitud

        self._action_client.wait_for_server()
        self.get_logger().info(f'Enviando: {n_lados} lados, longitud {longitud:.2f} m')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Objetivo rechazado')
            return

        self.get_logger().info('Objetivo aceptado, esperando resultado...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Progreso: {(feedback.progreso * 100):.1f}%')

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Resultado: {result.mensaje} (Éxito: {result.success})')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    cliente = ClientePoligono()

    n_lados = int(input('Número de lados (>=3): '))
    longitud = float(input('Longitud de cada lado (m): '))
    cliente.send_goal(n_lados, longitud)

    rclpy.spin(cliente)


if __name__ == '__main__':
    main()
