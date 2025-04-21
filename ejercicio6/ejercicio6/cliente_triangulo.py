import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from acciones_pkg.action import MoverTriangulo


class TrianguloActionClient(Node):

    def __init__(self):
        super().__init__('cliente_triangulo')
        self._action_client = ActionClient(self, MoverTriangulo, 'mover_triangulo')

    def send_goal(self, lado):
        goal_msg = MoverTriangulo.Goal()
        goal_msg.lado = lado

        self._action_client.wait_for_server()
        self.get_logger().info(f'Enviando solicitud para triángulo con lado = {lado:.2f} m')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Objetivo rechazado por el servidor.')
            return

        self.get_logger().info('Objetivo aceptado. Ejecutando...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Progreso: {feedback.progreso * 100:.1f}%')

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Resultado: {result.mensaje} (Éxito: {result.success})')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    cliente = TrianguloActionClient()

    try:
        lado = float(input("Introduce la longitud del lado del triángulo (en metros): "))
    except ValueError:
        print("Por favor, introduce un valor numérico válido.")
        return

    cliente.send_goal(lado)
    rclpy.spin(cliente)

