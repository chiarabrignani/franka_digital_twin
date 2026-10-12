import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class CommandManager(Node):

    def __init__(self):
        super().__init__('command_manager')

        # Nomi dei sette giunti del braccio FR3
        self.joint_names = [
            'fr3_joint1',
            'fr3_joint2',
            'fr3_joint3',
            'fr3_joint4',
            'fr3_joint5',
            'fr3_joint6',
            'fr3_joint7'
        ]

        # Publisher verso il controller di posizione
        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/fr3_position_controller/commands',
            10
        )

        # Subscriber ai comandi provenienti dal Digital Twin
        self.command_subscription = self.create_subscription(
            Float64MultiArray,
            '/digital_twin/command_positions',
            self.command_callback,
            10
        )

        self.get_logger().info(
            'Command Manager avviato. '
            'In attesa di comandi su /digital_twin/command_positions.'
        )

    def command_callback(self, msg):

        # Verifica che il vettore contenga sette posizioni
        if len(msg.data) != len(self.joint_names):
            self.get_logger().error(
                'Comando rifiutato: sono richieste '
                'esattamente sette posizioni articolari.'
            )
            return

        # Verifica che tutti i valori siano numerici e finiti
        if not all(
            math.isfinite(value) for value in msg.data
        ):
            self.get_logger().error(
                'Comando rifiutato: il vettore contiene '
                'valori non finiti.'
            )
            return

        # Crea il messaggio destinato al controller
        command = Float64MultiArray()
        command.data = list(msg.data)

        # Pubblica il comando
        self.command_publisher.publish(command)

        self.get_logger().info(
            'Comando di posizione inviato al controller FR3.'
        )

        for joint, position in zip(
            self.joint_names,
            command.data
        ):
            self.get_logger().info(
                f'{joint}: posizione obiettivo = '
                f'{position:.6f} rad'
            )


def main(args=None):

    rclpy.init(args=args)

    node = CommandManager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
