import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class StateManager(Node):

    def __init__(self):
        super().__init__('state_manager')

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

        # Stato digitale inizialmente non disponibile
        self.digital_state = None

        # Informazioni temporali dell'ultimo aggiornamento
        self.source_timestamp = None
        self.receipt_timestamp = None

        # Sottoscrizione al topic dello stato del robot
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Stampa lo stato digitale una volta al secondo
        self.timer = self.create_timer(
            1.0,
            self.print_digital_state
        )

        self.get_logger().info(
            'State Manager avviato. In attesa di /joint_states...'
        )

    def joint_state_callback(self, msg):

        # Associa ogni nome di giunto alla posizione ricevuta
        positions = dict(zip(msg.name, msg.position))

        # Verifica che siano disponibili tutti i sette giunti
        if not all(
            joint in positions for joint in self.joint_names
        ):
            self.get_logger().warning(
                'Messaggio ricevuto senza tutti i sette giunti.'
            )
            return

        # Associa ogni giunto alla sua velocità, se disponibile
        velocities = dict(zip(msg.name, msg.velocity))

        # Aggiorna la rappresentazione digitale
        self.digital_state = {}

        for joint in self.joint_names:
            self.digital_state[joint] = {
                'position': positions[joint],
                'velocity': velocities.get(joint)
            }

        # Conserva separatamente i riferimenti temporali
        self.source_timestamp = (
            msg.header.stamp.sec,
            msg.header.stamp.nanosec
        )

        self.receipt_timestamp = self.get_clock().now()

    def print_digital_state(self):

        # Non stampa nulla finché non riceve uno stato valido
        if self.digital_state is None:
            return

        self.get_logger().info('--- Stato digitale FR3 ---')

        for joint, values in self.digital_state.items():

            position = values['position']
            velocity = values['velocity']

            if velocity is None:
                velocity_text = 'non disponibile'
            else:
                velocity_text = f'{velocity:.4f} rad/s'

            self.get_logger().info(
                f'{joint}: '
                f'posizione={position:.4f} rad, '
                f'velocita={velocity_text}'
            )

        self.get_logger().info(
            f'Timestamp sorgente: {self.source_timestamp}'
        )

        self.get_logger().info(
            f'Ultima ricezione: {self.receipt_timestamp.nanoseconds}'
            ' ns'
        )


def main(args=None):

    rclpy.init(args=args)

    node = StateManager()

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
