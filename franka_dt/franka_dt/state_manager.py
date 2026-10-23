import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class StateManager(Node):

    def __init__(self):
        super().__init__('state_manager')

        self.joint_names = [
            'fr3_joint1',
            'fr3_joint2',
            'fr3_joint3',
            'fr3_joint4',
            'fr3_joint5',
            'fr3_joint6',
            'fr3_joint7',
        ]

        self.digital_state = None
        self.source_timestamp = None
        self.receipt_timestamp = None

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
        )

        # Pubblica lo stato acquisito dal robot simulato.
        self.state_publisher = self.create_publisher(
            JointState,
            '/digital_twin/joint_states',
            10,
        )

        self.timer = self.create_timer(
            1.0,
            self.print_digital_state,
        )

        self.get_logger().info(
            'State Manager avviato. In attesa di /joint_states...'
        )

        self.get_logger().info(
            'Pubblicazione dello stato digitale su '
            '/digital_twin/joint_states.'
        )

    def joint_state_callback(self, msg):
        # Verifica che nomi e posizioni siano coerenti.
        if len(msg.name) != len(msg.position):
            self.get_logger().warning(
                'Messaggio ignorato: nomi e posizioni hanno '
                'lunghezze differenti.'
            )
            return

        positions = dict(zip(msg.name, msg.position))

        if not all(joint in positions for joint in self.joint_names):
            self.get_logger().warning(
                'Messaggio ricevuto senza tutti i sette giunti.'
            )
            return

        # Rifiuta posizioni non valide.
        if not all(
            math.isfinite(positions[joint])
            for joint in self.joint_names
        ):
            self.get_logger().warning(
                'Messaggio ignorato: contiene posizioni non finite.'
            )
            return

        velocities = dict(zip(msg.name, msg.velocity))
        efforts = dict(zip(msg.name, msg.effort))

        # Aggiorna la rappresentazione digitale interna.
        self.digital_state = {}

        for joint in self.joint_names:
            self.digital_state[joint] = {
                'position': positions[joint],
                'velocity': velocities.get(joint),
            }

        self.source_timestamp = (
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
        )
        self.receipt_timestamp = self.get_clock().now()

        # Pubblica esclusivamente i sette giunti del braccio,
        # ordinati in modo deterministico.
        digital_msg = JointState()
        digital_msg.header = msg.header
        digital_msg.name = list(self.joint_names)
        digital_msg.position = [
            float(positions[joint]) for joint in self.joint_names
        ]

        if all(joint in velocities for joint in self.joint_names):
            digital_msg.velocity = [
                float(velocities[joint]) for joint in self.joint_names
            ]

        if all(joint in efforts for joint in self.joint_names):
            digital_msg.effort = [
                float(efforts[joint]) for joint in self.joint_names
            ]

        self.state_publisher.publish(digital_msg)

    def print_digital_state(self):
        if self.digital_state is None:
            return

        self.get_logger().info('--- Stato digitale FR3 ---')

        for joint, values in self.digital_state.items():
            position = values['position']
            velocity = values['velocity']

            velocity_text = (
                'non disponibile'
                if velocity is None
                else f'{velocity:.4f} rad/s'
            )

            self.get_logger().info(
                f'{joint}: posizione={position:.4f} rad, '
                f'velocita={velocity_text}'
            )

        self.get_logger().info(
            f'Timestamp sorgente: {self.source_timestamp}'
        )
        self.get_logger().info(
            f'Ultima ricezione: {self.receipt_timestamp.nanoseconds} ns'
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
