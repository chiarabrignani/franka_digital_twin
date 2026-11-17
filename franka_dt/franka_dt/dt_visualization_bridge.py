import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class DTVisualizationBridge(Node):

    def __init__(self):
        super().__init__('dt_visualization_bridge')

        self.finger_position = None
        self.finger_velocity = 0.0

        # Stati autonomi del braccio digitale
        self.dt_subscription = self.create_subscription(
            JointState,
            '/digital_twin/predicted_joint_states',
            self.dt_callback,
            10
        )

        # Stato della pinza del Physical Twin
        self.pt_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.pt_callback,
            10
        )

        self.publisher = self.create_publisher(
            JointState,
            '/digital_twin/visualization_joint_states',
            10
        )

        self.get_logger().info(
            'DT Visualization Bridge avviato.'
        )

    def pt_callback(self, msg):

        # Cerchiamo il giunto della pinza fisica
        if 'fr3_finger_joint1' not in msg.name:
            return

        index = msg.name.index('fr3_finger_joint1')

        self.finger_position = msg.position[index]

        if len(msg.velocity) > index:
            self.finger_velocity = msg.velocity[index]

    def dt_callback(self, msg):

        # Aspettiamo di ricevere il primo dato della pinza fisica
        if self.finger_position is None:
            return

        visual_msg = JointState()

        visual_msg.header = msg.header

        # Giunti del braccio digitale
        visual_msg.name = [
            'dt_' + name if name.startswith('fr3_joint') else name
            for name in msg.name
        ]

        visual_msg.position = list(msg.position)
        visual_msg.velocity = list(msg.velocity)
        visual_msg.effort = list(msg.effort)

        # Giunto principale della pinza digitale
        visual_msg.name.append('dt_fr3_finger_joint1')

        visual_msg.position.append(self.finger_position)
        visual_msg.velocity.append(self.finger_velocity)

        self.publisher.publish(visual_msg)


def main(args=None):

    rclpy.init(args=args)

    node = DTVisualizationBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
