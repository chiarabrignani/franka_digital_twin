import csv
import os
import time
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateRecorder(Node):
    def __init__(self):
        super().__init__('joint_state_recorder')

        self.output_file = '/tmp/fr3_joint1_capture.csv'
        self.joint_name = 'fr3_joint1'

        self.file = open(self.output_file, 'w', newline='')
        self.writer = csv.writer(self.file)

        self.writer.writerow([
            'message_index',
            'header_stamp_seconds',
            'receive_time_unix',
            'receive_time_monotonic',
            'joint_name',
            'position_rad',
            'velocity_rad_s'
        ])

        self.file.flush()
        self.message_count = 0

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.callback,
            100
        )

        self.get_logger().info('Registrazione avviata.')
        self.get_logger().info(f'File: {self.output_file}')

    def callback(self, msg):
        try:
            i = msg.name.index(self.joint_name)
        except ValueError:
            return

        if i >= len(msg.position) or i >= len(msg.velocity):
            return

        stamp = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

        receive_unix = time.time()
        receive_monotonic = time.monotonic()

        position = msg.position[i]
        velocity = msg.velocity[i]

        self.message_count += 1

        self.writer.writerow([
            self.message_count,
            f'{stamp:.9f}',
            f'{receive_unix:.9f}',
            f'{receive_monotonic:.9f}',
            self.joint_name,
            f'{position:.12f}',
            f'{velocity:.12f}'
        ])

        self.file.flush()

    def destroy_node(self):
        self.file.flush()
        self.file.close()

        self.get_logger().info(
            f'Registrazione terminata. '
            f'Messaggi acquisiti: {self.message_count}'
        )
        self.get_logger().info(
            f'File salvato in: {self.output_file}'
        )

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JointStateRecorder()

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
