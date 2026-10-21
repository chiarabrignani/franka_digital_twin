import csv
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class TrackingTest(Node):

    def __init__(self):
        super().__init__('tracking_test')

        self.joint_names = [
            'fr3_joint1',
            'fr3_joint2',
            'fr3_joint3',
            'fr3_joint4',
            'fr3_joint5',
            'fr3_joint6',
            'fr3_joint7',
        ]

        # Piccoli spostamenti simultanei dei giunti 2, 4 e 7.
        self.offsets = {
            'fr3_joint2': 0.03,
            'fr3_joint4': 0.03,
            'fr3_joint7': -0.03,
        }

        self.tolerance = 0.01
        self.timeout = 10.0
        self.stable_duration = 0.5

        self.positions = {}
        self.initial_positions = None
        self.target_positions = None
        self.command_sent = False
        self.start_time = None
        self.stable_since = None
        self.finished = False
        self.max_errors = {}

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
        )
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/digital_twin/command_positions',
            10,
        )
        self.timer = self.create_timer(0.05, self.test_step)

        self.get_logger().info(
            'Tracking Test avviato: in attesa di sette stati articolari validi.'
        )

    def joint_state_callback(self, msg):
        if len(msg.name) != len(msg.position):
            return

        received = dict(zip(msg.name, msg.position))

        if not all(name in received for name in self.joint_names):
            return

        values = [received[name] for name in self.joint_names]
        if not all(math.isfinite(value) for value in values):
            return

        self.positions = {
            name: received[name] for name in self.joint_names
        }

    def test_step(self):
        if self.finished:
            return

        now = time.monotonic()

        if self.initial_positions is None:
            if len(self.positions) != len(self.joint_names):
                return

            self.initial_positions = dict(self.positions)
            self.target_positions = dict(self.initial_positions)

            for name, offset in self.offsets.items():
                self.target_positions[name] += offset

            # Controlli preliminari sui limiti noti dei giunti selezionati.
            limits = {
                'fr3_joint2': (-1.7628, 1.7628),
                'fr3_joint4': (-3.0718, -0.0698),
                'fr3_joint7': (-2.9671, 2.9671),
            }
            for name, (lower, upper) in limits.items():
                target = self.target_positions[name]
                if not lower <= target <= upper:
                    self.get_logger().error(
                        f'Test annullato: target fuori dai limiti '
                        f'impostati per {name}: {target:.4f} rad.'
                    )
                    self.finish()
                    return

            command = Float64MultiArray()
            command.data = [
                float(self.target_positions[name])
                for name in self.joint_names
            ]

            self.start_time = now
            self.publisher.publish(command)
            self.command_sent = True

            self.get_logger().info(
                'Comando inviato. Verifica del raggiungimento dei target...'
            )
            return

        elapsed = now - self.start_time
        errors = {
            name: abs(
                self.target_positions[name] - self.positions[name]
            )
            for name in self.offsets
        }

        for name, error in errors.items():
            self.max_errors[name] = max(
                self.max_errors.get(name, 0.0), error
            )

        all_reached = all(
            error <= self.tolerance for error in errors.values()
        )

        if all_reached:
            if self.stable_since is None:
                self.stable_since = now
            elif now - self.stable_since >= self.stable_duration:
                self.save_results(
                    success=True,
                    elapsed=now - self.start_time,
                )
                self.finish()
                return
        else:
            self.stable_since = None

        if elapsed >= self.timeout:
            self.get_logger().error(
                f'Test fallito: timeout dopo {elapsed:.2f} secondi.'
            )
            self.save_results(success=False, elapsed=elapsed)
            self.finish()

    def save_results(self, success, elapsed):
        results_dir = Path.home() / 'franka_digital_twin' / 'results'
        results_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_file = results_dir / f'tracking_test_{timestamp}.csv'

        with output_file.open('w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow([
                'joint',
                'initial_position_rad',
                'target_position_rad',
                'final_position_rad',
                'absolute_error_rad',
                'max_observed_error_rad',
                'test_success',
                'elapsed_time_s',
            ])

            for name in self.offsets:
                final = self.positions.get(name, float('nan'))
                target = self.target_positions[name]
                error = abs(target - final) if math.isfinite(final) else float('nan')

                writer.writerow([
                    name,
                    f'{self.initial_positions[name]:.6f}',
                    f'{target:.6f}',
                    f'{final:.6f}',
                    f'{error:.6f}',
                    f'{self.max_errors.get(name, float("nan")):.6f}',
                    success,
                    f'{elapsed:.3f}',
                ])

        self.get_logger().info(f'Risultati salvati in: {output_file}')
        self.get_logger().info(
            f'Esito: {"SUPERATO" if success else "FALLITO"}; '
            f'tempo: {elapsed:.3f} s.'
        )

    def finish(self):
        self.finished = True
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = TrackingTest()

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
