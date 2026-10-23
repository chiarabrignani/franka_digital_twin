import csv
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class FullSequenceTest(Node):

    def __init__(self):
        super().__init__('full_sequence_test')

        self.joint_names = [
            'fr3_joint1',
            'fr3_joint2',
            'fr3_joint3',
            'fr3_joint4',
            'fr3_joint5',
            'fr3_joint6',
            'fr3_joint7',
        ]

        # Limiti articolari utilizzati per il controllo preliminare.
        self.limits = {
            'fr3_joint1': (-2.8973, 2.8973),
            'fr3_joint2': (-1.7628, 1.7628),
            'fr3_joint3': (-2.8973, 2.8973),
            'fr3_joint4': (-3.0718, -0.0698),
            'fr3_joint5': (-2.8973, 2.8973),
            'fr3_joint6': (-0.0175, 3.7525),
            'fr3_joint7': (-2.8973, 2.8973),
        }

        # Tre configurazioni rispetto alla posizione iniziale.
        # Ogni comando coinvolge simultaneamente tutti i giunti.
        self.offset_sequences = [
            [0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02],
            [-0.02, -0.02, -0.02, -0.02, -0.02, -0.02, -0.02],
            [0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01],
        ]

        self.tolerance = 0.01
        self.timeout = 10.0
        self.stable_duration = 0.5

        self.robot_positions = None
        self.digital_positions = None
        self.initial_positions = None
        self.targets = []
        self.results = []

        self.step_index = -1
        self.step_start_time = None
        self.stable_since = None
        self.finished = False
        self.failed = False

        self.robot_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.robot_state_callback,
            10,
        )

        self.digital_subscription = self.create_subscription(
            JointState,
            '/digital_twin/joint_states',
            self.digital_state_callback,
            10,
        )

        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/digital_twin/command_positions',
            10,
        )

        self.timer = self.create_timer(0.05, self.test_step)

        self.get_logger().info(
            'Full Sequence Test avviato: attesa degli stati '
            'del robot e del Digital Twin.'
        )

    def extract_positions(self, msg):
        if len(msg.name) != len(msg.position):
            return None

        received = dict(zip(msg.name, msg.position))

        if not all(name in received for name in self.joint_names):
            return None

        positions = {
            name: received[name] for name in self.joint_names
        }

        if not all(math.isfinite(value) for value in positions.values()):
            return None

        return positions

    def robot_state_callback(self, msg):
        positions = self.extract_positions(msg)
        if positions is not None:
            self.robot_positions = positions

    def digital_state_callback(self, msg):
        positions = self.extract_positions(msg)
        if positions is not None:
            self.digital_positions = positions

    def test_step(self):
        if self.finished:
            return

        # Prima di iniziare devono essere disponibili entrambe
        # le rappresentazioni dello stato.
        if self.initial_positions is None:
            if (
                self.robot_positions is None
                or self.digital_positions is None
            ):
                return

            self.initial_positions = dict(self.robot_positions)

            # Verifica che il Digital Twin sia allineato al robot
            # prima di inviare nuovi comandi.
            initial_errors = {
                name: abs(
                    self.robot_positions[name]
                    - self.digital_positions[name]
                )
                for name in self.joint_names
            }

            if any(
                error > self.tolerance
                for error in initial_errors.values()
            ):
                self.get_logger().error(
                    'Test annullato: lo stato iniziale del Digital Twin '
                    'non coincide con quello del robot entro la tolleranza.'
                )
                self.finish()
                return

            for offsets in self.offset_sequences:
                target = {
                    name: self.initial_positions[name] + offset
                    for name, offset in zip(self.joint_names, offsets)
                }

                for name, value in target.items():
                    lower, upper = self.limits[name]
                    if not lower <= value <= upper:
                        self.get_logger().error(
                            f'Test annullato: target fuori dai limiti '
                            f'di {name}: {value:.4f} rad.'
                        )
                        self.finish()
                        return

                self.targets.append(target)

            self.get_logger().info(
                'Stato iniziale valido. Avvio della sequenza '
                'di tre movimenti.'
            )
            self.start_next_step()
            return

        if self.step_index < 0 or self.step_index >= len(self.targets):
            return

        now = time.monotonic()
        target = self.targets[self.step_index]
        elapsed = now - self.step_start_time

        robot_errors = {
            name: abs(target[name] - self.robot_positions[name])
            for name in self.joint_names
        }

        digital_errors = {
            name: abs(target[name] - self.digital_positions[name])
            for name in self.joint_names
        }

        robot_reached = all(
            error <= self.tolerance for error in robot_errors.values()
        )

        digital_reached = all(
            error <= self.tolerance for error in digital_errors.values()
        )

        # Confronta anche direttamente robot e Digital Twin.
        states_aligned = all(
            abs(
                self.robot_positions[name]
                - self.digital_positions[name]
            ) <= self.tolerance
            for name in self.joint_names
        )

        if robot_reached and digital_reached and states_aligned:
            if self.stable_since is None:
                self.stable_since = now
            elif now - self.stable_since >= self.stable_duration:
                self.save_step_result(
                    success=True,
                    elapsed=now - self.step_start_time,
                    robot_errors=robot_errors,
                    digital_errors=digital_errors,
                )

                self.get_logger().info(
                    f'Movimento {self.step_index + 1}/'
                    f'{len(self.targets)} SUPERATO.'
                )

                if self.step_index + 1 == len(self.targets):
                    self.finish_test()
                else:
                    self.start_next_step()

                return
        else:
            self.stable_since = None

        if elapsed >= self.timeout:
            self.save_step_result(
                success=False,
                elapsed=elapsed,
                robot_errors=robot_errors,
                digital_errors=digital_errors,
            )

            self.get_logger().error(
                f'Movimento {self.step_index + 1} FALLITO: '
                f'timeout dopo {elapsed:.2f} secondi.'
            )
            self.failed = True
            self.finish_test()

    def start_next_step(self):
        self.step_index += 1
        self.stable_since = None
        self.step_start_time = time.monotonic()

        command = Float64MultiArray()
        command.data = [
            float(self.targets[self.step_index][name])
            for name in self.joint_names
        ]

        self.command_publisher.publish(command)

        self.get_logger().info(
            f'Comando {self.step_index + 1}/{len(self.targets)} inviato.'
        )

        for name, value in zip(self.joint_names, command.data):
            self.get_logger().info(
                f'{name}: target={value:.4f} rad'
            )

    def save_step_result(
        self, success, elapsed, robot_errors, digital_errors
    ):
        target = self.targets[self.step_index]

        for name in self.joint_names:
            self.results.append({
                'step': self.step_index + 1,
                'joint': name,
                'initial_position_rad': self.initial_positions[name],
                'target_position_rad': target[name],
                'robot_position_rad': self.robot_positions[name],
                'digital_position_rad': self.digital_positions[name],
                'robot_error_rad': robot_errors[name],
                'digital_error_rad': digital_errors[name],
                'states_aligned': abs(
                    self.robot_positions[name]
                    - self.digital_positions[name]
                ) <= self.tolerance,
                'test_success': success,
                'elapsed_time_s': elapsed,
            })

    def finish_test(self):
        self.save_csv()
        self.finish()

    def save_csv(self):
        results_dir = Path.home() / 'franka_digital_twin' / 'results'
        results_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_file = results_dir / f'full_sequence_test_{timestamp}.csv'

        fields = [
            'step',
            'joint',
            'initial_position_rad',
            'target_position_rad',
            'robot_position_rad',
            'digital_position_rad',
            'robot_error_rad',
            'digital_error_rad',
            'states_aligned',
            'test_success',
            'elapsed_time_s',
        ]

        with output_file.open(
            'w', newline='', encoding='utf-8'
        ) as file:
            writer = csv.DictWriter(file, fieldnames=fields)
            writer.writeheader()

            for row in self.results:
                formatted = dict(row)
                for field in fields:
                    if field.endswith('_rad') or field == 'elapsed_time_s':
                        formatted[field] = f'{row[field]:.6f}'
                writer.writerow(formatted)

        self.get_logger().info(f'Risultati salvati in: {output_file}')

    def finish(self):
        self.finished = True
        self.timer.cancel()

        self.get_logger().info(
            'Test complessivo: '
            f'{"FALLITO" if self.failed else "COMPLETATO"}.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FullSequenceTest()

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
