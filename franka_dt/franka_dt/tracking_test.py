import csv
from datetime import datetime
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from franka_dt_interfaces.srv import GenerateCommand


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

        # Giunti sui quali valutare il tracking.
        self.monitored_joints = [
            'fr3_joint2',
            'fr3_joint4',
            'fr3_joint7',
        ]

        self.tolerance = 0.01
        self.timeout = 10.0
        self.stable_duration = 0.5

        self.positions = {}
        self.initial_positions = None
        self.target_positions = None

        self.command_requested = False
        self.command_received = False

        self.start_time = None
        self.stable_since = None

        self.finished = False
        self.max_errors = {}

        # Subscriber agli stati articolari del robot fisico.
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
        )

        # Client del servizio che genera i comandi.
        self.command_client = self.create_client(
            GenerateCommand,
            '/digital_twin/generate_command',
        )

        self.timer = self.create_timer(
            0.05,
            self.test_step,
        )

        self.get_logger().info(
            'Tracking Test avviato: '
            'in attesa di sette stati articolari validi.'
        )

    def joint_state_callback(self, msg):

        if len(msg.name) != len(msg.position):
            return

        if len(set(msg.name)) != len(msg.name):
            return

        received = dict(zip(msg.name, msg.position))

        if not all(
            name in received for name in self.joint_names
        ):
            return

        values = [
            float(received[name])
            for name in self.joint_names
        ]

        if not all(math.isfinite(value) for value in values):
            return

        self.positions = {
            name: float(received[name])
            for name in self.joint_names
        }

    def request_command(self):

        # Verifica che il servizio sia disponibile.
        if not self.command_client.service_is_ready():
            return

        # Memorizza la configurazione iniziale.
        self.initial_positions = dict(self.positions)

        request = GenerateCommand.Request()
        request.mode = 'tracking'
        request.step_index = 0

        self.command_requested = True
        self.start_time = time.monotonic()

        # Richiesta asincrona al Digital Command Generator.
        future = self.command_client.call_async(request)

        future.add_done_callback(
            self.command_response_callback
        )

        self.get_logger().info(
            'Richiesta di generazione del comando inviata.'
        )

    def command_response_callback(self, future):

        try:
            response = future.result()

        except Exception as error:
            self.get_logger().error(
                f'Errore nella richiesta del comando: {error}'
            )
            self.finish()
            return

        if not response.success:
            self.get_logger().error(
                'Generazione del comando fallita: '
                f'{response.message}'
            )
            self.finish()
            return

        command = response.command

        if len(command.target_positions) != len(self.joint_names):
            self.get_logger().error(
                'Comando non valido: numero di target articolari errato.'
            )
            self.finish()
            return

        if not all(
            math.isfinite(value)
            for value in command.target_positions
        ):
            self.get_logger().error(
                'Comando non valido: presenza di target non finiti.'
            )
            self.finish()
            return

        self.target_positions = {
            name: float(value)
            for name, value in zip(
                self.joint_names,
                command.target_positions,
            )
        }

        self.command_received = True

        self.get_logger().info(
            f'Comando ricevuto dal generatore: {command.command_id}'
        )

        self.get_logger().info(
            'Verifica del raggiungimento dei target...'
        )

        for name in self.monitored_joints:
            self.get_logger().info(
                f'{name}: '
                f'target={self.target_positions[name]:.4f} rad'
            )

    def test_step(self):

        if self.finished:
            return

        now = time.monotonic()

        # Attende il primo stato articolare valido.
        if self.initial_positions is None:
            if len(self.positions) != len(self.joint_names):
                return

            # Attende che il servizio sia disponibile.
            if not self.command_client.service_is_ready():
                return

            self.request_command()
            return

        # Attende la risposta del generatore.
        if not self.command_received:

            elapsed = now - self.start_time

            if elapsed >= self.timeout:
                self.get_logger().error(
                    'Test fallito: timeout durante '
                    "l'attesa del comando."
                )
                self.finish()

            return

        # Calcola gli errori rispetto ai target ricevuti.
        elapsed = now - self.start_time

        errors = {
            name: abs(
                self.target_positions[name] - self.positions[name]
            )
            for name in self.monitored_joints
        }

        for name, error in errors.items():
            self.max_errors[name] = max(
                self.max_errors.get(name, 0.0),
                error,
            )

        all_reached = all(
            error <= self.tolerance
            for error in errors.values()
        )

        if all_reached:

            if self.stable_since is None:
                self.stable_since = now

            elif now - self.stable_since >= self.stable_duration:

                self.save_results(
                    success=True,
                    elapsed=elapsed,
                )

                self.finish()
                return

        else:
            self.stable_since = None

        if elapsed >= self.timeout:

            self.get_logger().error(
                f'Test fallito: timeout dopo {elapsed:.2f} secondi.'
            )

            self.save_results(
                success=False,
                elapsed=elapsed,
            )

            self.finish()

    def save_results(self, success, elapsed):

        results_dir = (
            Path.home() / 'franka_digital_twin' / 'results'
        )

        results_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        output_file = (
            results_dir / f'tracking_test_{timestamp}.csv'
        )

        with output_file.open(
            'w',
            newline='',
            encoding='utf-8',
        ) as file:

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

            for name in self.monitored_joints:

                final = self.positions.get(
                    name,
                    float('nan'),
                )

                target = self.target_positions[name]

                error = (
                    abs(target - final)
                    if math.isfinite(final)
                    else float('nan')
                )

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

        self.get_logger().info(
            f'Risultati salvati in: {output_file}'
        )

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
            rclpy.spin_once(
                node,
                timeout_sec=0.1,
            )

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
