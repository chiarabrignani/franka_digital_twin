import csv
from datetime import datetime
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from franka_dt_interfaces.srv import GenerateCommand


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

        self.tolerance = 0.01

        # Timeout per le diverse fasi.
        self.timeout = 10.0
        self.service_wait_timeout = 10.0

        self.stable_duration = 0.5

        # Stati articolari del robot e del Digital Twin.
        self.robot_positions = None
        self.digital_positions = None

        # Configurazione iniziale.
        self.initial_positions = None

        # Target restituito dal generatore.
        self.current_target = None

        # Risultati sperimentali.
        self.results = []

        # Gestione della sequenza.
        self.step_index = -1
        self.pending_step_index = None

        self.command_pending = False
        self.command_received = False

        self.step_request_time = None
        self.step_start_time = None
        self.command_wait_start_time = None
        self.stable_since = None

        self.finished = False
        self.failed = False

        # Subscriber agli stati del robot fisico.
        self.robot_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.robot_state_callback,
            10,
        )

        # Subscriber agli stati previsti dal Digital Twin.
        self.digital_subscription = self.create_subscription(
            JointState,
            '/digital_twin/predicted_joint_states',
            self.digital_state_callback,
            10,
        )

        # Client del servizio di generazione dei comandi.
        self.command_client = self.create_client(
            GenerateCommand,
            '/digital_twin/generate_command',
        )

        self.timer = self.create_timer(
            0.05,
            self.test_step,
        )

        self.get_logger().info(
            'Full Sequence Test avviato: attesa degli stati '
            'del robot e del Digital Twin.'
        )

    def extract_positions(self, msg):

        if len(msg.name) != len(msg.position):
            return None

        if len(set(msg.name)) != len(msg.name):
            return None

        received = dict(zip(msg.name, msg.position))

        if not all(
            name in received for name in self.joint_names
        ):
            return None

        positions = {
            name: float(received[name])
            for name in self.joint_names
        }

        if not all(
            math.isfinite(value)
            for value in positions.values()
        ):
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

    def request_command(self):

        if not self.command_client.service_is_ready():
            return

        requested_step = self.step_index + 1

        request = GenerateCommand.Request()
        request.mode = 'full_sequence'
        request.step_index = requested_step

        self.pending_step_index = requested_step
        self.command_pending = True
        self.command_received = False

        self.step_request_time = time.monotonic()

        future = self.command_client.call_async(request)

        future.add_done_callback(
            self.command_response_callback
        )

        self.get_logger().info(
            f'Richiesta dello step {requested_step} '
            f'inviata al Digital Command Generator.'
        )

    def command_response_callback(self, future):

        if self.finished:
            return

        try:
            response = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Errore nella richiesta del comando: {error}'
            )

            self.failed = True
            self.finish_test()
            return

        if not response.success:

            self.get_logger().error(
                'Generazione del comando fallita: '
                f'{response.message}'
            )

            self.failed = True
            self.finish_test()
            return

        command = response.command

        if len(command.target_positions) != len(self.joint_names):

            self.get_logger().error(
                'Comando non valido: numero di target articolari errato.'
            )

            self.failed = True
            self.finish_test()
            return

        if not all(
            math.isfinite(value)
            for value in command.target_positions
        ):

            self.get_logger().error(
                'Comando non valido: presenza di target non finiti.'
            )

            self.failed = True
            self.finish_test()
            return

        # Acquisisce i target restituiti dal generatore.
        self.current_target = {
            name: float(value)
            for name, value in zip(
                self.joint_names,
                command.target_positions,
            )
        }

        self.step_index = self.pending_step_index

        self.command_pending = False
        self.command_received = True

        # Il movimento inizia quando riceviamo il comando.
        # L'attesa del servizio non viene conteggiata.
        self.step_start_time = time.monotonic()
        self.command_wait_start_time = None

        self.stable_since = None

        self.get_logger().info(
            f'Comando ricevuto: {command.command_id}'
        )

        self.get_logger().info(
            f'Avvio movimento {self.step_index + 1}/3.'
        )

        for name in self.joint_names:

            self.get_logger().info(
                f'{name}: '
                f'target={self.current_target[name]:.4f} rad'
            )

    def test_step(self):

        if self.finished:
            return

        now = time.monotonic()

        # --------------------------------------------------
        # ATTESA DEGLI STATI INIZIALI
        # --------------------------------------------------

        if self.initial_positions is None:

            if (
                self.robot_positions is None
                or self.digital_positions is None
            ):
                return

            # Acquisisce la configurazione iniziale del robot.
            self.initial_positions = dict(
                self.robot_positions
            )

            # Verifica l'allineamento iniziale.
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

                self.failed = True
                self.finish_test()
                return

            self.get_logger().info(
                'Stato iniziale valido. '
                'Attesa del servizio di generazione comandi.'
            )

            self.command_wait_start_time = time.monotonic()
            self.request_command()

            return

        # --------------------------------------------------
        # ATTESA DELLA RISPOSTA DEL GENERATORE
        # --------------------------------------------------

        if self.command_pending:

            elapsed = now - self.step_request_time

            if elapsed >= self.service_wait_timeout:

                self.get_logger().error(
                    f'Timeout durante la richiesta '
                    f'dello step {self.pending_step_index}.'
                )

                self.failed = True
                self.finish_test()

            return

        # Se non abbiamo ancora ricevuto il comando,
        # riprova a richiederlo quando il servizio è disponibile.
        if not self.command_received:

            if self.command_wait_start_time is None:
                self.command_wait_start_time = now

            elapsed = now - self.command_wait_start_time

            if elapsed >= self.service_wait_timeout:

                self.get_logger().error(
                    'Timeout: servizio di generazione comandi '
                    'non disponibile.'
                )

                self.failed = True
                self.finish_test()
                return

            if not self.command_pending:
                self.request_command()

            return

        # --------------------------------------------------
        # MONITORAGGIO DEL MOVIMENTO
        # --------------------------------------------------

        elapsed = now - self.step_start_time

        robot_errors = {
            name: abs(
                self.current_target[name]
                - self.robot_positions[name]
            )
            for name in self.joint_names
        }

        digital_errors = {
            name: abs(
                self.current_target[name]
                - self.digital_positions[name]
            )
            for name in self.joint_names
        }

        # Verifica il raggiungimento del target fisico.
        robot_reached = all(
            error <= self.tolerance
            for error in robot_errors.values()
        )

        # Verifica il raggiungimento del target digitale.
        digital_reached = all(
            error <= self.tolerance
            for error in digital_errors.values()
        )

        # Verifica l'allineamento tra i due sistemi.
        states_aligned = all(
            abs(
                self.robot_positions[name]
                - self.digital_positions[name]
            ) <= self.tolerance
            for name in self.joint_names
        )

        # Entrambi i sistemi devono raggiungere il target
        # e rimanere allineati per il tempo stabilito.
        if robot_reached and digital_reached and states_aligned:

            if self.stable_since is None:
                self.stable_since = now

            elif now - self.stable_since >= self.stable_duration:

                self.save_step_result(
                    success=True,
                    elapsed=elapsed,
                    robot_errors=robot_errors,
                    digital_errors=digital_errors,
                )

                self.get_logger().info(
                    f'Movimento {self.step_index + 1}/3 SUPERATO.'
                )

                self.command_received = False
                self.command_wait_start_time = time.monotonic()

                # Se abbiamo completato tutti e tre i movimenti,
                # termina il test.
                if self.step_index == 2:

                    self.finish_test()

                else:

                    # Richiede il movimento successivo.
                    self.request_command()

                return

        else:
            self.stable_since = None

        # --------------------------------------------------
        # CONTROLLO DEL TIMEOUT
        # --------------------------------------------------

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

    def save_step_result(
        self,
        success,
        elapsed,
        robot_errors,
        digital_errors,
    ):

        for name in self.joint_names:

            self.results.append({

                'step': self.step_index + 1,

                'joint': name,

                'initial_position_rad':
                    self.initial_positions[name],

                'target_position_rad':
                    self.current_target[name],

                'robot_position_rad':
                    self.robot_positions[name],

                'digital_position_rad':
                    self.digital_positions[name],

                'robot_error_rad':
                    robot_errors[name],

                'digital_error_rad':
                    digital_errors[name],

                'states_aligned': (
                    abs(
                        self.robot_positions[name]
                        - self.digital_positions[name]
                    ) <= self.tolerance
                ),

                'test_success': success,

                'elapsed_time_s': elapsed,
            })

    def finish_test(self):

        self.save_csv()
        self.finish()

    def save_csv(self):

        results_dir = (
            Path.home()
            / 'franka_digital_twin'
            / 'results'
        )

        results_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        output_file = (
            results_dir
            / f'full_sequence_test_{timestamp}.csv'
        )

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
            'w',
            newline='',
            encoding='utf-8',
        ) as file:

            writer = csv.DictWriter(
                file,
                fieldnames=fields,
            )

            writer.writeheader()

            for row in self.results:

                formatted = dict(row)

                for field in fields:

                    if (
                        field.endswith('_rad')
                        or field == 'elapsed_time_s'
                    ):

                        formatted[field] = (
                            f'{row[field]:.6f}'
                        )

                writer.writerow(formatted)

        self.get_logger().info(
            f'Risultati salvati in: {output_file}'
        )

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
