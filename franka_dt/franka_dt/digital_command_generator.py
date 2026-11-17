import math
import time
import uuid

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from franka_dt_interfaces.msg import DigitalTwinCommand
from franka_dt_interfaces.srv import GenerateCommand


class DigitalCommandGenerator(Node):

    def __init__(self):
        super().__init__('digital_command_generator')

        # Nomi dei sette giunti del Franka FR3.
        self.joint_names = [
            'fr3_joint1',
            'fr3_joint2',
            'fr3_joint3',
            'fr3_joint4',
            'fr3_joint5',
            'fr3_joint6',
            'fr3_joint7',
        ]

        # Limiti articolari del FR3, espressi in radianti.
        self.limits = {
            'fr3_joint1': (-2.9007, 2.9007),
            'fr3_joint2': (-1.8361, 1.8361),
            'fr3_joint3': (-2.9007, 2.9007),
            'fr3_joint4': (-3.0770, -0.1169),
            'fr3_joint5': (-2.8763, 2.8763),
            'fr3_joint6': (0.4398, 4.6216),
            'fr3_joint7': (-3.0508, 3.0508),
        }

        # Offset utilizzati nella modalità tracking.
        self.tracking_offsets = {
            'fr3_joint2': 0.03,
            'fr3_joint4': 0.03,
            'fr3_joint7': -0.03,
        }

        # Offset dei tre movimenti della modalità full_sequence.
        # Tutti gli offset sono riferiti alla configurazione iniziale.
        self.sequence_offsets = [
            [0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02],
            [-0.02, -0.02, -0.02, -0.02, -0.02, -0.02, -0.02],
            [0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01],
        ]

        # Stato della sequenza attualmente in corso.
        self.sequence_initial_positions = None
        self.next_sequence_step = 0

        # Tempo massimo di validità dello stato ricevuto.
        self.declare_parameter('state_timeout', 0.5)
        self.state_timeout = float(
            self.get_parameter('state_timeout').value
        )

        if (
            not math.isfinite(self.state_timeout)
            or self.state_timeout <= 0
        ):
            raise ValueError(
                'state_timeout deve essere finito e maggiore di zero.'
            )

        # Ultimo stato valido del robot.
        self.current_positions = None
        self.state_receipt_time = None

        # Publisher dei comandi destinati al Command Manager.
        self.command_publisher = self.create_publisher(
            DigitalTwinCommand,
            '/digital_twin/command_positions',
            10,
        )

        # Subscriber agli stati articolari del robot fisico.
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
        )

        # Servizio per richiedere la generazione di un comando.
        self.generate_command_service = self.create_service(
            GenerateCommand,
            '/digital_twin/generate_command',
            self.generate_command_callback,
        )

        self.get_logger().info(
            'Digital Command Generator avviato.'
        )

        self.get_logger().info(
            'In attesa di uno stato articolare valido su /joint_states.'
        )

    def joint_state_callback(self, msg):

        # Verifica la corrispondenza tra nomi e posizioni.
        if len(msg.name) != len(msg.position):
            return

        # Evita nomi di giunti duplicati.
        if len(set(msg.name)) != len(msg.name):
            return

        received = dict(zip(msg.name, msg.position))

        # Verifica che siano presenti tutti i sette giunti.
        if not all(
            name in received for name in self.joint_names
        ):
            return

        positions = {
            name: float(received[name])
            for name in self.joint_names
        }

        # Verifica che tutte le posizioni siano numeriche e finite.
        if not all(
            math.isfinite(value)
            for value in positions.values()
        ):
            return

        # Aggiorna lo stato soltanto dopo aver superato i controlli.
        self.current_positions = positions
        self.state_receipt_time = time.monotonic()

    def generate_command_callback(self, request, response):

        mode = request.mode.strip().lower()
        step_index = request.step_index

        # Verifica che la modalità richiesta sia supportata.
        if mode not in ('tracking', 'full_sequence'):
            response.success = False
            response.message = (
                f'Modalità non supportata: {mode}. '
                'Modalità disponibili: tracking e full_sequence.'
            )
            return response

        # Verifica che sia stato ricevuto uno stato valido.
        if (
            self.current_positions is None
            or self.state_receipt_time is None
        ):
            response.success = False
            response.message = (
                'Impossibile generare il comando: '
                'nessuno stato articolare valido ricevuto.'
            )
            return response

        # Verifica che lo stato non sia obsoleto.
        state_age = (
            time.monotonic() - self.state_receipt_time
        )

        if state_age > self.state_timeout:
            response.success = False
            response.message = (
                'Impossibile generare il comando: '
                f'ultimo stato ricevuto {state_age:.3f} s fa, '
                f'limite massimo {self.state_timeout:.3f} s.'
            )
            return response

        # Acquisisce una copia dello stato corrente.
        current_positions = dict(self.current_positions)

        # --------------------------------------------------
        # MODALITÀ TRACKING
        # --------------------------------------------------

        if mode == 'tracking':

            if step_index != 0:
                response.success = False
                response.message = (
                    'Per la modalità tracking step_index deve essere 0.'
                )
                return response

            initial_positions = current_positions

            target_positions = dict(initial_positions)

            for name, offset in self.tracking_offsets.items():
                target_positions[name] += offset

        # --------------------------------------------------
        # MODALITÀ FULL SEQUENCE
        # --------------------------------------------------

        else:

            if step_index >= len(self.sequence_offsets):
                response.success = False
                response.message = (
                    f'Step non valido: {step_index}. '
                    f'La sequenza contiene {len(self.sequence_offsets)} step.'
                )
                return response

            # Lo step 0 avvia una nuova sequenza.
            if step_index == 0:

                initial_positions = current_positions

            else:

                # Gli step successivi richiedono una sequenza già iniziata.
                if self.sequence_initial_positions is None:
                    response.success = False
                    response.message = (
                        'Impossibile generare il comando: '
                        'nessuna sequenza inizializzata. '
                        'Richiedere prima lo step 0.'
                    )
                    return response

                # Impedisce di saltare o ripetere gli step.
                if step_index != self.next_sequence_step:
                    response.success = False
                    response.message = (
                        f'Step non previsto: {step_index}. '
                        f'Step atteso: {self.next_sequence_step}.'
                    )
                    return response

                # Recupera sempre la configurazione iniziale.
                initial_positions = dict(
                    self.sequence_initial_positions
                )

            offsets = self.sequence_offsets[step_index]

            target_positions = {
                name: initial_positions[name] + offset
                for name, offset in zip(
                    self.joint_names,
                    offsets,
                )
            }

        # --------------------------------------------------
        # CONTROLLO DEI LIMITI ARTICOLARI
        # --------------------------------------------------

        for name, target in target_positions.items():

            lower, upper = self.limits[name]

            if not lower <= target <= upper:

                response.success = False
                response.message = (
                    f'Target non valido per {name}: '
                    f'{target:.4f} rad. '
                    f'Limiti consentiti: [{lower}, {upper}] rad.'
                )

                self.get_logger().error(response.message)
                return response

        # --------------------------------------------------
        # CREAZIONE E PUBBLICAZIONE DEL COMANDO
        # --------------------------------------------------

        command = DigitalTwinCommand()

        command.command_id = str(uuid.uuid4())

        command.target_positions = [
            float(target_positions[name])
            for name in self.joint_names
        ]

        # Registra t_DT' immediatamente prima della pubblicazione.
        command.emission_timestamp = (
            self.get_clock().now().to_msg()
        )

        self.command_publisher.publish(command)

        # Aggiorna lo stato della sequenza solo dopo
        # aver generato e pubblicato correttamente il comando.
        if mode == 'full_sequence':

            if step_index == 0:
                self.sequence_initial_positions = dict(
                    initial_positions
                )

            self.next_sequence_step = step_index + 1

        response.success = True
        response.message = (
            f'Comando {mode} generato e pubblicato. '
            f'Step: {step_index}.'
        )
        response.command = command

        self.get_logger().info(
            f'Comando {mode} generato: {command.command_id}'
        )

        for name, target in zip(
            self.joint_names,
            command.target_positions,
        ):
            self.get_logger().info(
                f'{name}: target={target:.4f} rad'
            )

        return response


def main(args=None):

    rclpy.init(args=args)

    node = DigitalCommandGenerator()

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
