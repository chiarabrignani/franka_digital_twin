import json
import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import String

from rclpy.clock import Clock
from rclpy.clock import ClockType


class DigitalModel(Node):

    def __init__(self):
        super().__init__('digital_model')

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

        # Parametri del modello.
        self.declare_parameter('max_joint_velocity', 0.5)
        self.declare_parameter('max_joint_acceleration', 150.0)
        self.declare_parameter('max_joint_deceleration', 140.0)
        self.declare_parameter('publish_rate', 50.0)

        self.max_joint_velocity = float(
            self.get_parameter('max_joint_velocity').value
        )
        self.max_joint_acceleration = float(
            self.get_parameter('max_joint_acceleration').value
        )
        self.max_joint_deceleration = float(
            self.get_parameter('max_joint_deceleration').value
        )

        self.publish_rate = float(
            self.get_parameter('publish_rate').value
        )

        if (
            not math.isfinite(self.max_joint_velocity)
            or self.max_joint_velocity <= 0
        ):
            raise ValueError(
                'max_joint_velocity deve essere finita e positiva.'
            )

        if (
            not math.isfinite(self.max_joint_acceleration)
            or self.max_joint_acceleration <= 0
        ):
            raise ValueError(
                'max_joint_acceleration deve essere finita e positiva.'
            )

        if (
            not math.isfinite(self.max_joint_deceleration)
            or self.max_joint_deceleration <= 0
        ):
            raise ValueError(
                'max_joint_deceleration deve essere finita e positiva.'
            )

        if (
            not math.isfinite(self.publish_rate)
            or self.publish_rate <= 0
        ):
            raise ValueError(
                'publish_rate deve essere finita e positiva.'
            )

        # Stato interno autonomo del Digital Model.
        self.initialized = False
        self.predicted_positions = None
        self.predicted_velocities = [0.0] * 7

        # Informazioni sulla traiettoria corrente.
        self.trajectory_start_positions = None
        self.trajectory_target_positions = None
        self.trajectory_start_time = None
        self.trajectory_duration = 0.0
        self.trajectory_distance = 0.0
        self.trajectory_peak_velocity = 0.0
        self.trajectory_acceleration_distance = 0.0
        self.trajectory_cruise_distance = 0.0
        self.trajectory_acceleration_time = 0.0
        self.trajectory_cruise_time = 0.0
        self.trajectory_deceleration_time = 0.0
        self.active_command_id = None

        # Eventuali comandi ricevuti prima dell'inizializzazione.
        self.pending_commands = []

        # Stato fisico separato dallo stato predetto.
        self.physical_positions = None
        self.physical_state_count = 0
        self.last_physical_state_time = None

        # Sottoscrizione persistente agli aggiornamenti PT.
        self.physical_state_subscription = self.create_subscription(
            JointState,
            '/digital_twin/joint_states',
            self.physical_state_callback,
            10,
        )

        # Sottoscrizione ai soli comandi accettati.
        self.command_subscription = self.create_subscription(
            String,
            '/digital_twin/accepted_command',
            self.command_callback,
            10,
        )

        # Pubblicazione dello stato articolare previsto.
        self.predicted_state_publisher = self.create_publisher(
            JointState,
            '/digital_twin/predicted_joint_states',
            10,
        )

        # Evento pubblicato dopo l'elaborazione degli aggiornamenti PT.
        self.state_processed_publisher = self.create_publisher(
            String,
            '/digital_twin/state_processed',
            10,
        )

        # Timer per l'evoluzione e la pubblicazione del modello.
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.update_model,
            clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

        self.get_logger().info(
            'Digital Model avviato. '
            'In attesa dello stato iniziale del FR3.'
        )

    def physical_state_callback(self, msg):
        """Riceve continuamente lo stato PT e registra l'elaborazione DT."""

        callback_start = time.monotonic()

        if len(msg.name) != len(msg.position):
            return

        positions = dict(zip(msg.name, msg.position))

        if not all(joint in positions for joint in self.joint_names):
            return

        physical_positions = [
            float(positions[joint])
            for joint in self.joint_names
        ]

        if not all(math.isfinite(value) for value in physical_positions):
            return

        # Memorizza lo stato fisico senza modificare la traiettoria predetta.
        self.physical_positions = physical_positions
        self.physical_state_count += 1

        receipt_time = time.monotonic()
        receipt_ros = self.get_clock().now().nanoseconds / 1e9
        self.last_physical_state_time = receipt_time

        # Il primo aggiornamento valido inizializza il modello.
        if not self.initialized:
            self.predicted_positions = list(physical_positions)
            self.predicted_velocities = [0.0] * 7
            self.initialized = True

            self.get_logger().info(
                'Digital Model inizializzato correttamente.'
            )

            for joint, position in zip(
                self.joint_names, physical_positions
            ):
                self.get_logger().info(
                    f'{joint}: posizione iniziale = {position:.4f} rad'
                )

        # Registra il completamento dell'elaborazione dell'aggiornamento.
        processed_time = time.monotonic()

        source_timestamp = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

        # Pubblica l'evento solo quando entrambi i timestamp sono validi.
        if receipt_ros > 0 and source_timestamp > 0:

            event = {
                'update_id': self.physical_state_count,
                'source_timestamp': source_timestamp,
                'receipt_ros_timestamp': receipt_ros,
                'receipt_monotonic': receipt_time,
                'processed_monotonic': processed_time,
                'processing_time_seconds': processed_time - callback_start,
            }

            event_msg = String()
            event_msg.data = json.dumps(event)
            self.state_processed_publisher.publish(event_msg)

        # Elabora i comandi eventualmente ricevuti prima dell'inizializzazione.
        if self.pending_commands:
            pending = self.pending_commands
            self.pending_commands = []

            for command_data in pending:
                self.process_accepted_command(command_data)

    def command_callback(self, msg):
        """Riceve un comando accettato dal Command Manager."""

        if not self.initialized:
            self.pending_commands.append(msg.data)

            self.get_logger().warning(
                'Comando ricevuto prima dell’inizializzazione. '
                'Sarà elaborato appena disponibile lo stato iniziale.'
            )
            return

        self.process_accepted_command(msg.data)

    def process_accepted_command(self, command_data):
        """Valida il messaggio e avvia una nuova traiettoria."""

        try:
            data = json.loads(command_data)

            command_id = data['command_id']
            target_positions = data['target_positions']

        except (json.JSONDecodeError, KeyError, TypeError):
            self.get_logger().error(
                'Comando accettato non valido: JSON o campi mancanti.'
            )
            return

        if not isinstance(command_id, str) or not command_id:
            self.get_logger().error(
                'Comando ignorato: command_id non valido.'
            )
            return

        if not isinstance(target_positions, list):
            self.get_logger().error(
                'Comando ignorato: target_positions non valido.'
            )
            return

        if len(target_positions) != 7:
            self.get_logger().error(
                'Comando ignorato: sono richieste sette posizioni.'
            )
            return

        try:
            target_positions = [
                float(position)
                for position in target_positions
            ]
        except (ValueError, TypeError):
            self.get_logger().error(
                'Comando ignorato: posizioni non numeriche.'
            )
            return

        if not all(
            math.isfinite(position)
            for position in target_positions
        ):
            self.get_logger().error(
                'Comando ignorato: posizioni non finite.'
            )
            return

        # Aggiorna la previsione fino all'istante corrente.
        now = time.monotonic()
        self.advance_trajectory(now)

        # Il nuovo movimento parte dallo stato previsto corrente,
        # non dallo stato fisico del robot.
        self.trajectory_start_positions = list(
            self.predicted_positions
        )

        self.trajectory_target_positions = target_positions
        self.trajectory_start_time = now
        self.active_command_id = command_id

        displacements = [
            abs(target - start)
            for start, target in zip(
                self.trajectory_start_positions,
                self.trajectory_target_positions,
            )
        ]

        max_displacement = max(displacements)
        self.trajectory_distance = max_displacement

        if max_displacement > 0.0:
            vmax = self.max_joint_velocity
            acceleration = self.max_joint_acceleration
            deceleration = self.max_joint_deceleration

            acceleration_distance = vmax ** 2 / (2.0 * acceleration)
            deceleration_distance = vmax ** 2 / (2.0 * deceleration)

            if max_displacement >= acceleration_distance + deceleration_distance:
                # Profilo trapezoidale: accelerazione, regime, decelerazione.
                peak_velocity = vmax
                acceleration_time = vmax / acceleration
                deceleration_time = vmax / deceleration

                cruise_distance = (
                    max_displacement
                    - acceleration_distance
                    - deceleration_distance
                )
                cruise_time = cruise_distance / vmax

            else:
                # Profilo triangolare: il movimento è troppo breve
                # per raggiungere la velocità massima.
                peak_velocity = math.sqrt(
                    2.0 * max_displacement * acceleration * deceleration
                    / (acceleration + deceleration)
                )

                acceleration_time = peak_velocity / acceleration
                deceleration_time = peak_velocity / deceleration

                acceleration_distance = (
                    peak_velocity ** 2 / (2.0 * acceleration)
                )
                cruise_distance = 0.0
                cruise_time = 0.0

            self.trajectory_peak_velocity = peak_velocity
            self.trajectory_acceleration_distance = acceleration_distance
            self.trajectory_cruise_distance = cruise_distance
            self.trajectory_acceleration_time = acceleration_time
            self.trajectory_cruise_time = cruise_time
            self.trajectory_deceleration_time = deceleration_time

            self.trajectory_duration = (
                acceleration_time + cruise_time + deceleration_time
            )
        else:
            self.trajectory_duration = 0.0

        if self.trajectory_duration == 0.0:
            self.predicted_positions = list(target_positions)
            self.predicted_velocities = [0.0] * 7
            self.clear_trajectory()

        self.get_logger().info(
            f'Avviata traiettoria per comando {command_id}.'
        )

        self.get_logger().info(
            f'Durata prevista: {self.trajectory_duration:.3f} s'
        )

    def advance_trajectory(self, now):
        """Aggiorna lo stato previsto usando un profilo trapezoidale."""

        if self.trajectory_start_time is None:
            return

        elapsed = now - self.trajectory_start_time

        ta = self.trajectory_acceleration_time
        tc = self.trajectory_cruise_time
        td = self.trajectory_deceleration_time

        a = self.max_joint_acceleration
        d = self.max_joint_deceleration
        vp = self.trajectory_peak_velocity

        da = self.trajectory_acceleration_distance
        dc = self.trajectory_cruise_distance

        if elapsed < ta:
            # Fase di accelerazione.
            distance = 0.5 * a * elapsed ** 2
            scalar_velocity = a * elapsed

        elif elapsed < ta + tc:
            # Fase a velocità costante.
            cruise_elapsed = elapsed - ta
            distance = da + vp * cruise_elapsed
            scalar_velocity = vp

        elif elapsed < self.trajectory_duration:
            # Fase di decelerazione.
            deceleration_elapsed = elapsed - ta - tc
            distance = (
                da + dc
                + vp * deceleration_elapsed
                - 0.5 * d * deceleration_elapsed ** 2
            )
            scalar_velocity = max(
                0.0, vp - d * deceleration_elapsed
            )

        else:
            distance = self.trajectory_distance
            scalar_velocity = 0.0

        alpha = min(
            max(distance / self.trajectory_distance, 0.0),
            1.0
        )

        self.predicted_positions = [
            start + alpha * (target - start)
            for start, target in zip(
                self.trajectory_start_positions,
                self.trajectory_target_positions,
            )
        ]

        self.predicted_velocities = [
            (target - start) / self.trajectory_distance * scalar_velocity
            for start, target in zip(
                self.trajectory_start_positions,
                self.trajectory_target_positions,
            )
        ]

        if elapsed >= self.trajectory_duration:
            self.predicted_positions = list(
                self.trajectory_target_positions
            )
            self.predicted_velocities = [0.0] * 7

            actual_duration = elapsed

            self.get_logger().info(
                f'Traiettoria completata per comando '
                f'{self.active_command_id}.'
            )
            self.get_logger().info(
                f'Durata effettiva traiettoria DT: '
                f'{actual_duration:.3f} s'
            )
            self.get_logger().info(
                f'Durata prevista traiettoria DT: '
                f'{self.trajectory_duration:.3f} s'
            )

            self.clear_trajectory()

    def clear_trajectory(self):
        """Azzera i dati della traiettoria terminata."""

        self.trajectory_start_positions = None
        self.trajectory_target_positions = None
        self.trajectory_start_time = None
        self.trajectory_duration = 0.0
        self.trajectory_distance = 0.0
        self.trajectory_peak_velocity = 0.0
        self.trajectory_acceleration_distance = 0.0
        self.trajectory_cruise_distance = 0.0
        self.trajectory_acceleration_time = 0.0
        self.trajectory_cruise_time = 0.0
        self.trajectory_deceleration_time = 0.0
        self.active_command_id = None

    def update_model(self):
        """Aggiorna e pubblica periodicamente lo stato previsto."""

        if not self.initialized:
            return

        self.advance_trajectory(time.monotonic())

        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = list(self.predicted_positions)
        msg.velocity = list(self.predicted_velocities)

        self.predicted_state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = DigitalModel()

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
